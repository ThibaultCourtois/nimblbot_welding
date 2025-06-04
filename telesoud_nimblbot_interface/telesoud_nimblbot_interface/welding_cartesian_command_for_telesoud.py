import time
from math import pi
import numpy as np
from scipy.spatial.transform import Slerp
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.executors import ShutdownException
from rclpy.parameter import Parameter as RclpyParameter
from rcl_interfaces.msg import ParameterValue, Parameter, ParameterType
from std_srvs.srv import Empty as Empty_srv
from std_msgs.msg import Empty, Header, Float64, Bool, String, Int8MultiArray
from moveit_msgs.srv import ServoCommandType
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, Twist 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf_transformations import quaternion_from_euler, quaternion_multiply
from scipy.spatial.transform import Rotation as R
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rcl_interfaces.srv import GetParameters, SetParameters
from nimblpy.common.robot_loader import load_robot_config
from nimblpy.kinematics.kin_model import KinematicModel
from telesoud_msgs.msg import RobotData, Command, CommandStatus

SERVO_NODE = '/servo_node'

ROBOT_RATE = 20.0                   # NimblBot input command rate
TELESOUD_RATE = 40.0                # Telesoud output command rate
TIMER_PERIOD = 0.025                 # 40 Hz = Telesoud ouput command rate  

STOP_COMMAND_THRESHOLD = 2400       # ~60 sec at 40 Hz -- delay before automatic safety pause
POSITION_EPSILON = 0.001            # 1mm 

SPEED_CORRECTION_FACTOR = 1.0      # Correction factor for TCP_speed
DEFAULT_TCP_SPEED = 0.005

SERVICE_TIMEOUT = 5.0

MIN_MODULAR_VELOCITY = 0.05
MAX_MODULAR_VELOCITY = 0.5
MODULAR_VELOCITY_STEP = 0.05
DEFAULT_MODULAR_SPEED = 0.3 

# Modular control
AZIMUTH = 0
TILT = 1
MODULE_SELECT_COMMAND = 0
SECTION_SELECT_COMMAND = 1
ALL_SELECT_COMMAND = 2

class ControlMode:
    PAUSE: int=0
    MODULAR: int=1
    CARTESIAN : int=2

class RobotState:
    """Enum-like class for robot state machine"""
    IDLE: int = 0
    DYNAMIC_MOVEMENT: int = 1
    CARTESIAN_TRAJECTORY: int = 2
    JOINT_TRAJECTORY: int = 3
    MODULAR_CONTROL: int = 4
    PAUSE: int = 5


STATE_NAMES = {
    RobotState.IDLE: "IDLE",
    RobotState.DYNAMIC_MOVEMENT: "DYNAMIC_MOVEMENT",
    RobotState.CARTESIAN_TRAJECTORY: "CARTESIAN_TRAJECTORY",
    RobotState.JOINT_TRAJECTORY: "JOINT_TRAJECTORY",
    RobotState.MODULAR_CONTROL: "MODULAR_CONTROL",
    RobotState.PAUSE: "PAUSE"
}


class TelesoudCommandToCartesianNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._initialize_basic_parameters()
        self._setup_transform_infrastructure()
        self._create_publishers()
        self._create_subscriptions()
        self._create_clients()
        self._create_services()

        self.__timer_state_machine = self.create_timer(TIMER_PERIOD, self._update_state_machine)
        self.command_timer = self.create_timer(TIMER_PERIOD, self.process_pending_command)


    def _initialize_basic_parameters(self):
        # Modular control parameters
        self.__q_desired = None
        self.__modular_command_mode = [AZIMUTH, TILT]
        self.__modular_selector_mode = [MODULE_SELECT_COMMAND, SECTION_SELECT_COMMAND]
        self.__modular_selection_module = 0
        self.__modular_selection_section = 0
        self.modular_control_enabled = False
        self.modular_velocity = DEFAULT_MODULAR_SPEED
        self.amplified_modular_velocity = None
        self.joints_trajectory = None
        self.previous_modular_commands = {
            'select_module' : 0,
            'turn_module' : 0,
            'set_speed_module' : 0,
            'select_command_type' : 0,
            'selection_mode' : 0,
            'wrist_command' : 0
        }
        self.current_modular_states = {
                'turn_module' : 0,
                'wrist_command' : 0
        }
        # Command handling
        self.stop_command_counter = 0
        self.last_command_type = None
        self.pending_command = None
        self.command_data = None
        # State machine variables
        self.current_state = RobotState.PAUSE
        self.previous_state = RobotState.PAUSE
        self.current_control_mode = 0
        # Error handling
        self.current_error = ""
        self.robot_in_fault_status = False
        # Telesoud interpolated trajectory execution
        self.virtual_pose = None
        self.virtual_pose_initialized = False
        # Flags
        self.emergency_stop = False
        self.resuming_flag = False
        # Movement  parameters
        self.tcp_pose = None
        self.tcp_speed = DEFAULT_TCP_SPEED
        self.current_target_pose = None
        self.declare_parameter('modular_gain', 1.0)
        self.declare_parameter('pos_gain', 1.0)
        self.declare_parameter('quat_gain', 1.0)
        self.current_speed_vector = Twist()
        # Cartesian movement interpolation
        self.interpolated_line_poses = None
        self.line_progression_index = 0

    
    def _setup_transform_infrastructure(self):
        # Namespace from ID's
        self.namespace = "nb"
        self.__base_frame_robot = f"{self.namespace}/base_link"

        # Get ee_frame name config in servo_node
        self.__ee_frame_mimic: str = self._get_param(SERVO_NODE + '/get_parameters', 'moveit_servo_nb.ee_frame').string_value
        self.__ee_frame_robot: str = self.__ee_frame_mimic.replace("_mimic", "")

        # Load robot configuration
        self.declare_parameter("robot_type", "")
        robot_type = self.get_parameter("robot_type").get_parameter_value().string_value

        try:
            # Load robot configuration
            conf = load_robot_config(robot_type)
            self.__terminal_wrist = conf["robot_configuration"]["terminal_wrist"]
            self.__num_modules = conf["robot_configuration"]["num_modules"]
            self.__num_sections = self.__num_modules // 3
            self.__num_joints = 2 * self.__num_modules + self.__terminal_wrist
            self.__model = KinematicModel(conf, None)
            
            # Get joint names
            alias = conf['low_level_control']['alias']
            self.__joint_names_alias = [alias + str(i) if i != 0 else alias for i in range(self.__num_joints)]

        except Exception as e:
            self.get_logger().fatal(f'robot_type not define for cartesian or modular command : {e}')
            exit()
        
        # Set up transform infrastructure
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)


    def _create_publishers(self):
        # Multiple command publishers
        self.motor_lock_pub = self.create_publisher(
            Int8MultiArray,
            "/nb/motor_lock",
            10
        )

        self.desired_pose_pub = self.create_publisher(
            PoseStamped,
            '/nb/desired_pose',
            10
        )

        self.joints_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/nb/desired_trajectory_modular',
            10
        )
        
        # Telesoud interface publishers
        self.status_pub = self.create_publisher(
            CommandStatus,
            '/welding_command_handler/command_status',
            10
        )  
        
        # For RVIZ plugin panel
        self.robotState_pub = self.create_publisher(
            String,
            '/welding_command_handler/robotState',
            10
        )

        self.modular_mode_pub = self.create_publisher(
            String, 
            '/welding_command_handler/modular_mode',
            10
        )

        self.modular_velocity_indicator_pub = self.create_publisher(
            Float64,
            '/welding_command_handler/modular_velocity_indicator',
            10
        )

        self.tcp_pose_pub = self.create_publisher(
            Pose,
            'welding_command_handler/tcp_pose',
            10
        )

    def _create_subscriptions(self):

        _ = self.create_subscription(
            JointTrajectory,
            "/nb/desired_trajectory",
            self.on_desired_trajectory,
            10
        )

        # Rviz plugin 
        self.pos_gain_sub = self.create_subscription(
            Float64,
            '/rviz_plugin/pos_gain',
            self.on_pos_gain_changed,
            10
        )

        self.quat_gain_sub = self.create_subscription(
            Float64,
            '/rviz_plugin/quat_gain',
            self.on_quat_gain_changed,
            10
        )

        self.modular_gain_sub = self.create_subscription(
            Float64,
            'rviz_plugin/modular_gain',
            self.on_modular_gain_changed,
            10
        )

        self.modular_command_toggle_sub = self.create_subscription(
            Bool, 
            '/rviz_plugin/modular_command_state',
            self.on_modular_command_toggle,
            10
        )

        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/rviz_plugin/emergency_stop',
            self.on_emergency_stop,
            10
        )

        # For TCP trace clearing
        self.clear_tcp_trace_sub = self.create_subscription(
            Empty,
            '/rviz_plugin/clear_tcp_trace',
            self.on_clear_tcp_trace,
            10
        )

        # Telesoud interface subscribers
        self.command_sub = self.create_subscription(
            Command,
            '/translator/command',
            self.on_command,
            10
        )

    
    def _create_clients(self):
        # Multiple command client
        self.change_control_mode_client = self.create_client(
                ServoCommandType,
                '/nb/change_control_mode'
        )

        # Servo node clients
        self.tf_path_trail_nb_base_link_to_nb_tcp_client_clear_path = self.create_client(
            Empty_srv,
            '/tf_path_trail_' + self.__base_frame_robot.replace('/', '_') + '_to_' + self.__ee_frame_robot.replace('/', '_') + '/clear_path'
        )

        self.tf_path_trail_nb_mimic_base_link_to_nb_mimic_tcp_wrist_client_clear_path = self.create_client(
            Empty_srv,
            '/tf_path_trail_' + 'nb_mimic_base_link' + '_to_' + 'nb_mimic_tcp_wrist' + '/clear_path'
        )

        # Luos wrapper node
        self.set_zeros_client = self.create_client(
            Empty_srv,
            "/nb/set_zeros"
        )
     
    
    def _create_services(self):
        # Set zeros protocol
        self.set_zeros_srv = self.create_service(
                Empty_srv, 
                'welding_cartesian_command/set_zeros_request',
                self.on_set_zeros
            )
    

    def _get_param(self, node_name: str, parameters: str) -> ParameterValue:
        """Retrieve the value of a parameter from a specified node."""
        client = self.create_client(GetParameters, node_name)

        while not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info(f'Waiting for {node_name} to get parameter...')

        future = client.call_async(GetParameters.Request(names=[parameters]))

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            try:
                response: GetParameters.Response = future.result()
                if response.values:
                    return response.values[0]
                else:
                    self.get_logger().error("Parameter not found")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
            return ParameterValue()


    def on_command(self, msg):
        self.command_data = {
            'command_type': msg.command_type,
            'command_id': msg.command_id,
            'target_pose': msg.target_pose,
            'speed': msg.speed,
            'speed_vector': msg.speed_vector,
        }
        self.pending_command = True
        return

    
    def process_pending_command(self):
        if not self.pending_command:
            return

        data = self.command_data
        command_type = data['command_type']
        command_id = data['command_id']

        if command_type != 0:
            self.stop_command_counter = 0
            self.last_command_type = command_type

        self.pending_command = False

        status = CommandStatus()
        status.command_id = command_id
        status.command_type = command_type
        
        try: 
            status.robot_data = self._create_robot_data()
        except Exception as e:
            self.get_logger().warn(f'Could not create robot data : {e}')
    
        if self.modular_control_enabled:
            if command_type == 0:
                self.current_speed_vector = Twist()
            if command_type == 7:
                self._process_set_dynamic(status, data['speed_vector'])
            status.success = True
            status.message = "Modular command"
        
        else: 
            try:
                match command_type:
                    case 0:
                        self._process_stop_command(status)
                    case 1:
                        self._process_get_robot_data(status)
                    case 7:
                        self._process_set_dynamic(status, data['speed_vector'])
                    case 8:
                        self._process_start_dynamic(status)
                    case 15:
                        self._process_play_cartesian(status, data['target_pose'], data['speed'])
                    case 16:
                        self._process_play_joint(status, data['target_pose'], data['speed'])
                    case _:
                        status.success = False
                        status.message = f"Unknown command type: {command_type}"
            except Exception as e:
                status.success = False
                status.message = f"Error processing command: {str(e)}"
                self.get_logger().error(f"Error in command processing: {e}")
        
        self.status_pub.publish(status)
        self.tcp_pose_pub.publish(self.tcp_pose)


    def _process_stop_command(self, status):
        self.current_speed_vector = Twist()
        if self.last_command_type == 0:
            self.stop_command_counter += 1
        else:
            self.stop_command_counter = 0
            self.last_command_type = 0

        if self.stop_command_counter >= STOP_COMMAND_THRESHOLD and not self.current_control_mode == ControlMode.PAUSE and not self.emergency_stop:
            self.switch_control_mode(0) # ControlMode.PAUSE
            self.current_state = RobotState.PAUSE
            self.stop_command_counter = 0

        status.success = True
        status.message = "Stop command processed"

    
    def _process_get_robot_data(self, status):
        try:
            robot_data = self._create_robot_data()
            status.robot_data = robot_data
            status.success = True
            status.message = "Robot data retrieved"
        except Exception as e:
            status.success = False
            status.message = f"Error getting robot data: {str(e)}"
    
    
    def _process_set_dynamic(self, status, speed_vector):
        self.current_speed_vector = speed_vector
        status.success = True
        status.message = "Dynamic speed updated"

    
    def _process_start_dynamic(self, status):
        self.switch_control_mode(1) #ControlMode.TELEOP_XYZ
        self.current_state = RobotState.DYNAMIC_MOVEMENT
        status.success = True
        status.message = "Dynamic cartesian movement started"

  
    def _process_play_cartesian(self, status, target_pose, speed):
        self.tcp_speed = speed * SPEED_CORRECTION_FACTOR
        self.current_target_pose = target_pose
        
        self.switch_control_mode(1) #ControlMode.TELEOP_XYZ

        self.interpolated_line_poses = None
        self.line_progression_index = 0
        self.current_state = RobotState.CARTESIAN_TRAJECTORY
        
        status.success = True
        status.message = "Cartesian trajectory started"
   

    def _process_play_joint(self, status, target_pose, speed):
        self.tcp_speed = speed * SPEED_CORRECTION_FACTOR
        self.current_target_pose = target_pose
        
        self.switch_control_mode(1) #ControlMode.TELEOP_XYZ

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.__base_frame_robot
        pose_msg.pose = self.current_target_pose
        
        self.desired_pose_pub.publish(pose_msg)
        
        self.current_state = RobotState.JOINT_TRAJECTORY
        
        status.success = True
        status.message = "Joint trajectory started"

    
    def _update_state_machine(self):
        initial_state = self.current_state
        
        if self.current_state == RobotState.IDLE:
            pass
        
        elif self.current_state == RobotState.PAUSE:
            if self.resuming_flag:
                self.current_state = RobotState.IDLE
                self.get_logger().info('Pause finished, returning to IDLE state')
                self.resuming_flag = False

        elif self.current_state == RobotState.DYNAMIC_MOVEMENT:
            if self._is_dynamic_movement_complete():
                self.get_logger().info('Dynamic movement finished, returning to IDLE state')
                self.current_state = RobotState.IDLE

        elif self.current_state == RobotState.CARTESIAN_TRAJECTORY:
            if self._is_cartesian_trajectory_complete():
                self.get_logger().info('Cartesian trajectory finished, returning to IDLE state')
                self.current_state = RobotState.IDLE    

        elif self.current_state == RobotState.JOINT_TRAJECTORY:
            if self._is_joint_trajectory_complete():
                self.get_logger().info('Joint trajectory finished, returning to IDLE state')
                self.current_state = RobotState.IDLE    
        
        elif self.current_state == RobotState.MODULAR_CONTROL:
            self._process_modular_control()
            if not self.modular_control_enabled:
                self.get_logger().info("Modular control disabled, returning to IDLE state")
                self.current_state = RobotState.IDLE

        if self.current_state != initial_state:
            self.previous_state = initial_state
            self.get_logger().info(f'{STATE_NAMES[self.current_state]}')
        
        robotState_msg = String() 
        robotState_msg.data = STATE_NAMES[self.current_state]
        self.robotState_pub.publish(robotState_msg)
   

    def _is_cartesian_trajectory_complete(self):
        if self.current_state == RobotState.CARTESIAN_TRAJECTORY:
            current_pose = self._get_current_pose(mimic=False)
            if self.stop_command_counter > 0:
                return True
            if current_pose:
                completed = self.execute_line(current_pose.pose)
                if completed:
                    return True
        return False

    
    def _is_joint_trajectory_complete(self):
        if self.stop_command_counter > 0:
            return True
        if self.current_state == RobotState.JOINT_TRAJECTORY:
            try:
                current_pose = self._get_current_pose(mimic=False)
                if current_pose:
                    error = self._get_position_error(current_pose.pose, self.current_target_pose)
                    POSITION_EPSILON = 0.001

                    if error <= POSITION_EPSILON:
                        request = SetParameters.Request()
                        param_value = ParameterValue()
                        param_value.type = ParameterType.PARAMETER_DOUBLE
                        param_value.double_value = 0.025

                        linear_param = Parameter()
                        linear_param.name = 'servo.scale.linear'
                        linear_param.value = param_value

                        request.parameters = [linear_param]
                        
                        self.current_state = RobotState.IDLE
                        return True

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.__base_frame_robot
                pose_msg.pose = self.current_target_pose

                self.desired_pose_pub.publish(pose_msg)
            except Exception as e:
                self.get_logger().error(f'Error during joint trajectory execution {e}')
        return False

    
    def _is_dynamic_movement_complete(self):
        if self.current_state != RobotState.DYNAMIC_MOVEMENT:
            return False
        if self.current_speed_vector != Twist():
            target_pose_msg = self.compute_target_pose()
            if target_pose_msg:
                target_pose_msg.header.stamp = self.get_clock().now().to_msg() 
                self.desired_pose_pub.publish(target_pose_msg)
        if self.stop_command_counter > 0:
            self.current_state = RobotState.IDLE
            self.virtual_pose_initialized = False
            self.virtual_pose = None
            return True
        return False


    def _detect_modular_command_toggle(self, speed_vector):
        # Processing current_speed_vector into modular command instructions
        # Possible values for each instruction : [1, 0, -1]
        current_commands = {
            'select_module': np.sign(speed_vector.linear.x),
            'turn_module': np.sign(speed_vector.linear.y),
            'set_speed_module': np.sign(speed_vector.linear.z),
            'select_command_type': np.sign(speed_vector.angular.x),
            'selection_mode': np.sign(speed_vector.angular.y),
            'wrist_command': np.sign(speed_vector.angular.z)
        }

        toggles = {}

        for command, current_state in current_commands.items():
            previous_state = self.previous_modular_commands[command]
            if previous_state == 0 and current_state != 0:
                toggles[command] = 'rise'
            elif previous_state != 0 and current_state == 0:
                toggles[command] = 'fall'
            elif previous_state != current_state and current_state != 0:
                toggles[command] = 'change'
            else:
                toggles[command] = 'sleep'
        self.previous_modular_commands = current_commands.copy()
        
        for motor_cmd in ['turn_module', 'wrist_command']:
            if toggles[motor_cmd] in ['rise', 'change']:
                self.current_modular_states[motor_cmd] = current_commands[motor_cmd]
            elif toggles[motor_cmd] == 'fall':
                self.current_modular_states[motor_cmd] = 0
        return toggles, current_commands

    
    def _process_modular_control(self):
        if self.__q_desired is None:
            self.get_logger().info('Waiting /nb/desired_trajectory to be populated')
            return
        
        if self.current_speed_vector is None:
            return

        self._handle_modular_commands()
        self._execute_modular_movement()

    
    def _handle_modular_commands(self):
        toggles, current_commands = self._detect_modular_command_toggle(self.current_speed_vector)

        if toggles['select_module'] == 'rise':
            select_module = current_commands['select_module']
            if self.__modular_selector_mode[0] == MODULE_SELECT_COMMAND:
                self.__modular_selection_module = max(0, min(self.__modular_selection_module + select_module, self.__num_modules -1))
                self.__modular_selection_section = max(0, min(self.__modular_selection_module // 3, self.__num_sections - 1))
            elif self.__modular_selector_mode[0] == SECTION_SELECT_COMMAND:
                self.__modular_selection_section = max(0, min(self.__modular_selection_section + select_module, self.__num_sections - 1))
                self.__modular_selection_module = self.__modular_selection_section * 3

        if toggles['set_speed_module'] == 'rise':
            set_speed_module = current_commands['set_speed_module']
            self.modular_velocity = max(MIN_MODULAR_VELOCITY, min(self.modular_velocity + set_speed_module * MODULAR_VELOCITY_STEP, MAX_MODULAR_VELOCITY))
            self.get_logger().info(f"Modular velocity adjusted to: {self.modular_velocity:.2f}")

        if toggles['select_command_type'] == 'rise':
            self.__modular_command_mode = self.__modular_command_mode[1:] + [self.__modular_command_mode[0]]
            modular_mode_msg = String()
            modular_mode_msg.data = 'TILT' if self.__modular_command_mode[0] == TILT else 'AZIMUTH'
            self.modular_mode_pub.publish(modular_mode_msg)
            self.get_logger().info(f"Switched to {'TILT' if self.__modular_command_mode[0] == TILT else 'AZIMUTH'} mode")

        if toggles['selection_mode'] == 'rise':
            self.__modular_selector_mode = self.__modular_selector_mode[1:] + [self.__modular_selector_mode[0]]
            self.get_logger().info(f"Selection mode changed to {'MODULE' if self.__modular_selector_mode[0] == MODULE_SELECT_COMMAND  else 'SECTION'}")

    
    def _execute_modular_movement(self):
        turn_module = self.current_modular_states['turn_module']
        wrist_command = self.current_modular_states['wrist_command']

        modular_gain = self.get_parameter('modular_gain').value
        self.amplified_modular_velocity = modular_gain * self.modular_velocity

        self.joints_trajectory = JointTrajectory()
        self.joints_trajectory.header.stamp = self.get_clock().now().to_msg()
        joint_names = []
        positions = []
        velocities = []

        if self.__modular_selector_mode[0] == MODULE_SELECT_COMMAND:
            joint_inf_idx = int(self.__modular_selection_module) * 2
            joint_sup_idx = joint_inf_idx + 2
        else:
            joint_inf_idx = 6 * int(self.__modular_selection_section)
            joint_sup_idx = joint_inf_idx + 6
            
        module_joint_names = self.__joint_names_alias[joint_inf_idx:joint_sup_idx]
        module_command = turn_module * self.amplified_modular_velocity
        module_velocities = [module_command] * len(module_joint_names)

        if self.__modular_command_mode[0] == TILT:
            # alternate velocity direction for TILT mode (opposing pairs)
            module_velocities = [v if i % 2 == 0 else -v for i, v in enumerate(module_velocities)]

        if turn_module != 0:
            module_positions = [self.__q_desired[joint_inf_idx + i] + v / ROBOT_RATE for i, v in enumerate(module_velocities)] 
        else : 
            module_positions = [self.__q_desired[joint_inf_idx + i] for i in range(len(module_velocities))]

        joint_names.extend(module_joint_names)
        positions.extend(module_positions)
        velocities.extend(module_velocities)

        if self.__terminal_wrist:
            wrist_joint_idx = len(self.__q_desired) - 1
            wrist_joint_name = self.__joint_names_alias[-1]

            if wrist_command != 0:
                wrist_velocity = wrist_command * self.amplified_modular_velocity
                wrist_position = self.__q_desired[wrist_joint_idx] + wrist_velocity / ROBOT_RATE
            else:
                wrist_velocity = 0.0
                wrist_position = self.__q_desired[wrist_joint_idx]

            joint_names.append(wrist_joint_name)
            positions.append(wrist_position)
            velocities.append(wrist_velocity)
        
        if len(joint_names) > 0:
            self.joints_trajectory.joint_names = joint_names
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions
            trajectory_point.velocities = velocities
            self.joints_trajectory.points = [trajectory_point]

            self.joints_trajectory_pub.publish(self.joints_trajectory)
        
        # For visualization purposes
        msg = Float64()
        msg.data = self.amplified_modular_velocity
        self.modular_velocity_indicator_pub.publish(msg)
        
 
    def _get_current_pose(self, mimic: bool, timeout_sec: int = SERVICE_TIMEOUT) -> PoseStamped:
        """Get the current pose of the end-effector in the base frame."""
        ee_frame = self.__ee_frame_mimic if mimic else self.__ee_frame_robot

        try:
            current_pose = self.__tf_buffer.lookup_transform(self.__base_frame_robot,
                                                             ee_frame,
                                                             Time(),
                                                             Duration(seconds=timeout_sec)
                                                             )
        except:
            raise LookupError(f"Waiting transform of {ee_frame}")

        current_pose = PoseStamped(
            header=Header(frame_id=self.__base_frame_robot,
                          stamp=self.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(
                    x=current_pose.transform.translation.x,
                    y=current_pose.transform.translation.y,
                    z=current_pose.transform.translation.z
                ),
                orientation=current_pose.transform.rotation
            )
        )

        return current_pose

    
    def _get_position_error(self, current_pose, target_pose):
        error_vector = [
                    target_pose.position.x - current_pose.position.x,
                    target_pose.position.y - current_pose.position.y,
                    target_pose.position.z - current_pose.position.z
                ]
        return np.linalg.norm(error_vector)


    def compute_target_pose(self):
        if self.current_speed_vector == Twist():
            return None
        
        pos_gain = self.get_parameter('pos_gain').value  
        quat_gain = self.get_parameter('quat_gain').value 
        
        if not self.virtual_pose_initialized:
            current_pose = self._get_current_pose(mimic=False)
            if not current_pose:
                return None
            self.virtual_pose = PoseStamped()
            self.virtual_pose.header = current_pose.header
            self.virtual_pose.pose = Pose(
                position=Point(
                    x=current_pose.pose.position.x,
                    y=current_pose.pose.position.y,
                    z=current_pose.pose.position.z
                ),
                orientation=current_pose.pose.orientation
            )
            self.virtual_pose_initialized = True
        
        working_pose = self.virtual_pose
        dt = 1.0 / TELESOUD_RATE
        
        dx = self.current_speed_vector.linear.x * dt * pos_gain
        dy = self.current_speed_vector.linear.y * dt * pos_gain
        dz = self.current_speed_vector.linear.z * dt * pos_gain
        
        dquat = quaternion_from_euler(
            self.current_speed_vector.angular.x * dt * quat_gain,
            self.current_speed_vector.angular.y * dt * quat_gain,
            self.current_speed_vector.angular.z * dt * quat_gain,
            axes='sxyz'
        )
        
        current_quat = [
            working_pose.pose.orientation.x,
            working_pose.pose.orientation.y,
            working_pose.pose.orientation.z,
            working_pose.pose.orientation.w
        ]
        
        new_quat = quaternion_multiply(current_quat, dquat)
        
        target_pose_msg = PoseStamped()
        target_pose_msg.header.stamp = self.get_clock().now().to_msg()
        target_pose_msg.header.frame_id = self.__base_frame_robot
        
        self.virtual_pose.pose.position.x += dx
        self.virtual_pose.pose.position.y += dy
        self.virtual_pose.pose.position.z += dz
        self.virtual_pose.pose.orientation = Quaternion(
            x=new_quat[0], y=new_quat[1], z=new_quat[2], w=new_quat[3]
        )
        target_pose_msg.pose = Pose(
            position=Point(
                x=self.virtual_pose.pose.position.x,
                y=self.virtual_pose.pose.position.y,
                z=self.virtual_pose.pose.position.z
            ),
            orientation=Quaternion(
                x=self.virtual_pose.pose.orientation.x,
                y=self.virtual_pose.pose.orientation.y,
                z=self.virtual_pose.pose.orientation.z,
                w=self.virtual_pose.pose.orientation.w
            )
        )
        return target_pose_msg

    
    def execute_line(self, current_pose):
        try:
            if self.interpolated_line_poses is None:
                self.interpolated_line_poses = self.generate_interpolated_line(current_pose, self.current_target_pose)

            pose_stamped:PoseStamped = self.interpolated_line_poses[self.line_progression_index]
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.desired_pose_pub.publish(pose_stamped)

            self.line_progression_index = min(self.line_progression_index + 1, len(self.interpolated_line_poses) -1)

            if self.line_progression_index >= len(self.interpolated_line_poses) - 1:
                error = self._get_position_error(current_pose, self.current_target_pose)
                POSITION_EPSILON = 0.001

                if error <= POSITION_EPSILON:
                    self._finalize_movement()
                    self.get_logger().info('Linear welding finished')
                    return True
                else:
                    return False
            return False

        except Exception as e:
                self.get_logger().error(f"Trajectory execution error : {e}")
                return False

    
    def generate_interpolated_line(self, pose1, pose2):
        pos1 = np.array([
                pose1.position.x,
                pose1.position.y,
                pose1.position.z
            ])

        pos2 = np.array([
                pose2.position.x,
                pose2.position.y,
                pose2.position.z
            ])

        self.position_distance = np.linalg.norm(pos1 - pos2)

        quat1 = np.array([
                pose1.orientation.x,
                pose1.orientation.y,
                pose1.orientation.z,
                pose1.orientation.w
                ])

        quat2 = np.array([
                pose2.orientation.x,
                pose2.orientation.y,
                pose2.orientation.z,
                pose2.orientation.w
                ])

        orientation_distance = np.arccos(np.clip(np.abs(np.dot(quat1/np.linalg.norm(quat2), quat2/np.linalg.norm(quat2))), -1.0, 1.0)) / pi

        max_distance = self.tcp_speed * 1/ROBOT_RATE
        num_points = int(max(self.position_distance, orientation_distance) / max_distance + 2)

        rotation_interpolation = Slerp([0, num_points], R.from_quat([list(quat1), list(quat2)]))

        interpolated_line_poses = []
        for increments in range(num_points + 1):
            point = pos1 + (increments/num_points) * (pos2 - pos1)
            quat = rotation_interpolation(increments).as_quat()
            interpolated_line_poses.append(PoseStamped(header=Header(frame_id=self.__base_frame_robot),
                                          pose=Pose(position=Point(x=point[0],
                                                                   y=point[1],
                                                                   z=point[2]),
                                                    orientation=Quaternion(x=quat[0],
                                                                           y=quat[1],
                                                                           z=quat[2],
                                                                           w=quat[3])
                                                    )
                                            )
                                )
        return interpolated_line_poses


    def _finalize_movement(self):
        """Finalize a movement and publish status."""
        status_msg = CommandStatus()
        status_msg.command_type = Command.COMMAND_PLAY_CARTESIAN if self.current_state == RobotState.CARTESIAN_TRAJECTORY else Command.COMMAND_PLAY_JOINT
        status_msg.success = True
        status_msg.message = 'Movement completed successfully'
        status_msg.robot_data = self._create_robot_data()

        self.status_pub.publish(status_msg)
        self.get_logger().info('Movement finished')

    
    def _create_robot_data(self):
        robot_data = RobotData()
        current_pose = self._get_current_pose(mimic=False)
        robot_data.pose = current_pose.pose
        self.tcp_pose = current_pose.pose
        if self.current_state == RobotState.DYNAMIC_MOVEMENT and self.virtual_pose_initialized:
            robot_data.pose = self.virtual_pose.pose
        robot_data.robot_in_fault_status = self.robot_in_fault_status
        robot_data.error_message = self.current_error
        robot_data.timestamp = self.get_clock().now().to_msg()
        return robot_data
    

    def switch_control_mode(self, mode):
        try:
            if self.current_control_mode == mode:
                return
            
            self.resuming_flag = (self.current_control_mode == ControlMode.PAUSE and mode != ControlMode.PAUSE) 
            self.modular_control_enabled &= (mode != ControlMode.MODULAR)
            self.current_control_mode = mode

            if not self.change_control_mode_client.wait_for_service(timeout_sec = SERVICE_TIMEOUT):
                raise RuntimeError("Failed to switch control mode: service not available")

            request = ServoCommandType.Request(command_type = mode)
            future = self.change_control_mode_client.call_async(request)

            self.get_logger().info(f'Switched control mode to {mode}')
            
            time.sleep(0.5)
            self.motor_lock_pub.publish(Int8MultiArray())
            return future
        
        except Exception as e:
            self.current_error = str(e)
            self.get_logger().error(f'{e}')
            return None

   
    def on_set_zeros(self, request, response):
        try:
            if self.current_state != RobotState.MODULAR_CONTROL:
                self.get_logger().error("Cannot set zeros: robot must be in modular mode")
                response.success = False
                return response

            self.__timer_state_machine.cancel()

            self.get_logger().info('Setting pause mode before zeros')
            self.switch_control_mode(0) 
            time.sleep(0.25)


            self.get_logger().info('Calling set_zeros service...')
            if not self.set_zeros_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
                raise RuntimeError("Service /nb/set_zeros not available")

            future = self.set_zeros_client.call_async(Empty_srv.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)

            if future.exception() is not None:
                raise RuntimeError(f'Failed service call: {future.exception()}')

            self.get_logger().info("Set zeros successful")
            time.sleep(0.25)
            self.get_logger().info('Setting cartesian mode ...')
            self.switch_control_mode(1)

            self.__timer_state_machine.reset()

            self.current_state = RobotState.IDLE

            return response
        
        except Exception as e:
            self.current_error = str(e)
            self.get_logger().error(f'Error during calibration: {e}')
            return response
        

    def on_clear_tcp_trace(self, _):
        try:
            self.tf_path_trail_nb_base_link_to_nb_tcp_client_clear_path.call_async(Empty_srv.Request())
            self.tf_path_trail_nb_mimic_base_link_to_nb_mimic_tcp_wrist_client_clear_path.call_async(Empty_srv.Request())
            self.get_logger().info('TCP trace cleared')
        except Exception as e:
            self.get_logger().error(f'Error clearing TCP trace: {e}')


    def on_desired_trajectory(self, msg: JointTrajectory) -> None:
        """Update the current pose of the robot."""
        self.__q_desired = list(msg.points[0].positions)


    def on_pos_gain_changed(self, msg):
        new_gain = msg.data
        self.set_parameters([RclpyParameter('pos_gain', RclpyParameter.Type.DOUBLE, new_gain)])
        self.get_logger().info(f"Position gain updated to: {new_gain:.2f}")


    def on_quat_gain_changed(self, msg):
        new_gain = msg.data
        self.set_parameters([RclpyParameter('quat_gain', RclpyParameter.Type.DOUBLE, new_gain)])
        self.get_logger().info(f"Quaternion gain updated to: {new_gain:.2f}")
    
    
    def on_modular_gain_changed(self, msg):
        new_gain = msg.data
        self.set_parameters([RclpyParameter('modular_gain', RclpyParameter.Type.DOUBLE, new_gain)])
        self.get_logger().info(f"Modular gain updated to: {new_gain:.2f}")

    
    def on_modular_command_toggle(self, msg):
        try:
            self.modular_control_enabled = msg.data
           
            if self.modular_control_enabled:
                self.previous_state = self.current_state
                self.current_state = RobotState.MODULAR_CONTROL
                
                self.switch_control_mode(2) #ControlMode.TELEOP_MODULE
                self.get_logger().info("Switched to modular command mode")
            else:
                if self.current_state == RobotState.MODULAR_CONTROL:
                    self.switch_control_mode(1) #ControlMode.TELEOP_XYZ
                    self.current_state = RobotState.IDLE

                self.get_logger().info("Switched to cartesian control mode")
        except Exception as e:
            self.get_logger().error(f"Error toggling modular mode : {e}")

    
    def on_emergency_stop(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().info('EMERGENCY STOP')
            self.switch_control_mode(0)
            self.current_state = RobotState.PAUSE
        else: 
            if self.current_state == RobotState.PAUSE:
                self.current_state = RobotState.IDLE

   
def main(args=None):
    rclpy.init(args=args)
    telesoud_command_node = TelesoudCommandToCartesianNode("welding_cartesian_command_node")
    try:
        rclpy.spin(telesoud_command_node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('rclpy').info('signal_handler(signum=2)')
    except ShutdownException:
        telesoud_command_node.get_logger().info('ShutdownException...')
    except Exception as e:
        telesoud_command_node.current_error = str(e)
        telesoud_command_node.get_logger().error(f'Exception : {e}')
    finally:
        telesoud_command_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
