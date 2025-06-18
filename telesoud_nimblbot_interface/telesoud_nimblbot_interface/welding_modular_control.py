import time
import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import ShutdownException
from rclpy.parameter import Parameter as RclpyParameter
from std_srvs.srv import Empty as Empty_srv
from std_msgs.msg import Float64, Bool, String, Int8
from geometry_msgs.msg import Twist 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nimblpy.common.robot_loader import load_robot_config

TIMER_RATE = 40.0
ROBOT_RATE = 20.0
SERVICE_TIMEOUT = 5.0
MIN_MODULAR_VELOCITY = 0.05
MAX_MODULAR_VELOCITY = 0.5
MODULAR_VELOCITY_STEP = 0.05
DEFAULT_MODULAR_SPEED = 0.3 
AZIMUTH = 0
TILT = 1
MODULE_SELECT_COMMAND = 0
SECTION_SELECT_COMMAND = 1
ALL_SELECT_COMMAND = 2 

class ControlMode:
    PAUSE: int=0
    TELEOP_XYZ: int=1
    TELEOP_MODULE : int=2

class WeldingModularControlNode(Node):
    def __init__(self, node_name: str) -> None:
        """Initialize the WeldingCommandHandlerNode.
        
        Args:
            node_name: Name of the ROS2 node
        """
        super().__init__(node_name)
        self._setup_robot_configuration()
        self._initialize_reusable_ros_msg_objects()
        self._initialize_basic_parameters()
        self._create_publishers()
        self._create_subscriptions()
        self._create_clients()
        self._create_services()
    

    def _setup_robot_configuration(self) -> None:
        """Setup robot configuration for modular control."""
        self.declare_parameter("robot_type", "")
        robot_type = self.get_parameter("robot_type").get_parameter_value().string_value

        try:
            conf = load_robot_config(robot_type)
            self.__terminal_wrist = conf["robot_configuration"]["terminal_wrist"]
            self.__num_modules = conf["robot_configuration"]["num_modules"]
            self.__num_sections = self.__num_modules // 3
            self.__num_joints = 2 * self.__num_modules + self.__terminal_wrist
            
            alias = conf['low_level_control']['alias']
            self.__joint_names_alias = [alias + str(i) if i != 0 else alias for i in range(self.__num_joints)]

        except (KeyError, TypeError, FileNotFoundError) as e:
            self.get_logger().fatal(f'Failed to load robot configuration "{robot_type}" : {e}')
            rclpy.shutdown()

    
    def _initialize_basic_parameters(self) -> None:
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
        self.declare_parameter('modular_gain', 1.0)
        self.__q_desired = None
        self.current_speed_vector = None
        self.modular_timer = self.create_timer(1/TIMER_RATE, self._process_modular_control) 

    
    def _initialize_reusable_ros_msg_objects(self) -> None:
        """Initialize reusable ROS message objects to avoid repeated instantiation."""
        self._modular_mode_msg = String()
        self._modular_velocity_indicator_msg = Float64()
        self._status_msg = Bool()
        self._switch_mode_msg = Int8()

    
    def _create_publishers(self) -> None:
        
        self.joints_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/nb/desired_trajectory_modular',
            10
        )

        self.modular_mode_pub = self.create_publisher(
            String, 
            '/welding_modular_control/modular_mode',
            10
        )

        self.modular_velocity_indicator_pub = self.create_publisher(
            Float64,
            '/welding_modular_control/modular_velocity_indicator',
            10
        )

        self.modular_status_pub = self.create_publisher(
            Bool, 
            '/welding_modular_control/status', 
            10
        )

        self.modular_command_request_pub = self.create_publisher(
            Int8,
            '/welding_modular_control/control_mode_request',
            10
        )
    

    def _create_subscriptions(self) -> None:
        
        _ = self.create_subscription(
                Float64,
            'rviz_plugin/modular_gain',
            self.on_modular_gain_changed,
            10
        )

        _ = self.create_subscription(
            Bool, 
            '/rviz_plugin/modular_command_state',
            self.on_modular_command_toggle,
            10
        )

        _ = self.speed_vector_sub = self.create_subscription(
            Twist, 
            '/welding_command_handler/speed_vector',
            self.on_speed_vector,
            10
        )

        _ = self.joint_trajectory_sub = self.create_subscription(
                JointTrajectory,
                '/nb/desired_trajectory',
                self.on_desired_trajectory,
                10
        )
    

    def _create_clients(self) -> None:

        # Luos wrapper node
        self.set_zeros_client = self.create_client(
            Empty_srv,
            "/nb/set_zeros"
        )
     
    
    def _create_services(self) -> None:
        """Create ROS2 services provided by this node.
        
        Sets up services for:
        - Robot calibration (set zeros) requests
        """
        # Set zeros protocol
        self.set_zeros_srv = self.create_service(
            Empty_srv, 
            'welding_cartesian_command/set_zeros_request',
            self.on_set_zeros
        )


    def _detect_modular_command_toggle(self, speed_vector: Twist) -> tuple:
        """Detect toggle events in modular command inputs from speed vector.
        
        Processes speed vector components into discrete modular commands
        and detects rising/falling edge transitions.
        
        Args:
            speed_vector: Input velocity commands in 6DOF
            
        Returns:
            Tuple of (toggles dict, current_commands dict)
        """
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

    
    def _process_modular_control(self) -> None:
        """Process modular control commands for continuum robot manipulation.
        
        Handles direct joint-level control of robot modules including
        azimuth/tilt movements and wrist control.
        """
        if not self.modular_control_enabled:
            return

        if self.__q_desired is None:
            self.get_logger().debug('Waiting /nb/desired_trajectory to be populated')
            return
        
        if self.current_speed_vector is None:
            return

        self._handle_modular_commands()
        self._execute_modular_movement()

    
    def _handle_modular_commands(self) -> None:
        """Handle modular control command processing and state updates.
        
        Processes toggle events to update:
        - Module/section selection
        - Velocity settings
        - Command mode (azimuth/tilt)
        - Selection mode (module/section)
        """
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
            self._modular_mode_msg.data = 'TILT' if self.__modular_command_mode[0] == TILT else 'AZIMUTH'
            self.modular_mode_pub.publish(self._modular_mode_msg)
            self.get_logger().info(f"Switched to {'TILT' if self.__modular_command_mode[0] == TILT else 'AZIMUTH'} mode")

        if toggles['selection_mode'] == 'rise':
            self.__modular_selector_mode = self.__modular_selector_mode[1:] + [self.__modular_selector_mode[0]]
            self.get_logger().info(f"Selection mode changed to {'MODULE' if self.__modular_selector_mode[0] == MODULE_SELECT_COMMAND  else 'SECTION'}")

    
    def _execute_modular_movement(self) -> None:
        """Execute modular movement commands by publishing joint trajectories.
        
        Generates and publishes joint-level commands for selected modules
        based on current modular control state and velocity settings.
        """
        turn_module = self.current_modular_states['turn_module']
        wrist_command = self.current_modular_states['wrist_command']

        modular_gain = self.get_parameter('modular_gain').value
        self.amplified_modular_velocity = modular_gain * self.modular_velocity

        self.joints_trajectory = JointTrajectory()
        self.joints_trajectory.header.stamp = self.get_clock().now().to_msg()
        joint_names = []
        positions = []
        velocities = []
        
        try:
            if self.__modular_selector_mode[0] == MODULE_SELECT_COMMAND:
                joint_inf_idx = int(self.__modular_selection_module) * 2
                joint_sup_idx = joint_inf_idx + 2
            else:
                joint_inf_idx = 6 * int(self.__modular_selection_section)
                joint_sup_idx = joint_inf_idx + 6
            
            if joint_sup_idx > len(self.__q_desired) or joint_inf_idx < 0:
                self.get_logger().error(f'Invalid joint indices: [{joint_inf_idx}:{joint_sup_idx}] for {len(self.__q_desired)} joints')
                return
                
            module_joint_names = self.__joint_names_alias[joint_inf_idx:joint_sup_idx]
        
        except (IndexError, ValueError, TypeError) as e:
            self.get_logger().error(f'Error calculating modular joint indces: {e}')
            return 

        module_command = turn_module * self.amplified_modular_velocity
        module_velocities = [module_command] * len(module_joint_names)

        if self.__modular_command_mode[0] == TILT:
            # alternate velocity direction for TILT mode (opposing pairs)
            module_velocities = [v if i % 2 == 0 else -v for i, v in enumerate(module_velocities)]
        
        try:
            if turn_module != 0:
                module_positions = [self.__q_desired[joint_inf_idx + i] + v / ROBOT_RATE for i, v in enumerate(module_velocities)] 
            else : 
                module_positions = [self.__q_desired[joint_inf_idx + i] for i in range(len(module_velocities))]
        except (IndexError, TypeError) as e:
            self.get_logger().error(f'Error accessing joint positions: {e}')
            return

        joint_names.extend(module_joint_names)
        positions.extend(module_positions)
        velocities.extend(module_velocities)

        if self.__terminal_wrist:
            try:
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
            except (IndexError, TypeError) as e:
                self.get_logger().error(f'Error handling wrist joint: {e}')

        if len(joint_names) > 0:
            self.joints_trajectory.joint_names = joint_names
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions
            trajectory_point.velocities = velocities
            self.joints_trajectory.points = [trajectory_point]

            self.joints_trajectory_pub.publish(self.joints_trajectory)
        
        # For visualization purposes
        self._modular_velocity_indicator_msg.data = self.amplified_modular_velocity
        self.modular_velocity_indicator_pub.publish(self._modular_velocity_indicator_msg)

    
    def on_modular_gain_changed(self, msg: Float64) -> None:
        """Handle modular gain parameter update from RViz plugin.
        
        Args:
            msg: New modular gain value
        """
        new_gain = msg.data
        self.set_parameters([RclpyParameter('modular_gain', RclpyParameter.Type.DOUBLE, new_gain)])
        self.get_logger().info(f"Modular gain updated to: {new_gain:.2f}")

    
    def on_modular_command_toggle(self, msg: Bool) -> None:
        """Handle modular command mode toggle from RViz plugin.
        
        Switches between cartesian and modular control modes.
        
        Args:
            msg: Boolean indicating modular mode state
        """
        try:
            self.modular_control_enabled = msg.data
            
            self._status_msg.data = self.modular_control_enabled
            self.modular_status_pub.publish(self._status_msg)
           
            if self.modular_control_enabled:
                self._switch_mode_msg.data = ControlMode.TELEOP_MODULE 
                self.modular_command_request_pub.publish(self._switch_mode_msg)
                self.get_logger().info("Switched to modular control mode")
            else:
                self._switch_mode_msg.data = ControlMode.TELEOP_XYZ
                self.modular_command_request_pub.publish(self._switch_mode_msg)
                self.get_logger().info("Switched to cartesian control mode")

        except Exception as e:
            self.get_logger().error(f"Error toggling modular mode : {e}")

   
    def on_set_zeros(self, request: Empty_srv.Request, response: Empty_srv.Response) -> Empty_srv.Response:
        """Handle set zeros calibration service request.
        
        Calibrates robot joint positions to zero configuration.
        Must be called in modular control mode.
        
        Args:
            request: Empty service request
            response: Service response to populate
            
        Returns:
            Service response with success status
        """
        try:
            if not self.modular_control_enabled:
                self.get_logger().error("Cannot set zeros: robot must be in modular mode")
                response.success = False
                return response

            self._switch_mode_msg.data = ControlMode.PAUSE
            self.modular_command_request_pub.publish(self._switch_mode_msg)

            self.get_logger().info('Setting pause mode before zeros')
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

            self._switch_mode_msg.data = ControlMode.TELEOP_XYZ
            self.modular_command_request_pub.publish(self._switch_mode_msg)
            self.get_logger().info('Setting cartesian mode ...')

            return response
        
        except Exception as e:
            self.get_logger().error(f'Error during calibration: {e}')
            return response
        

    def on_speed_vector(self, msg: Twist) -> None:
        """Handle speed vector from welding command handler."""
        self.current_speed_vector = msg

    
    def on_desired_trajectory(self, msg: JointTrajectory) -> None:
        """Update current joint positions from trajectory feedback."""
        if msg.points:
            self.__q_desired = list(msg.points[0].positions)


def main(args=None):
    rclpy.init(args=args)
    welding_modular_control_node = WeldingModularControlNode("welding_modular_control_node")
    try:
        rclpy.spin(welding_modular_control_node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('rclpy').info('signal_handler(signum=2)')
    except ShutdownException:
        welding_modular_control_node.get_logger().info('ShutdownException...')
    except Exception as e:
        welding_modular_control_node.current_error = str(e)
        welding_modular_control_node.get_logger().error(f'Exception : {e}')
    finally:
        welding_modular_control_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
