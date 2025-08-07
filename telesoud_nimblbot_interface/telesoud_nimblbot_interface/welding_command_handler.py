import time
from math import pi
import numpy as np
from rclpy.utilities import try_shutdown
from scipy.spatial.transform import Slerp
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import ShutdownException
from rclpy.parameter import Parameter as RclpyParameter
from rcl_interfaces.msg import ParameterValue, Parameter, ParameterType
from std_srvs.srv import Empty as Empty_srv
from std_msgs.msg import Empty, Header, Float64, Bool, String, Int8MultiArray, Int8
from moveit_msgs.srv import ServoCommandType
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, Twist
from tf_transformations import quaternion_multiply
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.srv import GetParameters, SetParameters
from interface_custom_msgs.msg import RobotData, Command, CommandStatus
from typing import List, Optional, Any
from collections import deque

SERVO_NODE = "/servo_node"

ROBOT_RATE = 20.0  # NimblBot input command rate
TELESOUD_RATE = 40.0  # Telesoud output command rate
TIMER_PERIOD = 0.025  # 40 Hz = Telesoud ouput command rate

STOP_COMMAND_THRESHOLD = 2400  # ~60 sec at 40 Hz -- delay before automatic safety pause
POSITION_EPSILON = 0.001  # 1mm

SPEED_CORRECTION_FACTOR = 1.0  # Correction factor for TCP_speed
DEFAULT_TCP_SPEED = 0.005

ZERO_THRESHOLD = 1e-8

SERVICE_TIMEOUT = 5


class ControlMode:
    PAUSE: int = 0
    TELEOP_XYZ: int = 1
    TELEOP_MODULE: int = 2


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
    RobotState.PAUSE: "PAUSE",
}


class WeldingCommandHandlerNode(Node):
    def __init__(self, node_name: str) -> None:
        """Initialize the WeldingCommandHandlerNode.

        Args:
            node_name: Name of the ROS2 node
        """
        super().__init__(node_name)
        self._initialize_reusable_ros_msg_objects()
        self._initialize_basic_parameters()
        self._create_publishers()
        self._create_subscriptions()
        self._create_clients()
        self.create_timer(TIMER_PERIOD, self._update_state_machine)
        self.create_timer(TIMER_PERIOD, self.process_pending_command)

    def _initialize_reusable_ros_msg_objects(self) -> None:
        """Initialize reusable ROS message objects to avoid repeated instantiation."""
        # Namespace from ID's
        self.namespace = "nb"
        self.__base_frame_robot = f"{self.namespace}/base_link"
        self._command_status_msg = CommandStatus()
        self._robot_data_msg = RobotData()
        self._robot_state_msg = String()
        self._modular_mode_msg = String()
        self._modular_velocity_indicator_msg = Float64()
        self._motor_lock_msg = Int8MultiArray()
        self._twist_msg = Twist()

        self._pose_stamped_msg = PoseStamped()
        self._pose_stamped_msg.header.frame_id = self.__base_frame_robot
        self._pose_stamped_msg.pose = Pose()

        self._target_pose_msg = PoseStamped()
        self._target_pose_msg.header.frame_id = self.__base_frame_robot
        self._target_pose_msg.pose = Pose()

        self._current_pose_msg = PoseStamped()
        self._current_pose_msg.header.frame_id = self.__base_frame_robot
        self._current_pose_msg.pose = Pose()

    def _initialize_basic_parameters(self) -> None:
        """Initialize basic node parameters and state variables.

        Sets up command handling state,
        robot state machine variables, and movement parameters.
        """
        # Command handling
        self.stop_command_counter = 0
        self.last_command_type = None
        self.pending_command = None
        self.command_data = None
        self.critical_commands = deque()
        # State machine variables
        self.current_state = RobotState.PAUSE
        self.previous_state = RobotState.PAUSE
        self.current_control_mode = ControlMode.PAUSE
        # Error handling
        self.current_error = ""
        self.robot_in_fault_status = False
        # Telesoud interpolated trajectory execution
        self.virtual_pose = None
        self.virtual_pose_initialized = False
        # Flags
        self.emergency_stop = False
        self.resuming_flag = False
        self.modular_control_enabled = False
        # Movement  parameters
        self.tcp_pose = None
        self.tcp_speed = DEFAULT_TCP_SPEED

        default_pose = Pose()
        default_pose.orientation.w = 1.0
        self.current_target_pose = default_pose

        self.declare_parameter("pos_gain", 1.0)
        self.declare_parameter("quat_gain", 1.0)
        self.declare_parameter("welding_scene", "standard")
        self.welding_scene = (
            self.get_parameter("welding_scene").get_parameter_value().string_value
        )
        if self.welding_scene != "standard":
            # self._motor_lock_msg.data=list(range(6))
            pass
        self.current_speed_vector = self._twist_msg
        # Cartesian movement interpolation
        self.interpolated_line_poses = None
        self.line_progression_index = 0
        # timestamp
        self.command_start_timestamp = None
        # Get ee_frame name config in servo_node
        self.__ee_frame_mimic: str = self._get_param(
            SERVO_NODE + "/get_parameters", "moveit_servo_nb.ee_frame"
        ).string_value
        self.__ee_frame_robot: str = self.__ee_frame_mimic.replace("_mimic", "")
        # TF
        self.__tf_buffer = Buffer()
        TransformListener(self.__tf_buffer, self)

    def _create_publishers(self) -> None:
        """Create ROS2 publishers for robot control and status communication.

        Sets up publishers for:
        - Motor lock commands
        - Desired pose commands
        - Command status feedback
        - Robot state visualization
        - TCP pose feedback
        - Speed vectors
        """
        # Multiple command publishers
        self.motor_lock_pub = self.create_publisher(
            Int8MultiArray, "/nb/motor_lock", 10
        )
        self.motor_lock_pub.publish(self._motor_lock_msg)

        self.desired_pose_pub = self.create_publisher(
            PoseStamped, "/nb/desired_pose", 10
        )

        # Telesoud interface publishers
        self.status_pub = self.create_publisher(
            CommandStatus, "/welding_command_handler/command_status", 10
        )

        # For RVIZ plugin panel
        self.robotState_pub = self.create_publisher(
            String, "/welding_command_handler/robotState", 10
        )

        self.tcp_pose_pub = self.create_publisher(
            Pose, "welding_command_handler/tcp_pose", 10
        )

        # Welding modular control
        self.modular_speed_vector_pub = self.create_publisher(
            Twist, "/welding_command_handler/speed_vector", 10
        )

    def _create_subscriptions(self) -> None:
        """Create ROS2 subscriptions for incoming control signals and parameters.

        Sets up subscriptions for:
        - RViz plugin parameter updates
        - Emergency stop signals
        - TCP trace clearing commands
        - Robot commands from translator
        - Control mode switch request from welding modular control
        """
        # Rviz plugin
        _ = self.create_subscription(
            Float64, "/rviz_plugin/pos_gain", self.on_pos_gain_changed, 10
        )

        _ = self.create_subscription(
            Float64, "/rviz_plugin/quat_gain", self.on_quat_gain_changed, 10
        )

        _ = self.create_subscription(
            Bool, "/rviz_plugin/emergency_stop", self.on_emergency_stop, 10
        )

        # For TCP trace clearing
        _ = self.create_subscription(
            Empty, "/rviz_plugin/clear_tcp_trace", self.on_clear_tcp_trace, 10
        )

        # Telesoud interface subscribers
        _ = self.create_subscription(
            Command, "/translator/command", self.on_command, 10
        )

        # Welding modular control
        _ = self.create_subscription(
            Bool, "/welding_modular_control/status", self.on_modular_status, 10
        )

        _ = self.create_subscription(
            Int8,
            "/welding_modular_control/control_mode_request",
            self.on_modular_control_mode_request,
            10,
        )

    def _create_clients(self) -> None:
        """Create ROS2 service clients for robot control and configuration.

        Sets up clients for:
        - Control mode switching
        - TCP trace path clearing
        """
        # Multiple command client
        self.change_control_mode_client = self.create_client(
            ServoCommandType, "/nb/change_control_mode"
        )

        # Servo node clients
        self.tf_path_trail_nb_base_link_to_nb_tcp_client_clear_path = (
            self.create_client(
                Empty_srv,
                "/tf_path_trail_"
                + self.__base_frame_robot.replace("/", "_")
                + "_to_"
                + self.__ee_frame_robot.replace("/", "_")
                + "/clear_path",
            )
        )

        self.tf_path_trail_nb_mimic_base_link_to_nb_mimic_tcp_wrist_client_clear_path = self.create_client(
            Empty_srv,
            "/tf_path_trail_"
            + "nb_mimic_base_link"
            + "_to_"
            + "nb_mimic_tcp_wrist"
            + "/clear_path",
        )

    def _get_param(self, node_name: str, parameters: str) -> Optional[ParameterValue]:
        """Retrieve parameter value from a specified ROS2 node.

        Args:
            node_name: Name of the target ROS2 node
            parameters: Name of the parameter to retrieve

        Returns:
            Parameter value, or empty ParameterValue if failed
        """
        client = self.create_client(GetParameters, node_name)

        while not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info(f"Waiting for {node_name} to get parameter...")

        future = client.call_async(GetParameters.Request(names=[parameters]))
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None:
            try:
                response: GetParameters.Response = result
                if response.values and len(response.values) > 0:
                    values_list = list(response.values)
                    return values_list[0]
                else:
                    self.get_logger().error("Parameter not found")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
            return ParameterValue()

    def on_command(self, msg: Command) -> None:
        """Handle incoming robot commands from translator.

        Args:
            msg: Command message with type, pose, speed, and other parameters
        """
        command_data = {
            "command_type": msg.command_type,
            "command_id": msg.command_id,
            "target_pose": msg.target_pose,
            "speed": msg.speed,
            "speed_vector": msg.speed_vector,
        }
        if msg.command_type >= 8:
            self.critical_commands.append(command_data)
        else:
            self.command_data = command_data
        self.pending_command = True

    def process_pending_command(self) -> None:
        """Process pending robot command and execute appropriate action.

        Routes commands to specific handlers based on command type and
        publishes command status feedback.
        """
        if self.critical_commands:
            data = self.critical_commands.popleft()
        elif self.command_data:
            data = self.command_data
            self.command_data = None
        else:
            self.pending_command = False
            return

        command_type = data["command_type"]
        command_id = data["command_id"]

        self.command_start_timestamp = self.get_clock().now().to_msg()

        if command_type != 0:
            self.stop_command_counter = 0
            self.last_command_type = command_type

        self.pending_command = False

        self._command_status_msg.command_id = command_id
        self._command_status_msg.command_type = command_type

        try:
            self._command_status_msg.robot_data = self._create_robot_data()
        except Exception as e:
            self.get_logger().warn(f"Could not create robot data : {e}")

        if self.modular_control_enabled:
            if command_type == 0:
                self.current_speed_vector = self._twist_msg
            if command_type == 7:
                self._process_set_dynamic(
                    self._command_status_msg, data["speed_vector"]
                )
            self._command_status_msg.success = True
            self._command_status_msg.message = "Modular command"

        else:
            try:
                match command_type:
                    case 0:
                        self._process_stop_command(self._command_status_msg)
                    case 1:
                        self._process_get_robot_data(self._command_status_msg)
                    case 7:
                        self._process_set_dynamic(
                            self._command_status_msg, data["speed_vector"]
                        )
                    case 8:
                        self._process_start_dynamic(self._command_status_msg)
                    case 15:
                        self._process_play_cartesian(
                            self._command_status_msg, data["target_pose"], data["speed"]
                        )
                    case 16:
                        self._process_play_joint(
                            self._command_status_msg, data["target_pose"], data["speed"]
                        )
                    case _:
                        self._command_status_msg.success = False
                        self._command_status_msg.message = (
                            f"Unknown command type: {command_type}"
                        )
            except Exception as e:
                self._command_status_msg.success = False
                self._command_status_msg.message = f"Error processing command: {str(e)}"
                self.get_logger().error(f"Error in command processing: {e}")

        self.status_pub.publish(self._command_status_msg)

    def _process_stop_command(self, status: CommandStatus) -> None:
        """Process STOP command and handle safety timeout.

        Args:
            status: CommandStatus object to update with execution result
        """
        self.current_speed_vector = self._twist_msg
        if self.last_command_type == 0:
            self.stop_command_counter += 1
        else:
            self.stop_command_counter = 0
            self.last_command_type = 0

        if (
            self.stop_command_counter >= STOP_COMMAND_THRESHOLD
            and not self.current_control_mode == ControlMode.PAUSE
            and not self.emergency_stop
        ):
            self.switch_control_mode(ControlMode.PAUSE)
            self.current_state = RobotState.PAUSE
            self.stop_command_counter = 0

        status.success = True
        status.message = "Stop command processed"

    def _process_get_robot_data(self, status: CommandStatus) -> None:
        """Process GET_ROBOT_DATA command and retrieve current robot state.

        Args:
            status: CommandStatus object to populate with robot data
        """
        try:
            robot_data = self._create_robot_data()
            status.robot_data = robot_data
            status.success = True
            status.message = "Robot data retrieved"
        except Exception as e:
            status.success = False
            status.message = f"Error getting robot data: {str(e)}"

    def _process_set_dynamic(self, status: CommandStatus, speed_vector: Twist) -> None:
        """Process SET_DYNAMIC command to update cartesian speed vector.

        Args:
            status: CommandStatus object to update
            speed_vector: Target velocity in cartesian coordinates
        """
        self.current_speed_vector = speed_vector
        status.success = True
        status.message = "Dynamic speed updated"

    def _process_start_dynamic(self, status: CommandStatus) -> None:
        """Process START_DYNAMIC command to begin dynamic cartesian movement.

        Args:
            status: CommandStatus object to update with execution result
        """
        self.switch_control_mode(ControlMode.TELEOP_XYZ)
        self.current_state = RobotState.DYNAMIC_MOVEMENT
        status.success = True
        status.message = "Dynamic cartesian movement started"

    def _process_play_cartesian(
        self, status: CommandStatus, target_pose: Pose, speed: float
    ) -> None:
        """Process PLAY_CARTESIAN command for cartesian trajectory execution.

        Args:
            status: CommandStatus object to update
            target_pose: Target end-effector pose
            speed: Movement speed for trajectory
        """
        self.tcp_speed = speed * SPEED_CORRECTION_FACTOR
        self.current_target_pose = target_pose

        self.switch_control_mode(ControlMode.TELEOP_XYZ)

        self.interpolated_line_poses = None
        self.line_progression_index = 0
        self.current_state = RobotState.CARTESIAN_TRAJECTORY

        status.success = True
        status.message = "Cartesian trajectory started"

    def _process_play_joint(
        self, status: CommandStatus, target_pose: Pose, speed: float
    ) -> None:
        """Process PLAY_JOINT command for joint-space trajectory execution.

        Args:
            status: CommandStatus object to update
            target_pose: Target end-effector pose
            speed: Movement speed for trajectory
        """
        self.tcp_speed = speed * SPEED_CORRECTION_FACTOR
        self.current_target_pose = target_pose

        self.switch_control_mode(ControlMode.TELEOP_XYZ)

        self._pose_stamped_msg.pose = self.current_target_pose
        self._pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        self.desired_pose_pub.publish(self._pose_stamped_msg)

        self.current_state = RobotState.JOINT_TRAJECTORY

        status.success = True
        status.message = "Joint trajectory started"

    def _process_modular_control(self) -> None:
        """Forward speed vector to welding modular control node"""
        if self.current_speed_vector is not None:
            self.modular_speed_vector_pub.publish(self.current_speed_vector)

    def _update_state_machine(self) -> None:
        """Update robot state machine and handle state transitions.

        Manages transitions between IDLE, PAUSE, DYNAMIC_MOVEMENT,
        CARTESIAN_TRAJECTORY, JOINT_TRAJECTORY, and MODULAR_CONTROL states.
        """
        initial_state = self.current_state

        if self.current_state == RobotState.IDLE:
            pass

        elif self.current_state == RobotState.PAUSE:
            if self.resuming_flag:
                self.current_state = RobotState.IDLE
                self.get_logger().info("Pause finished, returning to IDLE state")
                self.resuming_flag = False

        elif self.current_state == RobotState.DYNAMIC_MOVEMENT:
            if self._is_dynamic_movement_complete():
                self.get_logger().info(
                    "Dynamic movement finished, returning to IDLE state"
                )
                self.current_state = RobotState.IDLE

        elif self.current_state == RobotState.CARTESIAN_TRAJECTORY:
            if self._is_cartesian_trajectory_complete():
                self.get_logger().info(
                    "Cartesian trajectory finished, returning to IDLE state"
                )
                self.current_state = RobotState.IDLE

        elif self.current_state == RobotState.JOINT_TRAJECTORY:
            if self._is_joint_trajectory_complete():
                self.get_logger().info(
                    "Joint trajectory finished, returning to IDLE state"
                )
                self.current_state = RobotState.IDLE

        elif self.current_state == RobotState.MODULAR_CONTROL:
            self._process_modular_control()
            if not self.modular_control_enabled:
                self.get_logger().info(
                    "Modular control disabled, returning to IDLE state"
                )
                self.current_state = RobotState.IDLE

        if self.current_state != initial_state:
            self.previous_state = initial_state
            self.get_logger().info(f"{STATE_NAMES[self.current_state]}")

        self._robot_state_msg.data = STATE_NAMES[self.current_state]
        if self.tcp_pose:
            self.tcp_pose_pub.publish(self.tcp_pose)
        self.robotState_pub.publish(self._robot_state_msg)

    def _is_cartesian_trajectory_complete(self) -> bool:
        """Execute and check if cartesian trajectory execution is complete.

        Returns:
            True if trajectory is finished or should be stopped, False otherwise
        """
        if self.current_state == RobotState.CARTESIAN_TRAJECTORY:
            current_pose = self._get_current_pose(mimic=False)
            if self.stop_command_counter > 0:
                return True
            if current_pose:
                completed = self.execute_line(current_pose.pose)
                if completed:
                    return True
        return False

    def _is_joint_trajectory_complete(self) -> bool:
        """Execute and check if joint trajectory execution is complete.

        Returns:
            True if trajectory is finished or should be stopped, False otherwise
        """
        if self.stop_command_counter > 0:
            return True
        if self.current_state == RobotState.JOINT_TRAJECTORY:
            try:
                current_pose = self._get_current_pose(mimic=False)
                if current_pose:
                    error = self._get_position_error(
                        current_pose.pose, self.current_target_pose
                    )
                    if error <= POSITION_EPSILON:
                        request = SetParameters.Request()
                        param_value = ParameterValue()
                        param_value.type = ParameterType.PARAMETER_DOUBLE
                        param_value.double_value = 0.025

                        linear_param = Parameter()
                        linear_param.name = "servo.scale.linear"
                        linear_param.value = param_value

                        request.parameters = [linear_param]

                        self.current_state = RobotState.IDLE
                        return True

                self._pose_stamped_msg.pose = self.current_target_pose
                self._pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
                self.desired_pose_pub.publish(self._pose_stamped_msg)
            except Exception as e:
                self.get_logger().error(f"Error during joint trajectory execution {e}")
        return False

    def _is_dynamic_movement_complete(self) -> bool:
        """Execute and check if dynamic cartesian movement should continue or stop.

        Returns:
            True if movement should stop, False if it should continue
        """
        if self.current_state != RobotState.DYNAMIC_MOVEMENT:
            return False
        if self.current_speed_vector != self._twist_msg:
            target_pose_msg = self.compute_target_pose()
            if target_pose_msg:
                self.desired_pose_pub.publish(target_pose_msg)
        if self.stop_command_counter > 0:
            self.get_logger().info("OUI")
            self.current_state = RobotState.IDLE
            self.virtual_pose_initialized = False
            self.virtual_pose = None
            return True
        return False

    def _get_current_pose(
        self, mimic: bool, timeout_sec: int = SERVICE_TIMEOUT
    ) -> PoseStamped:
        """Get current end-effector pose in base frame.

        Args:
            mimic: If True, use mimic frame; if False, use actual robot frame
            timeout_sec: Timeout for trarnsform lookup

        Returns:
            Current pose of end-effector

        Raises:
            LookupError: If transform looekup fails
        """
        ee_frame = self.__ee_frame_mimic if mimic else self.__ee_frame_robot

        try:
            transform_result = self.__tf_buffer.lookup_transform(
                self.__base_frame_robot, ee_frame, Time(), Duration(seconds=timeout_sec)
            )
        except Exception as e:
            raise LookupError(f"Waiting transform of {ee_frame}, {e}")

        current_pose = self._current_pose_msg
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.pose.position.x = transform_result.transform.translation.x
        current_pose.pose.position.y = transform_result.transform.translation.y
        current_pose.pose.position.z = transform_result.transform.translation.z
        current_pose.pose.orientation = transform_result.transform.rotation

        return current_pose

    def _get_position_error(self, current_pose: Pose, target_pose: Pose) -> np.array:
        """Calculate euclidean distance between current and target positions.

        Args:
            current_pose: Current end-effector pose
            target_pose: Target end-effector pose

        Returns:
            Distance in meters between positions
        """
        error_vector = [
            target_pose.position.x - current_pose.position.x,
            target_pose.position.y - current_pose.position.y,
            target_pose.position.z - current_pose.position.z,
        ]
        return np.linalg.norm(error_vector)

    def compute_target_pose(self) -> Optional["PoseStamped"]:
        """Compute target pose for dynamic cartesian movement.

        Integrates current speed vector to generate incremental pose updates
        for real-time cartesian control.

        Speed vectors orientation fields are interpreted following
        axe-angle rotation coding convention.

        Returns:
            Target pose message, or None if no movement commanded
        """
        if self.current_speed_vector == self._twist_msg:
            return None

        pos_gain = self.get_parameter("pos_gain").value
        quat_gain = self.get_parameter("quat_gain").value

        if pos_gain is None or quat_gain is None:
            self.get_logger().error("pos_gain or quat_gain parameter not set")
            return None

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
                    z=current_pose.pose.position.z,
                ),
                orientation=current_pose.pose.orientation,
            )
        self.virtual_pose_initialized = True

        working_pose = self.virtual_pose
        dt = 1.0 / TELESOUD_RATE

        dx = self.current_speed_vector.linear.x * dt * pos_gain
        dy = self.current_speed_vector.linear.y * dt * pos_gain
        dz = self.current_speed_vector.linear.z * dt * pos_gain

        self.virtual_pose.pose.position.x += dx
        self.virtual_pose.pose.position.y += dy
        self.virtual_pose.pose.position.z += dz

        rx = self.current_speed_vector.angular.x
        ry = self.current_speed_vector.angular.y
        rz = self.current_speed_vector.angular.z

        omega_norm = np.sqrt(rx**2 + ry**2 + rz**2)

        if omega_norm > ZERO_THRESHOLD:
            rotation_vector = np.array([rx, ry, rz]) * dt * quat_gain

            try:
                rotation = R.from_rotvec(rotation_vector)
                dquat_scipy = rotation.as_quat()  # Format [x,y,z,w]

                current_quat_xyzw = [
                    working_pose.pose.orientation.x,
                    working_pose.pose.orientation.y,
                    working_pose.pose.orientation.z,
                    working_pose.pose.orientation.w,
                ]

                new_quat_xyzw = quaternion_multiply(current_quat_xyzw, dquat_scipy)

                self.virtual_pose.pose.orientation = Quaternion(
                    x=new_quat_xyzw[0],
                    y=new_quat_xyzw[1],
                    z=new_quat_xyzw[2],
                    w=new_quat_xyzw[3],
                )

            except Exception as e:
                self.get_logger().error(f"Error in rotation computation: {e}")
                return None

        target_pose_msg = self._target_pose_msg
        target_pose_msg.header.stamp = self.get_clock().now().to_msg()
        target_pose_msg.pose = Pose(
            position=Point(
                x=self.virtual_pose.pose.position.x,
                y=self.virtual_pose.pose.position.y,
                z=self.virtual_pose.pose.position.z,
            ),
            orientation=Quaternion(
                x=self.virtual_pose.pose.orientation.x,
                y=self.virtual_pose.pose.orientation.y,
                z=self.virtual_pose.pose.orientation.z,
                w=self.virtual_pose.pose.orientation.w,
            ),
        )
        return target_pose_msg

    def execute_line(self, current_pose: Pose) -> bool:
        """Execute linear trajectory between current and target poses.

        Args:
            current_pose: Current end-effector pose

        Returns:
            True if trajectory execution is complete, False otherwise
        """
        try:
            if self.interpolated_line_poses is None:
                self.interpolated_line_poses = self.generate_interpolated_line(
                    current_pose, self.current_target_pose
                )

            pose_stamped = self.interpolated_line_poses[self.line_progression_index]
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.desired_pose_pub.publish(pose_stamped)

            self.line_progression_index = min(
                self.line_progression_index + 1, len(self.interpolated_line_poses) - 1
            )

            if self.line_progression_index >= len(self.interpolated_line_poses) - 1:
                error = self._get_position_error(current_pose, self.current_target_pose)
                if error <= POSITION_EPSILON:
                    self._finalize_movement()
                    self.get_logger().info("Linear welding finished")
                    return True
                else:
                    return False
            return False

        except Exception as e:
            self.get_logger().error(f"Trajectory execution error : {e}")
            return False

    def generate_interpolated_line(
        self, pose1: Pose, pose2: Pose
    ) -> List["PoseStamped"]:
        """Generate interpolated trajectory between two poses.

        Creates smooth trajectory with position and orientation interpolation
        based on configured TCP speed.

        Args:
            pose1: Start pose
            pose2: End pose

        Returns:
            List of interpolated poses forming the trajectory
        """
        try:
            pos1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])

            pos2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])

            self.position_distance = np.linalg.norm(pos1 - pos2)

            if self.position_distance < 1e-10:
                self.get_logger().warning("Start and end position are identical")
                return [
                    PoseStamped(
                        header=Header(frame_id=self.__base_frame_robot), pose=pose1
                    )
                ]

            quat1 = np.array(
                [
                    pose1.orientation.x,
                    pose1.orientation.y,
                    pose1.orientation.z,
                    pose1.orientation.w,
                ]
            )

            quat2 = np.array(
                [
                    pose2.orientation.x,
                    pose2.orientation.y,
                    pose2.orientation.z,
                    pose2.orientation.w,
                ]
            )

            quat1_norm = np.linalg.norm(quat1)
            quat2_norm = np.linalg.norm(quat2)

            if quat1_norm < 1e-10 or quat2_norm < 1e-10:
                raise ValueError("Invalid quaternion with zero norm")

            quat1 = quat1 / quat1_norm
            quat2 = quat2 / quat2_norm

            dot_product = np.clip(np.abs(np.dot(quat1, quat2)), 0.0, 1.0)
            orientation_distance = np.arccos(dot_product) / pi

            max_distance = self.tcp_speed * 1 / ROBOT_RATE

            if max_distance <= 0:
                raise ValueError(f"Invalid TCP speed {self.tcp_speed}")

            num_points = int(
                max(self.position_distance, orientation_distance) / max_distance + 2
            )

            rotation_interpolation = Slerp(
                [0, num_points], R.from_quat([list(quat1), list(quat2)])
            )

            interpolated_line_poses = []
            for increments in range(num_points + 1):
                point = pos1 + (increments / num_points) * (pos2 - pos1)
                quat = rotation_interpolation(increments).as_quat()
                intermediate_pose = PoseStamped(
                    header=Header(frame_id=self.__base_frame_robot),
                    pose=Pose(
                        position=Point(x=point[0], y=point[1], z=point[2]),
                        orientation=Quaternion(
                            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                        ),
                    ),
                )
                interpolated_line_poses.append(intermediate_pose)
            return interpolated_line_poses
        except (ValueError, ZeroDivisionError, np.linalg.LinAlgError) as e:
            raise ValueError(f"Mathematical error in linear trajectory generation: {e}")
        except Exception as e:
            raise RuntimeError(f"Unexpected error in linear trajectory generation: {e}")

    def _finalize_movement(self) -> None:
        """Finalize trajectory execution and publish completion status.

        Called when a trajectory (cartesian or joint) completes successfully.
        Publishes status message with robot data.
        """
        status_msg = CommandStatus()
        status_msg.command_type = (
            Command.COMMAND_PLAY_CARTESIAN
            if self.current_state == RobotState.CARTESIAN_TRAJECTORY
            else Command.COMMAND_PLAY_JOINT
        )
        status_msg.success = True
        status_msg.message = "Movement completed successfully"
        status_msg.robot_data = self._create_robot_data()

        self.status_pub.publish(status_msg)
        self.get_logger().info("Movement finished")

    def _create_robot_data(self) -> RobotData:
        """Create robot data message with current state.

        Returns:
            RobotData message containing current pose, fault status, and error info
        """
        try:
            current_pose = self._get_current_pose(mimic=False)
            self._robot_data_msg.pose = (
                self.virtual_pose.pose
                if self.virtual_pose_initialized
                else current_pose.pose
            )
            self.tcp_pose = current_pose.pose
            self._robot_data_msg.timestamp = current_pose.header.stamp

        except LookupError as e:
            self.get_logger().debug(f"Transform not available : {e}")
            default_pose = Pose()
            self._robot_data_msg.pose = default_pose
            self.tcp_pose = default_pose
            self._robot_data_msg.timestamp = self.get_clock().now().to_msg()
        except Exception as e:
            self.get_logger().error(f"Unexpected error creating robot data: {e}")
            default_pose = Pose()
            self._robot_data_msg.pose = default_pose
            self.tcp_pose = default_pose
            self._robot_data_msg.timestamp = self.get_clock().now().to_msg()

        self._robot_data_msg.robot_in_fault_status = self.robot_in_fault_status
        self._robot_data_msg.error_message = self.current_error
        return self._robot_data_msg

    def switch_control_mode(self, mode: int) -> Optional[Any]:
        """Switch robot control mode defined in the multiple command code.

        Args:
            mode: Control mode (0=PAUSE, 1=CARTESIAN, 2=MODULAR)

        Returns:
            Service call future, or None if failed
        """
        if self.current_control_mode == mode:
            return

        try:
            if not self.change_control_mode_client.wait_for_service(
                timeout_sec=SERVICE_TIMEOUT
            ):
                raise RuntimeError(
                    f"Control mode service not available after {SERVICE_TIMEOUT}s"
                )

            request = ServoCommandType.Request(command_type=mode)
            future = self.change_control_mode_client.call_async(request)

            self.resuming_flag = (
                self.current_control_mode == ControlMode.PAUSE
                and mode != ControlMode.PAUSE
            )
            self.current_control_mode = mode

            self.get_logger().info(f"Switched control mode to {mode}")

            for _ in range(2): 
                time.sleep(0.5)
                self.motor_lock_pub.publish(self._motor_lock_msg)

            return future

        except Exception as e:
            self.current_error = f"Failed to switch control mode: {e}"
            self.get_logger().error(self.current_error)
            self.robot_in_fault_status = True
            return None

    def on_clear_tcp_trace(self, _: Empty) -> None:
        """Handle TCP trace clearing command from RViz plugin.

        Args:
            msg: Empty message triggering trace clear
        """
        try:
            self.tf_path_trail_nb_base_link_to_nb_tcp_client_clear_path.call_async(
                Empty_srv.Request()
            )
            self.tf_path_trail_nb_mimic_base_link_to_nb_mimic_tcp_wrist_client_clear_path.call_async(
                Empty_srv.Request()
            )
            self.get_logger().info("TCP trace cleared")
        except Exception as e:
            self.get_logger().error(f"Error clearing TCP trace: {e}")

    def on_pos_gain_changed(self, msg: Float64) -> None:
        """Handle position gain parameter update from RViz plugin.

        Args:
            msg: New position gain value
        """
        new_gain = msg.data
        self.set_parameters(
            [RclpyParameter("pos_gain", RclpyParameter.Type.DOUBLE, new_gain)]
        )
        self.get_logger().info(f"Position gain updated to: {new_gain:.2f}")

    def on_quat_gain_changed(self, msg: Float64) -> None:
        """Handle quaternion gain parameter update from RViz plugin.

        Args:
            msg: New quaternion gain value
        """
        new_gain = msg.data
        self.set_parameters(
            [RclpyParameter("quat_gain", RclpyParameter.Type.DOUBLE, new_gain)]
        )
        self.get_logger().info(f"Quaternion gain updated to: {new_gain:.2f}")

    def on_emergency_stop(self, msg: Bool) -> None:
        """Handle emergency stop signal from RViz plugin.

        Immediately stops robot motion and switches to pause mode when activated.

        Args:
            msg: Boolean indicating emergency stop state
        """
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().info("EMERGENCY STOP")
            self.switch_control_mode(0)
            self.current_state = RobotState.PAUSE
        else:
            if self.current_state == RobotState.PAUSE:
                self.current_state = RobotState.IDLE

    def on_modular_status(self, msg: Bool) -> None:
        """Handle modular control status updates"""
        if msg.data != self.modular_control_enabled:
            self.modular_control_enabled = msg.data
            if self.modular_control_enabled:
                self.current_state = RobotState.MODULAR_CONTROL
            elif self.current_state == RobotState.MODULAR_CONTROL:
                self.current_state = RobotState.IDLE

    def on_modular_control_mode_request(self, msg: Int8) -> None:
        """Handle control mode change requests from welding modular control"""
        mode = msg.data
        self.switch_control_mode(mode)


def main(args=None):
    rclpy.init(args=args)
    welding_command_handler_node = WeldingCommandHandlerNode(
        "welding_cartesian_command_node"
    )
    try:
        rclpy.spin(welding_command_handler_node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger("rclpy").info("signal_handler(signum=2)")
    except ShutdownException:
        welding_command_handler_node.get_logger().info("ShutdownException...")
    except Exception as e:
        welding_command_handler_node.current_error = str(e)
        welding_command_handler_node.get_logger().error(f"Exception : {e}")
    finally:
        welding_command_handler_node.destroy_node()
        try_shutdown()


if __name__ == "__main__":
    main()
