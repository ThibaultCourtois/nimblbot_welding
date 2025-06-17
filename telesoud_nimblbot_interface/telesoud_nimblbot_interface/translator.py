import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from interface_custom_msgs.msg import TelesoudInstruction, RobotData, Command, CommandStatus
from typing import List
from geometry_msgs.msg import Pose

class TranslatorNode(Node):
    def __init__(self) -> None:
        """Initialize the TranslatorNode.
        
        Sets up ROS2 node for translating Telesoud instructions to robot commands.
        Initializes publishers, subscribers, and timers for instruction processing.
        """
        super().__init__('translator_node')
        
        # Instruction from Telesoud
        self.last_instruction_code = None
        self.pending_instruction = None
        self.instruction_data = None

        # Command for welding_command
        self.next_command_id = 0
        self.pending_commands = {}
        
        self._create_subscriptions()
        self._create_publishers()
        self._initialize_reusable_ros_msg_objects()

        self.instruction_timer = self.create_timer(0.02, self.process_pending_instruction)
        

    def _initialize_reusable_ros_msg_objects(self) -> None:
        ''' Initialize reusable ROS message objects to avoid repeated instantiation'''
        self._command_msg = Command()
        self._robot_data_msg = RobotData()
        
    
    def _create_subscriptions(self) -> None:
        """Create ROS2 subscriptions for incoming messages.
        
        Sets up subscriptions for:
        - Telesoud instructions
        - Command status feedback from welding command handler
        """
        self.instruction_sub = self.create_subscription(
            TelesoudInstruction, 
            '/telesoud/instructions',
            self._on_telesoud_instructions,
            10
        )

        self.command_status_sub = self.create_subscription(
            CommandStatus,
            '/welding_command_handler/command_status',
            self._on_command_status,
            10
        )
    
    
    def _create_publishers(self) -> None:
        """Create ROS2 publishers for outgoing messages.
        
        Sets up publishers for:
        - Robot commands to welding command handler
        - Robot data feedback to Telesoud
        """
        self.command_pub = self.create_publisher(
            Command,
            '/translator/command',
            10
        )

        self.robot_data_pub = self.create_publisher(
            RobotData,
            '/translator/robotData',
            10
        )

    
    def _on_telesoud_instructions(self, msg: TelesoudInstruction) -> None:
        """Handle incoming Telesoud instructions.
        
        Args:
            msg: TelesoudInstruction message containing instruction code, poses, and parameters
        """
        self.instruction_data = {
            'instruction': msg.instruction_code,
            'pose1': msg.pose1,
            'tcp_speed_vector': msg.speed_vector,
            'tcp_speed': msg.speed,
            'free_drive_btn_status': msg.free_drive_btn_status
        }
        self.pending_instruction = True
    
    
    def process_pending_instruction(self) -> None:
        """Process pending Telesoud instruction and convert to robot command.
        
        Converts Telesoud instruction codes to appropriate robot commands:
        - 0: STOP
        - 1: GET_ROBOT_DATA  
        - 7: SET_DYNAMIC (speed vector)
        - 8: START_DYNAMIC
        - 15: PLAY_CARTESIAN
        - 16: PLAY_JOINT
        """
        if not self.pending_instruction:
            return
            
        data = self.instruction_data
        instruction = data['instruction']
        pose1 = data['pose1']
        tcp_speed_vector = data['tcp_speed_vector']
        tcp_speed = data['tcp_speed']

        self.pending_instruction = False

        self._command_msg.command_id = self.next_command_id
        self.next_command_id += 1

        self.pending_commands[self._command_msg.command_id] = {
            'instruction': instruction,
            'timestamp': self.get_clock().now()
        }

        if instruction is not None:
            match instruction:
                case 0:
                    self._command_msg.command_type = Command.COMMAND_STOP
                    self.get_logger().debug('Instruction STOP')
                    
                case 1:
                    self._command_msg.command_type = Command.COMMAND_GET_ROBOT_DATA
                    self.get_logger().debug('Instruction GET ROBOT DATA')
                    
                case 7:
                    self._command_msg.command_type = Command.COMMAND_SET_DYNAMIC
                    self.get_logger().debug('Instruction SET DYNAMIC CARTESIAN MOVEMENT')

                    if tcp_speed_vector is not None and len(pose1) > 0:
                        self._command_msg.speed_vector.linear.x = tcp_speed_vector[0]
                        self._command_msg.speed_vector.linear.y = tcp_speed_vector[1]
                        self._command_msg.speed_vector.linear.z = tcp_speed_vector[2]
                        self._command_msg.speed_vector.angular.x = tcp_speed_vector[3]
                        self._command_msg.speed_vector.angular.y = tcp_speed_vector[4]
                        self._command_msg.speed_vector.angular.z = tcp_speed_vector[5]
                        
                case 8:
                    self._command_msg.command_type = Command.COMMAND_START_DYNAMIC
                    self.get_logger().debug('Instruction START DYNAMIC CARTESIAN MOVEMENT')
                    
                case 15:
                    self._command_msg.command_type = Command.COMMAND_PLAY_CARTESIAN
                    self.get_logger().info('Instruction PLAY CARTESIAN TRAJECTORY')

                    if tcp_speed is not None:
                        self._command_msg.speed = tcp_speed

                    if pose1 is not None and len(pose1) > 0:
                        self._command_msg.target_pose = self._construct_command_pose(pose1)
                        
                case 16:
                    self._command_msg.command_type = Command.COMMAND_PLAY_JOINT
                    self.get_logger().info('Instruction PLAY JOINT TRAJECTORY')

                    if tcp_speed is not None:
                        self._command_msg.speed = tcp_speed

                    if pose1 is not None and len(pose1) > 0:
                        self._command_msg.target_pose = self._construct_command_pose(pose1)
        
        self.command_pub.publish(self._command_msg)
        self.last_instruction_code = instruction

    
    def _on_command_status(self, msg: CommandStatus) -> None:
        """Handle command status feedback from welding command handler.
        
        Args:
            msg: CommandStatus message with execution result and robot data
        """
        command_id = msg.command_id

        if command_id in self.pending_commands:
            if msg.success:
                self.get_logger().debug(f'Command {msg.command_type} executed successfully: {msg.message}')
            else:
                self.get_logger().error(f'Command {msg.command_type} failed: {msg.message}')

            if msg.robot_data is not None:
                self._forward_robot_data(msg.robot_data)
            del self.pending_commands[command_id]
        else:
            self.get_logger().warning(f'Received status for unknown command ID: {command_id}')

    
    def _forward_robot_data(self, robot_data: RobotData) -> None:
        """Forward robot data to Telesoud interface.
        
        Converts robot pose from quaternion to Euler angles and publishes
        robot status information including pose, fault status, and error messages.
        
        Args:
            robot_data: RobotData message from welding command handler
        """
        try:
            self._robot_data_msg.pose = robot_data.pose
            
            euler = euler_from_quaternion([
                self._robot_data_msg.pose.orientation.x,
                self._robot_data_msg.pose.orientation.y,
                self._robot_data_msg.pose.orientation.z,
                self._robot_data_msg.pose.orientation.w
            ])
                
            xyzwpr = [
                self._robot_data_msg.pose.position.x,
                self._robot_data_msg.pose.position.y,
                self._robot_data_msg.pose.position.z,
                euler[0],  # Telesoud expects radians for the orientation
                euler[1],
                euler[2]
            ]
                
            self._robot_data_msg.robot_in_fault_status = robot_data.robot_in_fault_status
            self._robot_data_msg.error_message = robot_data.error_message
            self._robot_data_msg.xyzwpr = xyzwpr
            self._robot_data_msg.robot_in_slave_mode_status = False
            self._robot_data_msg.collision_status = False
            self._robot_data_msg.emergency_stop = False
            self._robot_data_msg.welding_trigger_plc_signal = False
            self._robot_data_msg.operation_mode = 1
            
            self.robot_data_pub.publish(self._robot_data_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error forwarding robot data: {e}')

    
    def _construct_command_pose(self, pose: List[float]) -> Pose:
        """Construct ROS Pose message from pose array.
        
        Converts XYZWPR format (position + Euler angles) to ROS Pose
        with quaternion orientation.
        
        Args:
            pose: List of 6 floats [x, y, z, roll, pitch, yaw]
            
        Returns:
            Pose message with position and quaternion orientation
        """
        self._command_msg.target_pose.position.x = pose[0]
        self._command_msg.target_pose.position.y = pose[1]
        self._command_msg.target_pose.position.z = pose[2]

        quaternion = quaternion_from_euler(pose[3], pose[4], pose[5])

        self._command_msg.target_pose.orientation.x = quaternion[0]
        self._command_msg.target_pose.orientation.y = quaternion[1]
        self._command_msg.target_pose.orientation.z = quaternion[2]
        self._command_msg.target_pose.orientation.w = quaternion[3]
        
        return self._command_msg.target_pose


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TranslatorNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
