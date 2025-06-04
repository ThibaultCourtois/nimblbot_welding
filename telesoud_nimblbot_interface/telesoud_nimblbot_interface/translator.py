import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from interface_custom_msgs.msg import TelesoudInstruction, RobotData, Command, CommandStatus

class TranslatorNode(Node):
    def __init__(self):
        super().__init__('translator_node')
        
        # Instruction from Telesoud
        self.last_instruction_code = None
        self.pending_instruction = None
        self.instruction_data = None

        # Command for welding_command
        self.next_command_id = 0
        self.pending_commands = {}

        self.instruction_timer = self.create_timer(0.02, self.process_pending_instruction)
        
        self._create_subscriptions()
        self._create_publishers()
                       
    
    def _create_subscriptions(self):
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
    
    
    def _create_publishers(self):
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

    
    def _on_telesoud_instructions(self, msg):
        """Handle incoming Telesoud instructions"""
        self.instruction_data = {
            'instruction': msg.instruction_code,
            'pose1': msg.pose1,
            'tcp_speed_vector': msg.speed_vector,
            'tcp_speed': msg.speed,
            'free_drive_btn_status': msg.free_drive_btn_status
        }
        self.pending_instruction = True
    
    
    def process_pending_instruction(self):
        """Process pending instruction, called by timer"""
        if not self.pending_instruction:
            return
            
        data = self.instruction_data
        instruction = data['instruction']
        pose1 = data['pose1']
        tcp_speed_vector = data['tcp_speed_vector']
        tcp_speed = data['tcp_speed']

        self.pending_instruction = False

        command = Command()
        command.command_id = self.next_command_id
        self.next_command_id += 1

        self.pending_commands[command.command_id] = {
            'instruction': instruction,
            'timestamp': self.get_clock().now()
        }

        if instruction is not None:
            match instruction:
                case 0:
                    command.command_type = Command.COMMAND_STOP
                    self.get_logger().debug('Instruction STOP')
                    
                case 1:
                    command.command_type = Command.COMMAND_GET_ROBOT_DATA
                    self.get_logger().debug('Instruction GET ROBOT DATA')
                    
                case 7:
                    command.command_type = Command.COMMAND_SET_DYNAMIC
                    self.get_logger().debug('Instruction SET DYNAMIC CARTESIAN MOVEMENT')

                    if tcp_speed_vector is not None and len(pose1) > 0:
                        command.speed_vector.linear.x = tcp_speed_vector[0]
                        command.speed_vector.linear.y = tcp_speed_vector[1]
                        command.speed_vector.linear.z = tcp_speed_vector[2]
                        command.speed_vector.angular.x = tcp_speed_vector[3]
                        command.speed_vector.angular.y = tcp_speed_vector[4]
                        command.speed_vector.angular.z = tcp_speed_vector[5]
                        
                case 8:
                    command.command_type = Command.COMMAND_START_DYNAMIC
                    self.get_logger().debug('Instruction START DYNAMIC CARTESIAN MOVEMENT')
                    
                case 15:
                    command.command_type = Command.COMMAND_PLAY_CARTESIAN
                    self.get_logger().info('Instruction PLAY CARTESIAN TRAJECTORY')

                    if tcp_speed is not None:
                        command.speed = tcp_speed

                    if pose1 is not None and len(pose1) > 0:
                        command.target_pose = self._construct_command_pose(pose1)
                        
                case 16:
                    command.command_type = Command.COMMAND_PLAY_JOINT
                    self.get_logger().info('Instruction PLAY JOINT TRAJECTORY')

                    if tcp_speed is not None:
                        command.speed = tcp_speed

                    if pose1 is not None and len(pose1) > 0:
                        command.target_pose = self._construct_command_pose(pose1)
        
        self.command_pub.publish(command)
        self.last_instruction_code = instruction

    
    def _on_command_status(self, msg):
        """Handle command status feedback"""
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

    
    def _forward_robot_data(self, robot_data):
        """Forward robot data to Telesoud"""
        try:
            robot_data_msg = RobotData()
            robot_data_msg.pose = robot_data.pose
            
            euler = euler_from_quaternion([
                robot_data.pose.orientation.x,
                robot_data.pose.orientation.y,
                robot_data.pose.orientation.z,
                robot_data.pose.orientation.w
            ])
                
            xyzwpr = [
                robot_data.pose.position.x,
                robot_data.pose.position.y,
                robot_data.pose.position.z,
                euler[0],  # Telesoud expects radians for the orientation
                euler[1],
                euler[2]
            ]
                
            robot_data_msg.robot_in_fault_status = robot_data.robot_in_fault_status
            robot_data_msg.error_message = robot_data.error_message
            robot_data_msg.xyzwpr = xyzwpr
            robot_data_msg.robot_in_slave_mode_status = False
            robot_data_msg.collision_status = False
            robot_data_msg.emergency_stop = False
            robot_data_msg.welding_trigger_plc_signal = False
            robot_data_msg.operation_mode = 1
            
            self.robot_data_pub.publish(robot_data_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error forwarding robot data: {e}')

    
    def _construct_command_pose(self, pose):
        """Construct pose message from pose array"""
        command = Command()
        
        command.target_pose.position.x = pose[0]
        command.target_pose.position.y = pose[1]
        command.target_pose.position.z = pose[2]

        quaternion = quaternion_from_euler(pose[3], pose[4], pose[5])

        command.target_pose.orientation.x = quaternion[0]
        command.target_pose.orientation.y = quaternion[1]
        command.target_pose.orientation.z = quaternion[2]
        command.target_pose.orientation.w = quaternion[3]
        
        return command.target_pose


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



