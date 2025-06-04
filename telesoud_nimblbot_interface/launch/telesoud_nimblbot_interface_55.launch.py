import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import (
        DeclareLaunchArgument,
        IncludeLaunchDescription,
        TimerAction
    )

def generate_launch_description():

    namespace_param = DeclareLaunchArgument(
            'robot_namespace',
            default_value = 'nb',
            description = ''
        )

    robot_type_param = DeclareLaunchArgument(
            "robot_type",
            default_value = 'nb55_v7_welding',
            description = ''
        )

    simulation_param = DeclareLaunchArgument(
            'simulation',
            default_value = 'true',
            description = 'Simulation of the robot',
            choices=['true', 'false']
        )

    simulate_dynamics_param = DeclareLaunchArgument(
            'simulate_dynamics',
            default_value = 'false',
            description="Simulation of dynamics",
            choices=['true', 'false']
        )
    
    rviz_moveit_param = DeclareLaunchArgument(
            'rviz_moveit',
            default_value = 'true',
            description = 'Launch rviz for this robot',
            choices = ['true', 'false']
        )

    rviz_template_param = DeclareLaunchArgument(
            'rviz_template',
            default_value = os.path.join(get_package_share_directory('telesoud_nimblbot_interface'), 'config', 'projet_soudure.rviz'),
            description = 'Template Rviz',
        )
    
    robot_namespace = LaunchConfiguration('robot_namespace', default=namespace_param.default_value)
    robot_type = LaunchConfiguration('robot_type', default=robot_type_param.default_value)
    simulation = LaunchConfiguration('simulation', default=simulation_param.default_value)
    simulate_dynamics = LaunchConfiguration('simulate_dynamics', default=simulate_dynamics_param.default_value)
    rviz_moveit = LaunchConfiguration('rviz_moveit', default=rviz_moveit_param.default_value)
    rviz_template = LaunchConfiguration('rviz_template', default=rviz_template_param.default_value)

    translator = Node(
            package='telesoud_nimblbot_interface',
            executable='translator',
            name='translator_node',
            output='log'
        )
    
    welding_command_handler = Node(
            package='telesoud_nimblbot_interface',
            executable= 'welding_command_handler',
            name='welding_command_handler_node',
            output='log',
            parameters=[{
                'robot_type':robot_type,
                }]
            )

    telesoud_api = Node(
            package='telesoud_api',
            executable= 'telesoud_api',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            )

    nb_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('nimblbot_bringup'),
                    'launch'
                    ), '/nb_wrapper_launch.py'
            ]),
            launch_arguments = {
                'robot_type':robot_type,
                'control_mode':'6',
                'robot_namespace':robot_namespace,
                'rviz':'false',
                'simulate_dynamics':simulate_dynamics,
                'random_init':'false',
                'simulation':simulation
            }.items()
        )
    
    mesh_torche_soudure = Node(
            package='nimblbot_simulation_scenes',
            executable='mesh_publisher',
            name='mesh_outil_soudure',
            output='log',
            arguments = [
                    '0.0',
                    '0.0',
                    '0.03',
                    '0.0',
                    '-1.5707',
                    '0.0',
                    'nb/wrist',
                    'file://' + os.path.join(get_package_share_directory('telesoud_nimblbot_interface'), 'welding_meshes', 'NB55_torche_laser_low.stl'),
                    '0.001',
                    '0.001',
                    '0.001',
                    '0.2',
                    '0.23',
                    '0.38',
                    '1.0']
            )

    robot_moveit_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('nimblbot_bringup'),
                    'launch',
                    ), '/moveit_launch.py'
                ]),
            launch_arguments = {
                'robot_type':robot_type,
                'robot_namespace':robot_namespace,
                'rviz_moveit':rviz_moveit,
                'rviz_template':rviz_template,
            }.items()
        )

    usb_cam_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output = 'log',
            parameters = [{
                'video_device':'/dev/video2',
                'framerate':30.0,
                'image_width':640,
                'image_height':480,
                }]
            )
    
    tf_path_trail_base_link_wrist = Node(
        package='nimblbot_cartesian_command',
        executable='tf_path_trail',
        name='tf_path_trail_base_link_wrist',
        output='screen',
        arguments=['nb/base_link', 'nb/tcp_wrist'],
    )
    
    tf_path_trail_base_link_wrist_mimic = Node(
        package='nimblbot_cartesian_command',
        executable='tf_path_trail',
        name='tf_path_trail_base_link_wrist_mimic',
        output='screen',
        arguments=['nb_mimic/base_link', 'nb_mimic/tcp_wrist'],
    )
    
    welding_scene_publisher_node = Node(
            package='welding_scene_publisher',
            executable='welding_scene_publisher',
            name='welding_scene_publisher',
            output='screen',
    )

    return LaunchDescription([
            TimerAction(period=0.5, 
                actions = [nb_nodes]
            ),
            mesh_torche_soudure, 
            robot_moveit_nodes,
            TimerAction(period=5.0, 
                actions=[telesoud_api, translator, welding_command_handler]
            ),
            usb_cam_node,
            tf_path_trail_base_link_wrist,
            tf_path_trail_base_link_wrist_mimic,
            welding_scene_publisher_node
        ])

