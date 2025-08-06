import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction


def generate_launch_description():
    namespace_param = DeclareLaunchArgument(
        "robot_namespace", default_value="nb", description=""
    )

    robot_type_param = DeclareLaunchArgument(
        "robot_type", default_value="nb120_3m_welding", description=""
    )

    simulation_param = DeclareLaunchArgument(
        "simulation",
        default_value="true",
        description="Simulation of the robot",
        choices=["true", "false"],
    )

    simulate_dynamics_param = DeclareLaunchArgument(
        "simulate_dynamics",
        default_value="false",
        description="Simulation of dynamics",
        choices=["true", "false"],
    )

    rviz_moveit_param = DeclareLaunchArgument(
        "rviz_moveit",
        default_value="true",
        description="Launch rviz for this robot",
        choices=["true", "false"],
    )

    rviz_template_param = DeclareLaunchArgument(
        "rviz_template",
        default_value=os.path.join(
            get_package_share_directory("telesoud_nimblbot_interface"),
            "config",
            "projet_soudure.rviz",
        ),
        description="Template Rviz",
    )

    welding_scene_param = DeclareLaunchArgument(
        "welding_scene",
        default_value="standard",
        description="Welding scene",
    )

    robot_namespace = LaunchConfiguration(
        "robot_namespace", default=namespace_param.default_value
    )
    robot_type = LaunchConfiguration(
        "robot_type", default=robot_type_param.default_value
    )
    simulation = LaunchConfiguration(
        "simulation", default=simulation_param.default_value
    )
    simulate_dynamics = LaunchConfiguration(
        "simulate_dynamics", default=simulate_dynamics_param.default_value
    )
    rviz_moveit = LaunchConfiguration(
        "rviz_moveit", default=rviz_moveit_param.default_value
    )
    rviz_template = LaunchConfiguration(
        "rviz_template", default=rviz_template_param.default_value
    )
    welding_scene = LaunchConfiguration(
        "welding_scene", default=welding_scene_param.default_value
    )
    translator = Node(
        package="telesoud_nimblbot_interface",
        executable="translator",
        name="translator_node",
        output="log",
    )

    welding_command_handler = Node(
        package="telesoud_nimblbot_interface",
        executable="welding_command_handler",
        name="welding_command_handler_node",
        output="log",
        parameters=[{"welding_scene": welding_scene}],
    )

    welding_modular_control = Node(
        package="telesoud_nimblbot_interface",
        executable="welding_modular_control",
        name="welding_modular_control_node",
        output="log",
        parameters=[
            {
                "robot_type": robot_type,
            }
        ],
    )

    telesoud_api = Node(
        package="telesoud_api",
        executable="telesoud_api",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )

    nb_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("nimblbot_bringup"), "launch"),
                "/nb_wrapper_launch.py",
            ]
        ),
        launch_arguments={
            "robot_type": robot_type,
            "control_mode": "6",
            "robot_namespace": robot_namespace,
            "rviz": "false",
            "simulate_dynamics": simulate_dynamics,
            "random_init": "false",
            "simulation": simulation,
        }.items(),
    )

    mesh_torche_soudure = Node(
        package="nimblbot_simulation_scenes",
        executable="mesh_publisher",
        name="mesh_outil_soudure",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.075",
            "0.0",
            "1.5707",
            "3.14159265359",
            "nb/wrist",
            "file://"
            + os.path.join(
                get_package_share_directory("telesoud_nimblbot_interface"),
                "welding_meshes",
                "TorchBracket.stl",
            ),
            "0.001",
            "0.001",
            "0.001",
            "0.2",
            "0.23",
            "0.38",
            "1.0",
        ],
    )

    robot_moveit_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nimblbot_bringup"),
                    "launch",
                ),
                "/moveit_launch.py",
            ]
        ),
        launch_arguments={
            "robot_type": robot_type,
            "robot_namespace": robot_namespace,
            "rviz_moveit": rviz_moveit,
            "rviz_template": rviz_template,
        }.items(),
    )

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        output="log",
        parameters=[
            {
                "video_device": "/dev/video2",
                "framerate": 30.0,
                "image_width": 640,
                "image_height": 480,
            }
        ],
    )

    tf_path_trail_base_link_wrist = Node(
        package="nimblbot_cartesian_command",
        executable="tf_path_trail",
        name="tf_path_trail_base_link_wrist",
        output="screen",
        arguments=["nb/base_link", "nb/tcp_wrist"],
    )

    tf_path_trail_base_link_wrist_mimic = Node(
        package="nimblbot_cartesian_command",
        executable="tf_path_trail",
        name="tf_path_trail_base_link_wrist_mimic",
        output="screen",
        arguments=["nb_mimic/base_link", "nb_mimic/tcp_wrist"],
    )

    tf2_world_node_vertical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_base_link",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "nb/base_link",
        ],
        condition=UnlessCondition(
            PythonExpression(['"', welding_scene, '".endswith("H")'])
        ),
    )

    tf2_world_node_horizontal = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_base_link",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0.776",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "-1.57057",
            "--frame-id",
            "world",
            "--child-frame-id",
            "nb/base_link",
        ],
        condition=IfCondition(
            PythonExpression(['"', welding_scene, '".endswith("H")'])
        ),
    )

    welding_scene_publisher_node = Node(
        package="welding_scene_publisher",
        executable="welding_scene_publisher",
        name="welding_scene_publisher",
        output="screen",
        parameters=[{"welding_scene": welding_scene}],
    )

    return LaunchDescription(
        [
            TimerAction(period=0.0, actions=[nb_nodes]),
            TimerAction(period=1.0, actions=[robot_moveit_nodes]),
            TimerAction(
                period=2.0, actions=[tf2_world_node_horizontal, tf2_world_node_vertical]
            ),
            TimerAction(
                period=3.0,
                actions=[
                    telesoud_api,
                    translator,
                    welding_command_handler,
                    welding_modular_control,
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    tf_path_trail_base_link_wrist,
                    tf_path_trail_base_link_wrist_mimic,
                    mesh_torche_soudure,
                    welding_scene_publisher_node,
                ],
            ),
            TimerAction(period=5.0, actions=[usb_cam_node]),
        ]
    )
