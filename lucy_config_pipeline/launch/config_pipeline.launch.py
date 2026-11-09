from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_pkg_arg = DeclareLaunchArgument("robot_package", default_value="thais_urdf")
    config_dir_arg = DeclareLaunchArgument("config_dir", default_value="")

    node = Node(
        package="lucy_config_pipeline",
        executable="config_pipeline_node",
        output="screen",
        parameters=[
            {
                "robot_package": LaunchConfiguration("robot_package"),
                "config_dir": LaunchConfiguration("config_dir"),
            }
        ],
    )

    # Separate process so a registry fault can never take down config services.
    registry_node = Node(
        package="lucy_config_pipeline",
        executable="client_registry_node",
        output="screen",
    )

    return LaunchDescription([robot_pkg_arg, config_dir_arg, node, registry_node])
