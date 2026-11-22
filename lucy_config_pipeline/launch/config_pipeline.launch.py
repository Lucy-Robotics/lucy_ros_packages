from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_pkg_arg = DeclareLaunchArgument("robot_package", default_value="thais_urdf")
    config_dir_arg = DeclareLaunchArgument("config_dir", default_value="")
    real_arg = DeclareLaunchArgument("real", default_value="true")

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
        parameters=[
            {
                "robot_package": LaunchConfiguration("robot_package"),
                "config_dir": LaunchConfiguration("config_dir"),
            }
        ],
    )

    robot_select_node = Node(
        package="lucy_config_pipeline",
        executable="robot_select_node",
        output="screen",
        parameters=[{"robot_package": LaunchConfiguration("robot_package")}],
    )

    # Separate process for the same reason. Robots with no calibration profile just idle.
    calibration_node = Node(
        package="lucy_config_pipeline",
        executable="calibration_node",
        output="screen",
        parameters=[
            {
                "robot_package": LaunchConfiguration("robot_package"),
                "config_dir": LaunchConfiguration("config_dir"),
                "real": LaunchConfiguration("real"),
            }
        ],
    )

    return LaunchDescription(
        [
            robot_pkg_arg,
            config_dir_arg,
            real_arg,
            node,
            registry_node,
            robot_select_node,
            calibration_node,
        ]
    )
