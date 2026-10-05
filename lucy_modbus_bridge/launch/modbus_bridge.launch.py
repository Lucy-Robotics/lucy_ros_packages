from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('node_name', default_value='lucy_hardware_interface'),
            DeclareLaunchArgument('serial_id', default_value=''),
            DeclareLaunchArgument('slave_address', default_value='1'),
            Node(
                package='lucy_modbus_bridge',
                executable='modbus_bridge_node',
                name='lucy_modbus_bridge',
                output='screen',
                parameters=[
                    {
                        'node_name': LaunchConfiguration('node_name'),
                        'serial_id': LaunchConfiguration('serial_id'),
                        'slave_address': LaunchConfiguration('slave_address'),
                    }
                ],
            ),
        ]
    )
