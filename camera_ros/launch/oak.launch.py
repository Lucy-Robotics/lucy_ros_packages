# Copyright 2026 Lucy Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Luxonis OAK-D (depthai-ros) plus a JET depth colorizer for the control panel."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetRemap


def generate_launch_description():
    depthai = get_package_share_directory("depthai_ros_driver")
    params = f"{get_package_share_directory('camera_ros')}/config/oak.yaml"
    return LaunchDescription(
        [
            GroupAction(
                [
                    # The driver's own state publisher would replace the robot's model.
                    SetRemap(src="robot_description", dst="/oak/robot_description"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            f"{depthai}/launch/camera.launch.py"
                        ),
                        launch_arguments=[("params_file", params)],
                    ),
                ],
                scoped=True,
            ),
            Node(
                package="camera_ros",
                executable="depth_colorizer.py",
                name="oak_depth_colorizer",
                output="screen",
                remappings=[
                    ("depth", "/oak/stereo/image_raw"),
                    ("depth/colorized/compressed", "/oak/stereo/colorized/compressed"),
                ],
            ),
        ]
    )
