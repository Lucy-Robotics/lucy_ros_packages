#!/usr/bin/env python3
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

"""
Launch file for Intel RealSense D435i camera.

Optimized for hand tracking at 30-60cm range (robot head-mounted camera).

Based on official realsense-ros documentation:
https://github.com/realsenseai/realsense-ros

Features:
- Depth-to-color alignment for RGBD
- Colorizer filter for JPEG compression
- Post-processing filters optimized for hand tracking:
  * Spatial filter: Edge-preserving smoothing
  * Temporal filter: Noise reduction over time
  * Hole filling: Fills gaps in depth data
- Depth module preset optimized for close-range hand detection
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        # Enable required streams
        'enable_color': True,
        'enable_depth': True,

        # Disable unused streams
        'enable_gyro': False,
        'enable_accel': False,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_pointcloud': False,

        # Enable RGBD topic (combines color and depth into RGB-D image)
        'enable_rgbd': True,
        'align_depth.enable': True,  # Required for RGBD
        'enable_sync': True,  # Required for RGBD synchronization

        # Depth module optimization for hand tracking (30-60cm range)
        # Preset values: 0=Custom, 1=Default, 2=Hand, 3=High Accuracy,
        # 4=High Density, 5=Medium Density
        'depth_module.visual_preset': 2,  # 2=Hand preset (optimized for hand tracking)

        # Enable colorizer filter to colorize depth for JPEG compression
        'colorizer.enable': True,
        'colorizer.visual_preset': 2,  # 0=Jet, 1=Classic, 2=WhiteToBlack, 3=BlackToWhite, etc.

        # Post-processing filters for improved depth quality at close range
        # Spatial filter: Edge-preserving smoothing (reduces noise, maintains edges)
        'spatial_filter.enable': True,
        'spatial_filter.filter_magnitude': 2,  # 1-5, higher = more smoothing
        'spatial_filter.filter_smooth_alpha': 0.5,  # 0.25-0.75, edge-preserving strength
        'spatial_filter.filter_smooth_delta': 20,  # 1-50, edge threshold
        'spatial_filter.holes_fill': 1,  # 0-5, fill small holes

        # Temporal filter: Reduces noise by averaging over time (good for hand tracking)
        'temporal_filter.enable': True,
        'temporal_filter.filter_smooth_alpha': 0.4,  # 0.0-1.0, persistence weight
        'temporal_filter.filter_smooth_delta': 20,  # Edge threshold
        'temporal_filter.holes_fill': 1,  # Fill temporal holes

        # Hole filling filter: Fills remaining gaps in depth data
        'hole_filling_filter.enable': True,
        # 0=fill_from_left, 1=farest_from_around, 2=nearest_from_around
        'hole_filling_filter.holes_fill': 1,

        # Disable decimation filter (not needed for close-range, reduces resolution)
        'decimation_filter.enable': False,
    }

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='realsense',
        parameters=[params],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        realsense_node,
    ])
