// Copyright 2026 Lucy Robotics Team
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/// @file position_limit_clamp.hpp
/// @brief URDF position-limit clamp for ``LucySystemHardware`` (real + mock).
///
/// Clamps position commands to the ros2_control command_interface
/// ``<param name="min/max">`` envelope before actuator mapping. Gazebo uses
/// stock ``gz_ros2_control`` and does not apply this helper.

#ifndef LUCY_ROS2_CONTROL__POSITION_LIMIT_CLAMP_HPP_
#define LUCY_ROS2_CONTROL__POSITION_LIMIT_CLAMP_HPP_

#include <algorithm>
#include <cmath>

namespace lucy_ros2_control
{

/// Clamp ``cmd`` to ``[min, max]`` ignoring non-finite bounds (treated as no
/// bound on that side). Equivalent to ``std::min(std::max(cmd, min), max)``
/// when both bounds are finite.
inline double clamp_position_command(double cmd, double min, double max)
{
  if (std::isfinite(min)) {
    cmd = std::max(cmd, min);
  }
  if (std::isfinite(max)) {
    cmd = std::min(cmd, max);
  }
  return cmd;
}

}  // namespace lucy_ros2_control

#endif  // LUCY_ROS2_CONTROL__POSITION_LIMIT_CLAMP_HPP_
