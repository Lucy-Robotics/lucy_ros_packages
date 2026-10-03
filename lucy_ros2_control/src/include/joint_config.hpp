// Copyright 2025 Sentience Robotics Team
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

/// @file joint_config.hpp
/// @brief Pure parsing / validation / mapping helpers for ``LucySystemHardware``.
///
/// These free functions hold the side-effect-free core of ``on_init``: reading
/// joint parameters, validating ros2_control interfaces, building actuator
/// mappings and computing servo angles. They take plain
/// ``hardware_interface`` structs (no live ROS node), so they are unit-tested
/// in isolation (see ``test/test_joint_config.cpp``).

#ifndef LUCY_ROS2_CONTROL__JOINT_CONFIG_HPP_
#define LUCY_ROS2_CONTROL__JOINT_CONFIG_HPP_

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"

namespace lucy_ros2_control
{

enum class Type
{
  PWM_SERVO,
  BUS_SERVO
};

/// Convert radians to degrees (UI / diagnostics only).
double rad_to_deg(double rad);

/// Convert degrees to radians (UI / diagnostics only).
double deg_to_rad(double deg);

/// URDF joint-space command envelope (radians); ``±inf`` means "unbounded".
struct JointLimits
{
  double min_rad{-std::numeric_limits<double>::infinity()};
  double max_rad{std::numeric_limits<double>::infinity()};
};

/// Per-actuator calibration + URDF limits parsed from one ros2_control joint.
struct ActuatedJointMapping
{
  std::size_t joint_index{0};
  Type type{0};
  int virtual_pin{0};
  int bus_id{0};
  double offset_rad{0.0};
  double direction{1.0};
  double scale{1.0};
  double servo_min_rad{0.0};
  double servo_max_rad{0.0};
  double servo_default_rad{0.0};
  /// URDF joint-space limits from command_interface min/max (rad).
  double min_rad{-std::numeric_limits<double>::infinity()};
  double max_rad{std::numeric_limits<double>::infinity()};
};

/// Parse a numeric string. Throws ``std::runtime_error`` (with ``context``) on
/// any non-numeric input.
double parse_double_string_or_throw(const std::string & raw, const std::string & context);

/// Fetch and parse a required joint parameter. Throws ``std::runtime_error``
/// when the key is missing, empty, or non-numeric.
double parse_required_double(
  const hardware_interface::ComponentInfo & joint,
  const std::string & key);

/// Parse an optional interface min/max value. Returns ``fallback`` when unset
/// or empty; throws ``std::runtime_error`` when present but non-numeric.
double parse_optional_interface_limit(
  const std::optional<std::string> & value,
  const std::string & context,
  double fallback);

/// Validate that a joint exposes exactly one position command interface and one
/// position state interface. Returns an empty string when valid, otherwise a
/// human-readable error message.
std::string validate_joint_interfaces(const hardware_interface::ComponentInfo & joint);

/// Parse the URDF command_interface ``min``/``max`` for a joint (radians).
/// Missing values default to ``±inf``. Throws ``std::runtime_error`` on
/// non-numeric input.
JointLimits parse_joint_limits(const hardware_interface::ComponentInfo & joint);

/// Build an actuator mapping for a joint, attaching the URDF limits
/// (``min_rad``/``max_rad``). Returns ``std::nullopt`` when the joint has no
/// ``virtual_pin`` (passive / unmapped). Throws ``std::runtime_error`` on any
/// invalid or non-numeric parameter (bad virtual_pin, zero direction/scale,
/// inverted servo or URDF range).
std::optional<ActuatedJointMapping> build_actuated_joint_mapping(
  const hardware_interface::ComponentInfo & joint,
  std::size_t joint_index,
  double min_rad,
  double max_rad);

/// Default joint position (rad) derived from the actuator's ``servo_default_rad``.
double default_joint_position_rad(const ActuatedJointMapping & m);

/// Servo angle (rad) published to firmware for a joint-space command (rad):
/// maps joint→servo radians, clamps to ``[servo_min_rad, servo_max_rad]``.
double actuator_command_to_servo_rad(const ActuatedJointMapping & m, double cmd_rad);

/// Sort ``mappings`` ascending by ``virtual_pin`` and return the first repeated
/// pin, or ``std::nullopt`` when all pins are unique.
std::optional<int> sort_and_find_duplicate_virtual_pin(
  std::vector<ActuatedJointMapping> & mappings);

}  // namespace lucy_ros2_control

#endif  // LUCY_ROS2_CONTROL__JOINT_CONFIG_HPP_
