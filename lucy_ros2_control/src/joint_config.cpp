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

#include "include/joint_config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "include/position_limit_clamp.hpp"

namespace lucy_ros2_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double rad_to_deg(double rad)
{
  return rad * 180.0 / kPi;
}

double deg_to_rad(double deg)
{
  return deg * kPi / 180.0;
}

double parse_double_string_or_throw(const std::string & raw, const std::string & context)
{
  try {
    return hardware_interface::stod(raw);
  } catch (const std::exception & e) {
    throw std::runtime_error(
            "invalid numeric value '" + raw + "' for " + context + ": " + e.what());
  }
}

double parse_required_double(
  const hardware_interface::ComponentInfo & joint,
  const std::string & key)
{
  const auto it = joint.parameters.find(key);
  if (it == joint.parameters.end() || it->second.empty()) {
    throw std::runtime_error(
            "missing required parameter '" + key + "' for joint '" + joint.name + "'");
  }
  return parse_double_string_or_throw(
    it->second, "joint '" + joint.name + "' parameter '" + key + "'");
}

double parse_optional_interface_limit(
  const std::optional<std::string> & value,
  const std::string & context,
  double fallback)
{
  if (!value.has_value() || value->empty()) {
    return fallback;
  }
  return parse_double_string_or_throw(value.value(), context);
}

std::string validate_joint_interfaces(const hardware_interface::ComponentInfo & joint)
{
  if (joint.command_interfaces.size() != 1) {
    return "Joint '" + joint.name + "' has " +
           std::to_string(joint.command_interfaces.size()) +
           " command interfaces found. 1 expected.";
  }
  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
    return "Joint '" + joint.name + "' has '" + joint.command_interfaces[0].name +
           "' command interface. '" + hardware_interface::HW_IF_POSITION + "' expected.";
  }
  if (joint.state_interfaces.size() != 1) {
    return "Joint '" + joint.name + "' has " +
           std::to_string(joint.state_interfaces.size()) +
           " state interfaces found. 1 expected.";
  }
  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
    return "Joint '" + joint.name + "' has '" + joint.state_interfaces[0].name +
           "' state interface. '" + hardware_interface::HW_IF_POSITION + "' expected.";
  }
  return std::string();
}

JointLimits parse_joint_limits(const hardware_interface::ComponentInfo & joint)
{
  JointLimits limits{};
  if (joint.command_interfaces.empty()) {
    return limits;
  }
  const auto & cmd_if = joint.command_interfaces[0];
  if (cmd_if.name != hardware_interface::HW_IF_POSITION) {
    return limits;
  }
  limits.min_rad = parse_optional_interface_limit(
    cmd_if.min,
    "joint '" + joint.name + "' command_interface min",
    -std::numeric_limits<double>::infinity());
  limits.max_rad = parse_optional_interface_limit(
    cmd_if.max,
    "joint '" + joint.name + "' command_interface max",
    std::numeric_limits<double>::infinity());
  return limits;
}

std::optional<ActuatedJointMapping> build_actuated_joint_mapping(
  const hardware_interface::ComponentInfo & joint,
  std::size_t joint_index,
  double min_rad,
  double max_rad)
{
  // `type` selects the servo family, but lucy_config_generator does not emit it:
  // every generated ros2_control xacro predates the field. Absent means the PWM
  // path the hardware had before bus servos existed, read like the other optional
  // params below rather than with at(), which threw on all of them.
  Type type = Type::PWM_SERVO;
  const auto it_type = joint.parameters.find("type");
  if (it_type != joint.parameters.end() && !it_type->second.empty()) {
    if (it_type->second == "pwm_servo") {
      type = Type::PWM_SERVO;
    } else if (it_type->second == "bus_servo") {
      type = Type::BUS_SERVO;
    }
  }


  const auto it_vpin = joint.parameters.find("virtual_pin");
  if (it_vpin == joint.parameters.end() || it_vpin->second.empty()) {
    return std::nullopt;
  }

  const auto it_bus = joint.parameters.find("bus_id");
  if ((it_bus == joint.parameters.end() || it_bus->second.empty()) && type == Type::BUS_SERVO) {
    return std::nullopt;
  }

  ActuatedJointMapping m{};
  m.joint_index = joint_index;
  try {
    m.virtual_pin = std::stoi(it_vpin->second, nullptr, 10);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "invalid virtual_pin '" + it_vpin->second + "' for joint '" + joint.name + "'");
  }
  m.bus_id = 0;
  if (it_bus != joint.parameters.end() && !it_bus->second.empty()) {
    try {
      m.bus_id = std::stoi(it_bus->second, nullptr, 10);
    } catch (const std::exception &) {
      throw std::runtime_error(
              "invalid bus_id '" + it_bus->second + "' for joint '" + joint.name + "'");
    }
  }
  m.type = type;
  m.offset_rad = parse_required_double(joint, "offset_rad");
  m.direction = parse_required_double(joint, "direction");
  m.scale = parse_required_double(joint, "scale");
  m.servo_min_rad = parse_required_double(joint, "servo_min_rad");
  m.servo_max_rad = parse_required_double(joint, "servo_max_rad");
  m.servo_default_rad = parse_required_double(joint, "servo_default_rad");
  m.min_rad = min_rad;
  m.max_rad = max_rad;

  if (m.virtual_pin < 0) {
    throw std::runtime_error(
            "joint '" + joint.name + "' has invalid virtual_pin " +
            std::to_string(m.virtual_pin) + " (must be >= 0)");
  }
  if (m.direction == 0.0 || m.scale == 0.0) {
    throw std::runtime_error(
            "joint '" + joint.name + "' has invalid direction/scale (must be non-zero)");
  }
  if (m.servo_min_rad > m.servo_max_rad) {
    throw std::runtime_error(
            "joint '" + joint.name + "' has servo_min_rad > servo_max_rad");
  }
  if (std::isfinite(m.min_rad) && std::isfinite(m.max_rad) && m.min_rad > m.max_rad) {
    throw std::runtime_error(
            "joint '" + joint.name + "' has command_interface min > max");
  }

  return m;
}

double default_joint_position_rad(const ActuatedJointMapping & m)
{
  return (m.servo_default_rad - m.offset_rad) * m.direction * m.scale;
}

double actuator_command_to_servo_rad(const ActuatedJointMapping & m, double cmd_rad)
{
  const double servo_rad = (cmd_rad / (m.direction * m.scale)) + m.offset_rad;
  return clamp_position_command(servo_rad, m.servo_min_rad, m.servo_max_rad);
}

std::optional<int> sort_and_find_duplicate_virtual_pin(
  std::vector<ActuatedJointMapping> & mappings)
{
  std::sort(
    mappings.begin(),
    mappings.end(),
    [](const ActuatedJointMapping & a, const ActuatedJointMapping & b) {
      return a.virtual_pin < b.virtual_pin;
    });
  for (std::size_t i = 1; i < mappings.size(); ++i) {
    if (mappings[i].virtual_pin == mappings[i - 1].virtual_pin) {
      return mappings[i].virtual_pin;
    }
  }
  return std::nullopt;
}

}  // namespace lucy_ros2_control
