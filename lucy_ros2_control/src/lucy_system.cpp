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

#include "include/lucy_system.hpp"

#include <format>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstddef>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "include/joint_config.hpp"
#include "include/position_limit_clamp.hpp"
#include "rclcpp/qos.hpp"

namespace lucy_ros2_control
{

LucySystemHardware::~LucySystemHardware()
{
}

hardware_interface::CallbackReturn LucySystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    auto it = info_.hardware_parameters.find("node_name");
    if (it != info_.hardware_parameters.end()) {
      node_name_ = it->second;
    } else {
      node_name_ = "lucy";
    }
  }

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(node_name_.c_str()));

  if (info_.joints.size() > MAX_ACTUATORS) {
    RCLCPP_FATAL(
      get_logger(),
      "LucySystemHardware supports a maximum of %d joints, but %zu were provided.",
      MAX_ACTUATORS, info_.joints.size()
    );
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), 0);
  hw_velocities_.resize(info_.joints.size(), 0);
  hw_accelerations_.resize(info_.joints.size(), 0);
  hw_positions_.resize(info_.joints.size(), 0);

  hw_torque_enabled_.resize(info_.joints.size(), 0);

  if (init_joint_limits() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (init_actuator_mappings() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::init_joint_limits()
{
  joint_min_rad_.assign(info_.joints.size(), -std::numeric_limits<double>::infinity());
  joint_max_rad_.assign(info_.joints.size(), std::numeric_limits<double>::infinity());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    try {
      const JointLimits limits = parse_joint_limits(info_.joints[i]);
      joint_min_rad_[i] = limits.min_rad;
      joint_max_rad_[i] = limits.max_rad;
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has invalid command_interface limits: %s",
        info_.joints[i].name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::init_actuator_mappings()
{
  mappings_.clear();
  mappings_.reserve(info_.joints.size());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];

    std::optional<ActuatedJointMapping> mapping;
    try {
      mapping = build_actuated_joint_mapping(joint, i, joint_min_rad_[i], joint_max_rad_[i]);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has invalid parameters: %s", joint.name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!mapping.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has no virtual_pin; treated as passive/unmapped for actuator output.",
        joint.name.c_str());
      continue;
    }

    const ActuatedJointMapping & m = mapping.value();
    if (!std::isfinite(m.command.min) || !std::isfinite(m.command.max)) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has no finite URDF position limits on command interface; "
        "only servo_min/max_deg will bound output.",
        joint.name.c_str());
    }

    mappings_.push_back(m);
    hw_commands_[i] = default_joint_position(m);
    hw_positions_[i] = hw_commands_[i];
  }

  if (const auto duplicate = sort_and_find_duplicate_virtual_pin(mappings_)) {
    RCLCPP_WARN(
      get_logger(),
      "virtual_pin %d is shared by several joints: they overwrite each other's "
      "registers every cycle, and only the last one written reaches the board. "
      "Give each joint its own slot.",
      duplicate.value());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LucySystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LucySystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        "torque_enable", &hw_torque_enabled_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn LucySystemHardware::on_error(
  const rclcpp_lifecycle::State & state)
{
  shared_memory_channel_.reset();
  RCLCPP_ERROR(get_logger(), "LucySystemHardware encountered an error and was cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Initializing shm");
  shared_memory_channel_ = SharedMemoryChannel::create(node_name_);
  if (!shared_memory_channel_.has_value()) {
    RCLCPP_FATAL(get_logger(), "Failed to create shared memory channel");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "SHM initialized");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  shared_memory_channel_.reset();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LucySystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!shared_memory_channel_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Shared memory channel is not available."
    );
    return hardware_interface::return_type::ERROR;
  }

  auto access = SharedMemoryChannel::Access(shared_memory_channel_.value());
  auto state_seq = (*access).state_seq.load(std::memory_order_seq_cst);
  if (state_seq == last_state_seq_) {
    return hardware_interface::return_type::OK;
  }
  if ((state_seq % 2) != 0) {
    return hardware_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < hw_commands_.size(); i++) {
    hw_positions_[i] = (*access).hw_positions[i];
    RCLCPP_DEBUG(
      get_logger(), "Read position for joint %zu: %f", i, hw_positions_[i]
    );
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type lucy_ros2_control::LucySystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!shared_memory_channel_.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Shared memory channel is not available."
    );
    return hardware_interface::return_type::ERROR;
  }

  auto access = SharedMemoryChannel::Access(shared_memory_channel_.value());
  for (const auto & m : mappings_) {
    const std::size_t i = m.joint_index;
    (*access).hw_commands[i] = actuator_command_to_servo_rad(m, hw_commands_[i]);
    (*access).hw_velocities[i] = hw_velocities_[i];
    (*access).hw_accelerations[i] = hw_accelerations_[i];
    (*access).hw_torque_enabled[i] = hw_torque_enabled_[i];
  }

  return hardware_interface::return_type::OK;
}

}  // namespace lucy_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  lucy_ros2_control::LucySystemHardware, hardware_interface::SystemInterface)
