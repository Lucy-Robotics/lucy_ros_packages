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

#include "include/lucy_system.hpp"

#include <algorithm>
#include <chrono>
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

namespace lucy_ros2_control
{

namespace
{
constexpr std::chrono::seconds kLinkWarningPeriod{1};
}  // namespace

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
  {
    auto it = info_.hardware_parameters.find("publish_actuators");
    loopback_encoders_ = it != info_.hardware_parameters.end() && it->second == "false";
  }

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(node_name_.c_str()));

  hw_commands_.resize(info_.joints.size(), 0);
  hw_velocities_.resize(info_.joints.size(), 0);
  hw_accelerations_.resize(info_.joints.size(), 0);
  hw_positions_.resize(info_.joints.size(), 0);
  hw_temperatures_.resize(info_.joints.size(), 0);
  hw_efforts_.resize(info_.joints.size(), 0);
  hw_currents_.resize(info_.joints.size(), 0);
  hw_loads_.resize(info_.joints.size(), 0);
  // torque_enable is claimed by a forward_command_controller. Stay off until it writes.
  hw_torque_enabled_.resize(info_.joints.size(), 0.0);

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
        "only limit_min/limit_max will bound output.",
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
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "temperature", &hw_temperatures_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "current", &hw_currents_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "load", &hw_loads_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
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

void LucySystemHardware::drop_torque_and_release_shm()
{
  // Last SHM write before unmap. The forward controller may already be stopped;
  // the host firmware must still see torque off.
  if (shared_memory_channel_.has_value()) {
    auto access = SharedMemoryChannel::Access(shared_memory_channel_.value());
    for (const auto & m : mappings_) {
      (*access).hw_torque_enabled()[m.joint_index] = 0.0f;
    }
  }
  std::fill(hw_torque_enabled_.begin(), hw_torque_enabled_.end(), 0.0);
  shared_memory_channel_.reset();
}

hardware_interface::CallbackReturn LucySystemHardware::on_error(
  const rclcpp_lifecycle::State & /*state*/)
{
  drop_torque_and_release_shm();
  RCLCPP_ERROR(get_logger(), "LucySystemHardware encountered an error and was cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::string shm_name = lucy_shm_name(node_name_);
  RCLCPP_INFO(get_logger(), "Initializing shm %s", shm_name.c_str());
  shared_memory_channel_ = SharedMemoryChannel::create(shm_name, info_.joints.size());
  if (!shared_memory_channel_.has_value()) {
    RCLCPP_FATAL(get_logger(), "Failed to create shared memory channel");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "SHM initialized");

  // create() zeroed the header: the firmware gets a full timeout to start beating.
  link_monitor_ = LinkMonitor(LinkMonitor::Clock::now());
  last_link_warning_ = {};

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
  drop_torque_and_release_shm();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void LucySystemHardware::update_link_health()
{
  // Mock hardware has no firmware behind the segment.
  if (loopback_encoders_) {
    return;
  }

  const auto now = LinkMonitor::Clock::now();
  const LinkHealth previous = link_monitor_.health();
  const LinkHealth health = link_monitor_.update(
    shared_memory_channel_->heartbeat(), shared_memory_channel_->link_state(), now);

  if (health == LinkHealth::Healthy) {
    if (previous != LinkHealth::Healthy) {
      RCLCPP_INFO(get_logger(), "Firmware link is up.");
      if (link_monitor_.rearm_required()) {
        RCLCPP_WARN(
          get_logger(),
          "Torque stays off until torque_enable goes to 0 then back to 1 "
          "(Control Robot OFF, then ON).");
      }
    }
    return;
  }

  // Not RCLCPP_WARN_THROTTLE: its state is static per call site, so the hardware
  // components of a multi-board robot would mask each other's warnings.
  if (health != previous || now - last_link_warning_ >= kLinkWarningPeriod) {
    last_link_warning_ = now;
    RCLCPP_WARN(
      get_logger(), "%s Positions are stale and torque is held off.",
      health == LinkHealth::FirmwareFrozen ?
      "No firmware heartbeat: the firmware is hung, stopped or not started yet." :
      "The firmware reports its hardware link down.");
  }
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

  // A lost link is not an ERROR: on_error would release the shm, and the firmware
  // could then never reconnect without a restart.
  update_link_health();

  // Passive/unmapped joints have no feedback: echo their last command.
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_positions_[i] = hw_commands_[i];
  }

  auto access = SharedMemoryChannel::Access(shared_memory_channel_.value());
  for (const auto & m : mappings_) {
    const std::size_t i = m.joint_index;
    if (loopback_encoders_) {
      (*access).hw_positions()[i] =
        static_cast<float>(actuator_command_to_servo_rad(m, hw_commands_[i]));
    }
    const double servo_rad = (*access).hw_positions()[i];
    hw_positions_[i] = servo_rad_to_joint_position(
      m, std::isfinite(servo_rad) ? servo_rad : actuator_command_to_servo_rad(m, hw_commands_[i]));
    hw_temperatures_[i] = (*access).hw_temperatures()[i];
    hw_efforts_[i] = (*access).hw_efforts()[i];
    hw_currents_[i] = (*access).hw_currents()[i];
    hw_loads_[i] = (*access).hw_loads()[i];
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

  const bool torque_requested = std::any_of(
    mappings_.begin(), mappings_.end(),
    [this](const ActuatedJointMapping & m) {return hw_torque_enabled_[m.joint_index] >= 0.5;});
  const bool torque_allowed = loopback_encoders_ || link_monitor_.torque_allowed(torque_requested);

  auto access = SharedMemoryChannel::Access(shared_memory_channel_.value());
  for (const auto & m : mappings_) {
    const std::size_t i = m.joint_index;
    (*access).hw_commands()[i] =
      static_cast<float>(actuator_command_to_servo_rad(m, hw_commands_[i]));
    (*access).hw_velocities()[i] = static_cast<float>(hw_velocities_[i]);
    (*access).hw_accelerations()[i] = static_cast<float>(hw_accelerations_[i]);
    (*access).hw_torque_enabled()[i] =
      torque_allowed ? static_cast<float>(hw_torque_enabled_[i]) : 0.0f;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace lucy_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  lucy_ros2_control::LucySystemHardware, hardware_interface::SystemInterface)
