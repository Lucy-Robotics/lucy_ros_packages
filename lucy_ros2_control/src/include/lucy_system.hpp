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

/// @file lucy_system.hpp
/// @brief ros2_control ``SystemInterface`` plugin for Lucy-compatible robots.
///
/// Maps URDF joint-space commands to servo-space output and clamps them to the
/// ros2_control command_interface ``<param name="min/max">`` envelope (real +
/// mock paths) via ``position_limit_clamp.hpp``. Gazebo uses stock
/// ``gz_ros2_control`` and does not load this plugin.

#ifndef LUCY_ROS2_CONTROL__LUCY_SYSTEM_HPP_
#define LUCY_ROS2_CONTROL__LUCY_SYSTEM_HPP_

#include <memory>
#include <cstddef>
#include <cstdint>
#include <atomic>
#include <string>
#include <thread>
#include <vector>
#include <array>

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "joint_config.hpp"
#include "shared_memory_channel.hpp"

namespace lucy_ros2_control
{

class LucySystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LucySystemHardware)

  ~LucySystemHardware();


  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const override {return *logger_;}

private:
  hardware_interface::CallbackReturn validate_joints();
  hardware_interface::CallbackReturn init_joint_limits();
  hardware_interface::CallbackReturn init_actuator_mappings();

  rclcpp::Node::SharedPtr node_;

  std::string node_name_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  // rclcpp::Clock::SharedPtr clock_;

  std::optional<SharedMemoryChannel> shared_memory_channel_;

  // Store the command for the simulated robot
  std::vector<double> hw_torque_enabled_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_accelerations_;

  uint64_t last_state_seq_{0};
  std::vector<double> hw_positions_;

  /** Per-joint URDF limits from command_interface min/max (rad); ±inf when unset. */
  std::vector<double> joint_min_rad_;
  std::vector<double> joint_max_rad_;

  std::vector<ActuatedJointMapping> mappings_;

  std::atomic<bool> controlled_{false};
};

}  // namespace lucy_ros2_control

#endif  // LUCY_ROS2_CONTROL__LUCY_SYSTEM_HPP_
