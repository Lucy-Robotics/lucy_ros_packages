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

/// @file test_lucy_system_state.cpp
/// @brief Unit tests for the state interfaces exported by LucySystemHardware.
///
/// Drives the plugin directly (on_init -> on_configure -> read) without a
/// controller_manager, and plays the firmware by writing into the POSIX shm
/// segment the plugin creates. Covers the per-joint telemetry copied from shm
/// (temperature, effort, current and load) and the reaction to a lost firmware
/// link (logs, read() result, torque written to shm).

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <unordered_map>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcutils/logging.h"

#include "link_monitor.hpp"
#include "lucy_system.hpp"
#include "shared_memory_channel.hpp"

namespace
{
using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::ComponentInfo;
using hardware_interface::InterfaceInfo;
using hardware_interface::StateInterface;

const std::vector<std::string> kExpectedStateInterfaces = {
  hardware_interface::HW_IF_POSITION, "temperature", "current", "load", "voltage",
  hardware_interface::HW_IF_EFFORT};

InterfaceInfo interface(const std::string & name)
{
  InterfaceInfo info;
  info.name = name;
  return info;
}

/// Actuated joint with identity calibration (servo value == joint value).
ComponentInfo actuated_joint(const std::string & name, int virtual_pin)
{
  ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
  joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
  joint.parameters = {
    {"virtual_pin", std::to_string(virtual_pin)},
    {"offset", "0"},
    {"direction", "1"},
    {"scale", "1"},
    {"limit_min", "-180"},
    {"limit_max", "180"},
    {"default", "0"},
  };
  return joint;
}

/// Joint without ``virtual_pin``: exported but never fed from shm.
ComponentInfo passive_joint(const std::string & name)
{
  ComponentInfo joint = actuated_joint(name, 0);
  joint.parameters.erase("virtual_pin");
  return joint;
}

/// Second mapping of the plugin's segment, standing in for the firmware.
class FirmwareMapping
{
public:
  FirmwareMapping(const std::string & name, std::size_t max_actuators)
  : size_(ActuatorSharedState::size_for(max_actuators))
  {
    const int fd = shm_open(name.c_str(), O_RDWR, 0);
    if (fd == -1) {
      return;
    }
    void * addr = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (addr != MAP_FAILED) {
      state_ = static_cast<ActuatorSharedState *>(addr);
    }
  }
  ~FirmwareMapping()
  {
    if (state_ != nullptr) {
      munmap(state_, size_);
    }
  }
  FirmwareMapping(const FirmwareMapping &) = delete;
  FirmwareMapping & operator=(const FirmwareMapping &) = delete;

  bool ok() const {return state_ != nullptr;}
  ActuatorSharedState & operator*() {return *state_;}

  /// One firmware tick: publish the link state and bump the heartbeat.
  void tick(std::uint32_t link_state)
  {
    state_->link_state.store(link_state);
    state_->heartbeat.fetch_add(1);
  }

private:
  std::size_t size_;
  ActuatorSharedState * state_{nullptr};
};

/// Records what the plugin logs while in scope.
class LogCapture
{
public:
  LogCapture()
  : previous_(rcutils_logging_get_output_handler())
  {
    records().clear();
    rcutils_logging_set_output_handler(&LogCapture::handle);
  }
  ~LogCapture()
  {
    rcutils_logging_set_output_handler(previous_);
  }
  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;

  /// Number of records at ``severity`` whose text contains ``text``.
  long count(int severity, const std::string & text) const
  {
    return std::count_if(
      records().begin(), records().end(),
      [&](const auto & r) {
        return r.first == severity && r.second.find(text) != std::string::npos;
      });
  }

private:
  static std::vector<std::pair<int, std::string>> & records()
  {
    static std::vector<std::pair<int, std::string>> instance;
    return instance;
  }

  static void handle(
    const rcutils_log_location_t *, int severity, const char *,
    rcutils_time_point_value_t, const char * format, va_list * args)
  {
    char buffer[512];
    std::vsnprintf(buffer, sizeof(buffer), format, *args);
    records().emplace_back(severity, buffer);
  }

  rcutils_logging_output_handler_t previous_;
};

/// Long enough for the monitor to call the heartbeat frozen.
void wait_for_heartbeat_timeout()
{
  std::this_thread::sleep_for(
    lucy_ros2_control::LinkMonitor::kHeartbeatTimeout + std::chrono::milliseconds(50));
}

class LucySystemStateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    node_name_ = "lucy_test_sys_" + std::to_string(getpid()) + "_" + info->name();
    shm_name_ = lucy_shm_name(node_name_);
    shm_unlink(shm_name_.c_str());
  }

  void TearDown() override
  {
    if (configured_) {
      system_.on_deactivate(rclcpp_lifecycle::State());
    }
    shm_unlink(shm_name_.c_str());
  }

  /// on_init + export + on_configure. Returns false on any failure.
  bool bring_up(
    std::vector<ComponentInfo> joints,
    std::unordered_map<std::string, std::string> extra_params = {})
  {
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info.name = "LucyTestSystem";
    params.hardware_info.type = "system";
    params.hardware_info.hardware_parameters = std::move(extra_params);
    params.hardware_info.hardware_parameters["node_name"] = node_name_;
    params.hardware_info.joints = std::move(joints);
    joint_count_ = params.hardware_info.joints.size();

    if (system_.on_init(params) != CallbackReturn::SUCCESS) {
      return false;
    }
    states_ = system_.export_state_interfaces();
    commands_ = system_.export_command_interfaces();
    if (system_.on_configure(rclcpp_lifecycle::State()) != CallbackReturn::SUCCESS) {
      return false;
    }
    configured_ = true;
    return true;
  }

  const StateInterface * find(const std::string & joint, const std::string & iface) const
  {
    for (const auto & s : states_) {
      if (s.get_prefix_name() == joint && s.get_interface_name() == iface) {
        return &s;
      }
    }
    return nullptr;
  }

  /// Value of ``joint/iface``; NaN when the interface is missing or unreadable.
  double value(const std::string & joint, const std::string & iface) const
  {
    const StateInterface * s = find(joint, iface);
    if (s == nullptr) {
      ADD_FAILURE() << "missing state interface " << joint << "/" << iface;
      return std::numeric_limits<double>::quiet_NaN();
    }
    const std::optional<double> v = s->get_optional<double>();
    if (!v.has_value()) {
      ADD_FAILURE() << "unreadable state interface " << joint << "/" << iface;
      return std::numeric_limits<double>::quiet_NaN();
    }
    return v.value();
  }

  hardware_interface::return_type do_read()
  {
    return system_.read(rclcpp::Time(0, 0), rclcpp::Duration(0, 0));
  }

  hardware_interface::return_type do_write()
  {
    return system_.write(rclcpp::Time(0, 0), rclcpp::Duration(0, 0));
  }

  /// Plays the torque_enable forward controller.
  void command_torque(const std::string & joint, double value)
  {
    for (auto & c : commands_) {
      if (c.get_prefix_name() == joint && c.get_interface_name() == "torque_enable") {
        ASSERT_TRUE(c.set_value(value));
        return;
      }
    }
    FAIL() << "missing command interface " << joint << "/torque_enable";
  }

  lucy_ros2_control::LucySystemHardware system_;
  std::vector<StateInterface> states_;
  std::vector<CommandInterface> commands_;
  std::string node_name_;
  std::string shm_name_;
  std::size_t joint_count_{0};
  bool configured_{false};
};
}  // namespace

// ---------------------------------------------------------------------------
// export_state_interfaces
// ---------------------------------------------------------------------------

TEST_F(LucySystemStateTest, ExportsFiveStateInterfacesPerJoint)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0), passive_joint("j1")}));
  ASSERT_EQ(states_.size(), kExpectedStateInterfaces.size() * joint_count_);

  for (const std::string joint : {"j0", "j1"}) {
    for (const auto & iface : kExpectedStateInterfaces) {
      EXPECT_NE(find(joint, iface), nullptr) << joint << "/" << iface;
    }
  }
}

TEST_F(LucySystemStateTest, CurrentAndLoadStartAtZero)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  EXPECT_EQ(value("j0", "current"), 0.0);
  EXPECT_EQ(value("j0", "load"), 0.0);
}

TEST_F(LucySystemStateTest, EachJointExportsItsOwnCurrentAndLoadStorage)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0), actuated_joint("j1", 1)}));
  // The interfaces must not alias each other (nor another telemetry array).
  std::vector<std::string> names;
  for (const std::string joint : {"j0", "j1"}) {
    for (const auto & iface : kExpectedStateInterfaces) {
      names.push_back(joint + "/" + iface);
    }
  }
  for (const auto & s : states_) {
    EXPECT_EQ(std::count(names.begin(), names.end(), s.get_name()), 1) << s.get_name();
  }
}

// ---------------------------------------------------------------------------
// read(): shm -> state interfaces
// ---------------------------------------------------------------------------

TEST_F(LucySystemStateTest, ReadCopiesCurrentAndLoadFromShm)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0), actuated_joint("j1", 1)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  (*fw).hw_currents()[0] = 0.42f;
  (*fw).hw_currents()[1] = -1.3f;
  (*fw).hw_loads()[0] = 0.25f;
  (*fw).hw_loads()[1] = -0.75f;

  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_FLOAT_EQ(value("j0", "current"), 0.42f);
  EXPECT_FLOAT_EQ(value("j1", "current"), -1.3f);
  EXPECT_FLOAT_EQ(value("j0", "load"), 0.25f);
  EXPECT_FLOAT_EQ(value("j1", "load"), -0.75f);
}

TEST_F(LucySystemStateTest, ReadKeepsTelemetryArraysSeparate)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  (*fw).hw_positions()[0] = 12.0f;
  (*fw).hw_temperatures()[0] = 41.5f;
  (*fw).hw_efforts()[0] = 0.9f;
  (*fw).hw_currents()[0] = 0.33f;
  (*fw).hw_loads()[0] = 0.66f;

  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_FLOAT_EQ(value("j0", hardware_interface::HW_IF_POSITION), 12.0f);
  EXPECT_FLOAT_EQ(value("j0", "temperature"), 41.5f);
  EXPECT_FLOAT_EQ(value("j0", hardware_interface::HW_IF_EFFORT), 0.9f);
  EXPECT_FLOAT_EQ(value("j0", "current"), 0.33f);
  EXPECT_FLOAT_EQ(value("j0", "load"), 0.66f);
}

TEST_F(LucySystemStateTest, ReadIndexesShmByJointIndexNotVirtualPin)
{
  // Pins in reverse order: mappings_ get sorted by pin, but shm is indexed by joint.
  ASSERT_TRUE(bring_up({actuated_joint("j0", 5), actuated_joint("j1", 2)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  (*fw).hw_currents()[0] = 1.0f;
  (*fw).hw_currents()[1] = 2.0f;
  (*fw).hw_loads()[0] = 0.1f;
  (*fw).hw_loads()[1] = 0.2f;

  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_FLOAT_EQ(value("j0", "current"), 1.0f);
  EXPECT_FLOAT_EQ(value("j1", "current"), 2.0f);
  EXPECT_FLOAT_EQ(value("j0", "load"), 0.1f);
  EXPECT_FLOAT_EQ(value("j1", "load"), 0.2f);
}

TEST_F(LucySystemStateTest, ReadTracksSuccessiveShmUpdates)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  (*fw).hw_currents()[0] = 0.5f;
  (*fw).hw_loads()[0] = 0.5f;
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);

  (*fw).hw_currents()[0] = 0.0f;
  (*fw).hw_loads()[0] = -0.2f;
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_FLOAT_EQ(value("j0", "current"), 0.0f);
  EXPECT_FLOAT_EQ(value("j0", "load"), -0.2f);
}

TEST_F(LucySystemStateTest, PassiveJointIgnoresShmCurrentAndLoad)
{
  ASSERT_TRUE(bring_up({passive_joint("j0"), actuated_joint("j1", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  // Slot 0 belongs to the passive joint: nothing must be read from it.
  (*fw).hw_currents()[0] = 9.0f;
  (*fw).hw_loads()[0] = 9.0f;
  (*fw).hw_currents()[1] = 0.8f;
  (*fw).hw_loads()[1] = 0.4f;

  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_EQ(value("j0", "current"), 0.0);
  EXPECT_EQ(value("j0", "load"), 0.0);
  EXPECT_FLOAT_EQ(value("j1", "current"), 0.8f);
  EXPECT_FLOAT_EQ(value("j1", "load"), 0.4f);
}

TEST_F(LucySystemStateTest, LoopbackModeStillReportsCurrentAndLoad)
{
  // publish_actuators=false loops commands back into positions, but the
  // electrical telemetry still comes from shm.
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}, {{"publish_actuators", "false"}}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  (*fw).hw_currents()[0] = 0.12f;
  (*fw).hw_loads()[0] = 0.34f;

  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_FLOAT_EQ(value("j0", "current"), 0.12f);
  EXPECT_FLOAT_EQ(value("j0", "load"), 0.34f);
}

// ---------------------------------------------------------------------------
// read()/write(): firmware link
// ---------------------------------------------------------------------------

TEST_F(LucySystemStateTest, LinkUpIsLoggedOnceOnTransition)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());
  LogCapture logs;

  fw.tick(0);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_INFO, "link is up"), 0);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "hardware link down"), 1);

  fw.tick(1);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_INFO, "link is up"), 1);

  fw.tick(1);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_INFO, "link is up"), 1);
}

TEST_F(LucySystemStateTest, LinkWarningIsThrottledAcrossCycles)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());
  LogCapture logs;

  for (int cycle = 0; cycle < 5; ++cycle) {
    fw.tick(0);
    ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "hardware link down"), 1);
}

TEST_F(LucySystemStateTest, FrozenHeartbeatWarnsWithoutFailingRead)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  fw.tick(1);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);

  LogCapture logs;
  wait_for_heartbeat_timeout();
  EXPECT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "No firmware heartbeat"), 1);

  // The shm stayed mapped: the firmware comes back on the same segment.
  (*fw).hw_currents()[0] = 0.7f;
  fw.tick(1);
  EXPECT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_INFO, "link is up"), 1);
  EXPECT_FLOAT_EQ(value("j0", "current"), 0.7f);
}

TEST_F(LucySystemStateTest, TorqueReachesShmOnHealthyLink)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  fw.tick(1);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  command_torque("j0", 1.0);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  EXPECT_EQ((*fw).hw_torque_enabled()[0], 1.0f);
}

TEST_F(LucySystemStateTest, LinkLossForcesTorqueOffUntilRearmed)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  fw.tick(1);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  command_torque("j0", 1.0);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  ASSERT_EQ((*fw).hw_torque_enabled()[0], 1.0f);

  // Outage: the controller keeps requesting torque.
  fw.tick(0);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  EXPECT_EQ(do_write(), hardware_interface::return_type::OK);
  EXPECT_EQ((*fw).hw_torque_enabled()[0], 0.0f);

  // Back up: still off, and the operator is told why.
  LogCapture logs;
  fw.tick(1);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  EXPECT_EQ((*fw).hw_torque_enabled()[0], 0.0f);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "Torque stays off"), 1);

  // Control Robot OFF, then ON.
  command_torque("j0", 0.0);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  EXPECT_EQ((*fw).hw_torque_enabled()[0], 0.0f);
  command_torque("j0", 1.0);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  EXPECT_EQ((*fw).hw_torque_enabled()[0], 1.0f);
}

TEST_F(LucySystemStateTest, FrozenHeartbeatForcesTorqueOff)
{
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());

  fw.tick(1);
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  command_torque("j0", 1.0);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  ASSERT_EQ((*fw).hw_torque_enabled()[0], 1.0f);

  wait_for_heartbeat_timeout();
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  EXPECT_EQ((*fw).hw_torque_enabled()[0], 0.0f);
}

TEST_F(LucySystemStateTest, LoopbackModeIgnoresTheFirmwareLink)
{
  // Mock hardware has no firmware: no warning, and torque is passed through.
  ASSERT_TRUE(bring_up({actuated_joint("j0", 0)}, {{"publish_actuators", "false"}}));
  FirmwareMapping fw(shm_name_, joint_count_);
  ASSERT_TRUE(fw.ok());
  LogCapture logs;

  wait_for_heartbeat_timeout();
  ASSERT_EQ(do_read(), hardware_interface::return_type::OK);
  command_torque("j0", 1.0);
  ASSERT_EQ(do_write(), hardware_interface::return_type::OK);
  EXPECT_EQ((*fw).hw_torque_enabled()[0], 1.0f);
  EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "firmware"), 0);
}
