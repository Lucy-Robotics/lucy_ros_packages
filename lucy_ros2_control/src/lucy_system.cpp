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

#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif

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
namespace
{
constexpr const char * kRegTableSuffix = ".lucy_reg_table";
constexpr const char * kRegHeaderSuffix = ".lucy_reg_header";

#ifndef _WIN32
/// Node name the POSIX objects are named after: node_name_ sanitised and capped.
///
/// shm_open() and sem_open() take a name, not a path: a leading '/' and no other
/// slash. Darwin caps the whole name at PSHMNAMLEN (31) and fails with
/// ENAMETOOLONG past it, and the generated node names
/// ("lucy_hardware_interface_left_arm") blow through that once a suffix is
/// appended. An over-long name keeps its tail: Lucy's components share the
/// "lucy_hardware_interface" prefix and differ only in the suffix.
std::string shm_node_name_for(const std::string & node_name)
{
  constexpr std::size_t kMaxShmName = 31;
  // Leading '/' plus the longest of the two suffixes.
  const std::size_t budget = kMaxShmName - 1 - std::strlen(kRegHeaderSuffix);

  std::string sanitised;
  sanitised.reserve(node_name.size());
  for (const char c : node_name) {
    const bool keep = std::isalnum(static_cast<unsigned char>(c)) != 0 ||
      c == '_' || c == '-' || c == '.';
    sanitised.push_back(keep ? c : '_');
  }
  if (sanitised.size() > budget) {
    sanitised.erase(0, sanitised.size() - budget);
  }
  return sanitised;
}

/// Create, size and map one shared-memory object of `size` bytes.
///
/// O_EXCL so an existing object is a fact to act on rather than one silently
/// adopted: it can only be a leak from a run that died before
/// release_registers(), and Darwin rejects ftruncate() on an object that
/// already has a size (EINVAL).
///
/// Returns nullptr on failure, having unlinked anything it created, so the
/// caller records the name only for an object that is now its own to release.
void * create_shm(
  const rclcpp::Logger & logger, const std::string & name, std::size_t size, const char * what)
{
  int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  if (fd == -1 && errno == EEXIST) {
    RCLCPP_WARN(logger, "Reclaiming '%s' left behind by an earlier run.", name.c_str());
    shm_unlink(name.c_str());
    fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  }
  if (fd == -1) {
    RCLCPP_FATAL(
      logger, "Failed to create '%s' for %s (shm_open(): %s).",
      name.c_str(), what, std::strerror(errno));
    return nullptr;
  }

  if (ftruncate(fd, static_cast<off_t>(size)) == -1) {
    RCLCPP_FATAL(
      logger, "Failed to size '%s' for %s (ftruncate(): %s).",
      name.c_str(), what, std::strerror(errno));
    close(fd);
    shm_unlink(name.c_str());
    return nullptr;
  }

  void * addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);  // fd not needed after mmap

  if (addr == MAP_FAILED) {
    RCLCPP_FATAL(
      logger, "Failed to map '%s' for %s (mmap(): %s).",
      name.c_str(), what, std::strerror(errno));
    shm_unlink(name.c_str());
    return nullptr;
  }
  return addr;
}
#endif  // !_WIN32
}  // namespace

LucySystemHardware::~LucySystemHardware()
{
  release_registers();
}

void LucySystemHardware::release_registers()
{
#ifndef _WIN32
  if (shared_registers_ != nullptr) {
    munmap(shared_registers_, sizeof(SharedRegisters));
    shared_registers_ = nullptr;
  }
  if (register_header_ != nullptr) {
    munmap(register_header_, sizeof(RegisterHeader));
    register_header_ = nullptr;
  }
  if (sem_ != nullptr && sem_ != SEM_FAILED) {
    sem_close(sem_);
  }
  sem_ = nullptr;

  // Drop the names as well: these objects outlive the process that created
  // them, and on_deactivate does not run when the node is killed outright.
  // Only names this component created are unlinked, never a peer's.
  if (!reg_table_name_.empty()) {
    shm_unlink(reg_table_name_.c_str());
    reg_table_name_.clear();
  }
  if (!reg_header_name_.empty()) {
    shm_unlink(reg_header_name_.c_str());
    reg_header_name_.clear();
  }
  if (!sem_name_.empty()) {
    sem_unlink(sem_name_.c_str());
    sem_name_.clear();
  }
#else
  shared_registers_ = nullptr;
  register_header_ = nullptr;
  sem_ = nullptr;
  reg_table_name_.clear();
  reg_header_name_.clear();
  sem_name_.clear();
#endif
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

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger((info_.name).c_str()));

  {
    auto it = info_.hardware_parameters.find("node_name");
    if (it != info_.hardware_parameters.end()) {
      node_name_ = it->second;
    } else {
      node_name_ = "lucy";
    }


  }

  // resizing command and state vectors
  hw_positions_.resize(info_.joints.size(), 0);
  // hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN()); // no velocities for our servos
  hw_commands_.resize(info_.joints.size(), 0);
  hw_old_commands_.resize(info_.joints.size(), 0);

  if (validate_joints() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (configure_publisher() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (init_joint_limits() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (init_actuator_mappings() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (init_registers() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::validate_joints()
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    const std::string error = validate_joint_interfaces(joint);
    if (!error.empty()) {
      RCLCPP_FATAL(get_logger(), "%s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::configure_publisher()
{
  auto it_publish = info_.hardware_parameters.find("publish_actuators");
  if (it_publish != info_.hardware_parameters.end()) {
    const std::string & v = it_publish->second;
    publish_actuators_ = !(v == "false" || v == "0" || v == "False");
  }

  std::string publisher_topic;
  auto it_topic = info_.hardware_parameters.find("publisher_topic");
  if (publish_actuators_) {
    if (it_topic == info_.hardware_parameters.end() || it_topic->second.empty()) {
      RCLCPP_FATAL(get_logger(), "Hardware parameter 'publisher_topic' is missing or empty.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    publisher_topic = it_topic->second;
  }

  node_ = std::make_shared<rclcpp::Node>(node_name_);

  // RELIABLE to match micro-ROS rclc_subscription_init_default (RELIABLE). A BEST_EFFORT publisher
  // does not match a RELIABLE subscription in ROS 2, so the Pico would receive no commands.
  if (publish_actuators_) {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(publisher_topic, qos);
    RCLCPP_INFO(
      get_logger(),
      "Publishing joint state on topic '%s' (RELIABLE for micro-ROS default subscriber)",
      publisher_topic.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "publish_actuators=false: URDF limits enforced in-process only (no actuator topics).");
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
    if (!std::isfinite(m.min_rad) || !std::isfinite(m.max_rad)) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has no finite URDF position limits on command interface; "
        "only servo_min/max_deg will bound output.",
        joint.name.c_str());
    }

    mappings_.push_back(m);
    hw_commands_[i] = default_joint_position_rad(m);
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

hardware_interface::CallbackReturn LucySystemHardware::init_registers()
{
#ifndef _WIN32
  release_registers();

  shm_node_name_ = shm_node_name_for(node_name_);
  if (shm_node_name_ != node_name_) {
    RCLCPP_WARN(
      get_logger(),
      "node_name '%s' does not fit a POSIX shm name; shared memory uses '%s'.",
      node_name_.c_str(), shm_node_name_.c_str());
  }
  const std::string reg_table_name = std::format("/{}{}", shm_node_name_, kRegTableSuffix);
  const std::string reg_header_name = std::format("/{}{}", shm_node_name_, kRegHeaderSuffix);
  const std::string sem_name = std::format("/{}", shm_node_name_);

  void * table = create_shm(
    get_logger(), reg_table_name, sizeof(SharedRegisters), "register table");
  if (table == nullptr) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Named from here on, so every later failure unlinks it back out.
  shared_registers_ = static_cast<SharedRegisters *>(table);
  reg_table_name_ = reg_table_name;

  void * header = create_shm(
    get_logger(), reg_header_name, sizeof(RegisterHeader), "register header");
  if (header == nullptr) {
    release_registers();
    return hardware_interface::CallbackReturn::ERROR;
  }
  register_header_ = static_cast<RegisterHeader *>(header);
  reg_header_name_ = reg_header_name;

  // Unlink before creating: a semaphore outlives its creator, and one left at 0
  // by a run that died between sem_wait() and sem_post() would be adopted as-is
  // and deadlock the first write().
  sem_unlink(sem_name.c_str());
  sem_ = sem_open(sem_name.c_str(), O_CREAT | O_EXCL, 0644, 1);
  if (sem_ == SEM_FAILED) {
    RCLCPP_FATAL(
      get_logger(), "Failed to create named sem '%s' (sem_open(): %s).",
      sem_name.c_str(), std::strerror(errno));
    release_registers();
    return hardware_interface::CallbackReturn::ERROR;
  }
  sem_name_ = sem_name;

  RCLCPP_INFO(
    get_logger(),
    "Registers mapped on '%s' and '%s', lock '%s'. Attach the firmware bridge "
    "with node name '%s'.",
    reg_table_name.c_str(), reg_header_name.c_str(), sem_name.c_str(),
    shm_node_name_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
#else
  (void)kRegTableSuffix;
  (void)kRegHeaderSuffix;
  RCLCPP_ERROR(
    get_logger(),
    "POSIX shared-memory register transport is not supported on Windows. "
    "Use mock_hardware or gazebo; real-hardware SHM requires Linux/macOS "
    "(Boost.Interprocess port tracked separately).");
  return hardware_interface::CallbackReturn::ERROR;
#endif
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
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

/*
This function should be used to initialize/activate the hardware.
In our case, the hardware is already ready to receive informations
*/
void LucySystemHardware::write_torque_opcode(
  const ActuatedJointMapping & m, uint16_t opcode)
{
#ifndef _WIN32
  const int reg = bus_block_base(m.virtual_pin);
  shared_registers_->register_table[reg + kBusServoIdOffset] =
    static_cast<uint16_t>(m.bus_id);
  register_header_->set_dirty(reg + kBusServoIdOffset);
  shared_registers_->register_table[reg + kBusServoCmdOffset] = opcode;
  register_header_->set_dirty(reg + kBusServoCmdOffset);
#else
  (void)m;
  (void)opcode;
#endif
}

void LucySystemHardware::apply_torque_state(bool enable)
{
#ifndef _WIN32
  const uint16_t opcode = enable ? kBusServoCmdEnableTorque : kBusServoCmdDisableTorque;
  for (const auto & m : mappings_) {
    if (m.type != Type::BUS_SERVO) {
      continue;
    }
    sem_wait(sem_);
    write_torque_opcode(m, opcode);
    sem_post(sem_);
    RCLCPP_INFO(
      get_logger(), "%s torque on bus servo id %d.",
      enable ? "Enabling" : "Disabling", m.bus_id);
  }
#endif
  torque_enabled_ = enable;
}

void LucySystemHardware::start_active_client_watch()
{
  if (node_ == nullptr || client_spin_thread_.joinable()) {
    return;
  }
  // Transient-local to match the latched publisher: the current controller is
  // delivered on subscribe, not only on the next change.
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable().transient_local();
  active_client_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kActiveClientTopic, qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      controlled_.store(!msg->data.empty(), std::memory_order_relaxed);
    });

  // A subscription needs an executor, and it must not run on the
  // controller-manager thread.
  client_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  client_executor_->add_node(node_);
  client_spin_thread_ = std::thread([this]() {client_executor_->spin();});
}

void LucySystemHardware::stop_active_client_watch()
{
  if (client_executor_ != nullptr) {
    client_executor_->cancel();
  }
  if (client_spin_thread_.joinable()) {
    client_spin_thread_.join();
  }
  if (client_executor_ != nullptr && node_ != nullptr) {
    client_executor_->remove_node(node_);
  }
  client_executor_.reset();
  active_client_sub_.reset();
}

hardware_interface::CallbackReturn LucySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  start_active_client_watch();

  apply_torque_state(false);

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LucySystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (shared_registers_ != nullptr && sem_ != nullptr) {
    apply_torque_state(false);
  }
  stop_active_client_watch();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  release_registers();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LucySystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Updating the position of each joint from the command
  for (std::size_t i = 0; i < hw_commands_.size(); i++) {
    // No encoder for our servos, we assume that the position is always reached
    hw_positions_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

namespace
{
uint16_t to_register_milliradians(double cmd_rad)
{
  double wire_rad = cmd_rad;
  if (wire_rad < 0.0) {
    wire_rad += 2.0 * M_PI;
  }
  const double milli = std::round(wire_rad * 1000.0);
  if (!std::isfinite(milli) || milli <= 0.0) {
    return 0;
  }
  constexpr double kMax = static_cast<double>(std::numeric_limits<uint16_t>::max());
  return static_cast<uint16_t>(milli > kMax ? kMax : milli);
}
}  // namespace

hardware_interface::return_type lucy_ros2_control::LucySystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    const double cmd_rad = lucy_ros2_control::clamp_position_command(
      hw_commands_[i], joint_min_rad_[i], joint_max_rad_[i]);
    hw_commands_[i] = cmd_rad;
    hw_positions_[i] = cmd_rad;
  }

  // Applied here, not in the subscription callback, to keep the register block
  // single-writer.
  const bool controlled = controlled_.load(std::memory_order_relaxed);
  if (controlled != torque_enabled_) {
    apply_torque_state(controlled);
    if (controlled) {
      // NaN fails the unchanged-command guard below, forcing every target to
      // be re-sent.
      for (std::size_t i = 0; i < hw_old_commands_.size(); ++i) {
        hw_old_commands_[i] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
  if (!torque_enabled_) {
    return hardware_interface::return_type::OK;
  }

  for (const auto & m : mappings_) {
    const std::size_t i = m.joint_index;
    const double cmd_rad = hw_commands_[i];
    if (std::abs(cmd_rad - hw_old_commands_[i]) <= 0.001) {
      continue;
    }
    hw_old_commands_[i] = cmd_rad;

    // Joint space -> servo space: applies offset_rad / direction / scale and
    // clamps to [servo_min_rad, servo_max_rad]. Sending the raw joint angle
    // skips the mechanical envelope and wraps negative commands to ~2*pi.
    const uint16_t wire = to_register_milliradians(actuator_command_to_servo_rad(m, cmd_rad));
    const int reg =
      m.type == Type::BUS_SERVO ? bus_block_base(m.virtual_pin) : pwm_block_base(m.virtual_pin);

#ifndef _WIN32
    sem_wait(sem_);
    switch (m.type) {
      case Type::PWM_SERVO:
        // Angle first, opcode last: same ordering rule as the bus block below.
        shared_registers_->register_table[reg + 1] = wire;
        register_header_->set_dirty(reg + 1);
        shared_registers_->register_table[reg] = 1;
        register_header_->set_dirty(reg);
        break;
      case Type::BUS_SERVO:
        // Operands first, opcode last: the bridge ships dirty registers in
        // ascending index order and the firmware clears cmd in the tick that
        // consumes it, so a cmd sent first fires on the previous id/angle.
        shared_registers_->register_table[reg + kBusServoIdOffset] =
          static_cast<uint16_t>(m.bus_id);
        register_header_->set_dirty(reg + kBusServoIdOffset);
        shared_registers_->register_table[reg + kBusServoAngleOffset] = wire;
        register_header_->set_dirty(reg + kBusServoAngleOffset);
        shared_registers_->register_table[reg + kBusServoCmdOffset] = kBusServoCmdMove;
        register_header_->set_dirty(reg + kBusServoCmdOffset);
        break;
    }
    sem_post(sem_);
#else
    (void)wire;
    (void)reg;
#endif
  }

  if (!publish_actuators_ || !joint_publisher_) {
    return hardware_interface::return_type::OK;
  }

  // Firmware reads inputs->position.data[joint->config.virtual_pin] per configured joint.

  if (mappings_.empty()) {
    return hardware_interface::return_type::OK;
  }

  for (const auto & m : mappings_) {

    //msg.position[static_cast<size_t>(m.virtual_pin)] =
    actuator_command_to_servo_rad(m, hw_commands_[m.joint_index]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace lucy_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  lucy_ros2_control::LucySystemHardware, hardware_interface::SystemInterface)
