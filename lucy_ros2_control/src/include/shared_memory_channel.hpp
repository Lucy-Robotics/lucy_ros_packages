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

#ifndef LUCY_ROS2_CONTROL__SHARED_MEMORY_CHANNEL_HPP_
#define LUCY_ROS2_CONTROL__SHARED_MEMORY_CHANNEL_HPP_

#include <atomic>
#include <optional>
#include <string>
#include <cstdint>
#include <mutex>
#include <span>

struct ActuatorSharedState
{
  alignas(64) std::atomic<uint64_t> command_seq{0};
  alignas(64) std::atomic<uint64_t> state_seq{0};
  alignas(64) std::atomic<uint64_t> heartbeat{0};
  uint32_t max_actuators{0};
  std::atomic<uint32_t> link_state{0};
  float payload[0];

  static constexpr std::size_t kArrays = 10;

  std::span<float> hw_commands() {return {payload, max_actuators};}
  std::span<float> hw_velocities() {return {payload + max_actuators, max_actuators};}
  std::span<float> hw_accelerations() {return {payload + 2 * max_actuators, max_actuators};}
  std::span<float> hw_torque_enabled() {return {payload + 3 * max_actuators, max_actuators};}
  std::span<float> hw_positions() {return {payload + 4 * max_actuators, max_actuators};}
  std::span<float> hw_temperatures() {return {payload + 5 * max_actuators, max_actuators};}
  std::span<float> hw_efforts() {return {payload + 6 * max_actuators, max_actuators};}
  std::span<float> hw_currents() {return {payload + 7 * max_actuators, max_actuators};}
  std::span<float> hw_loads() {return {payload + 8 * max_actuators, max_actuators};}
  std::span<float> hw_voltages() {return {payload + 9 * max_actuators, max_actuators};}


  std::span<const float> hw_commands() const {return {payload, max_actuators};}
  std::span<const float> hw_velocities() const {return {payload + max_actuators, max_actuators};}
  std::span<const float> hw_accelerations() const
  {
    return {payload + 2 * max_actuators, max_actuators};
  }
  std::span<const float> hw_torque_enabled() const
  {
    return {payload + 3 * max_actuators, max_actuators};
  }
  std::span<const float> hw_positions() const
  {
    return {payload + 4 * max_actuators, max_actuators};
  }
  std::span<const float> hw_temperatures() const
  {
    return {payload + 5 * max_actuators, max_actuators};
  }
  std::span<const float> hw_efforts() const
  {
    return {payload + 6 * max_actuators, max_actuators};
  }
  std::span<const float> hw_currents() const
  {
    return {payload + 7 * max_actuators, max_actuators};
  }
  std::span<const float> hw_loads() const
  {
    return {payload + 8 * max_actuators, max_actuators};
  }
  std::span<const float> hw_voltages() const
  {
    return {payload + 9 * max_actuators, max_actuators};
  }

  static constexpr std::size_t size_for(std::size_t max_actuators)
  {
    return sizeof(ActuatorSharedState) + kArrays * max_actuators * sizeof(float);
  }
};

static_assert(std::atomic<uint64_t>::is_always_lock_free, "uint64_t must be lock-free");
static_assert(std::atomic<uint32_t>::is_always_lock_free, "uint32_t must be lock-free");

/// POSIX shm name of a ros2_control node: ``/<node_name>.domain<ROS_DOMAIN_ID>``.
///
/// The domain keeps robots on separate ROS domains of one host on separate segments.
/// ROS_DOMAIN_ID unset or not a plain number counts as 0. The embed-rs firmwares build
/// the same name (lucy_embedded_firmware crates/host/src/shm.rs): keep both in step.
std::string lucy_shm_name(const std::string & node_name);

class SharedMemoryChannel
{
public:
  class Access
  {
public:
    Access(SharedMemoryChannel & channel)
    : channel_(channel)
    {
      channel_.lock();
    }
    ~Access()
    {
      channel_.unlock();
    }

    Access(const Access &) = delete;
    Access & operator=(const Access &) = delete;

    ActuatorSharedState & operator*() & {return *channel_.actuators_state_;}
    ActuatorSharedState & operator*() && = delete;

private:
    SharedMemoryChannel & channel_;
  };


  ~SharedMemoryChannel();
  SharedMemoryChannel(const SharedMemoryChannel &) = delete;
  SharedMemoryChannel & operator=(const SharedMemoryChannel &) = delete;

  SharedMemoryChannel(SharedMemoryChannel &&) noexcept;
  SharedMemoryChannel & operator=(SharedMemoryChannel &&) noexcept;

  static std::optional<SharedMemoryChannel> create(
    const std::string & node_name,
    std::size_t max_actuators);
  void release();

  /// Independent atomics written by the firmware: no ``Access`` (command seqlock) needed.
  uint64_t heartbeat() const {return actuators_state_->heartbeat.load(std::memory_order_acquire);}
  uint32_t link_state() const
  {
    return actuators_state_->link_state.load(std::memory_order_acquire);
  }

private:
  SharedMemoryChannel();

  void lock();
  void unlock();

  std::string node_name_;
  ActuatorSharedState * actuators_state_;
};

#endif
