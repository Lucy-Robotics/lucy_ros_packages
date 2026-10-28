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
  double payload[0];

  std::span<double> hw_commands() {return {payload, max_actuators};}
  std::span<double> hw_velocities() {return {payload + max_actuators, max_actuators};}
  std::span<double> hw_accelerations() {return {payload + 2 * max_actuators, max_actuators};}
  std::span<double> hw_torque_enabled() {return {payload + 3 * max_actuators, max_actuators};}
  std::span<double> hw_positions() {return {payload + 4 * max_actuators, max_actuators};}


  std::span<const double> hw_commands() const {return {payload, max_actuators};}
  std::span<const double> hw_velocities() const {return {payload + max_actuators, max_actuators};}
  std::span<const double> hw_accelerations() const
  {
    return {payload + 2 * max_actuators, max_actuators};
  }
  std::span<const double> hw_torque_enabled() const
  {
    return {payload + 3 * max_actuators, max_actuators};
  }
  std::span<const double> hw_positions() const
  {
    return {payload + 4 * max_actuators, max_actuators};
  }

  static constexpr std::size_t size_for(std::size_t max_actuators)
  {
    return sizeof(ActuatorSharedState) + 5 * max_actuators * sizeof(double);
  }
};

static_assert(std::atomic<uint64_t>::is_always_lock_free, "uint64_t must be lock-free");

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

private:
  SharedMemoryChannel();

  void lock();
  void unlock();

  std::string node_name_;
  ActuatorSharedState * actuators_state_;
};

#endif
