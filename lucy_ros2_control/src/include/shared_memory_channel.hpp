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
#include "constant.hpp"

struct alignas(64) ActuatorSharedState
{
  alignas(64)
  std::atomic<uint64_t> command_seq{0};
  double hw_commands[MAX_ACTUATORS]{};
  double hw_velocities[MAX_ACTUATORS]{};
  double hw_accelerations[MAX_ACTUATORS]{};

  double hw_torque_enabled[MAX_ACTUATORS]{};

  alignas(64)
  std::atomic<uint64_t> state_seq{0};
  double hw_positions[MAX_ACTUATORS]{};

  alignas(64)
  std::atomic<uint64_t> heartbeat{0};
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

  static std::optional<SharedMemoryChannel> create(const std::string & node_name);
  void release();

private:
  SharedMemoryChannel();

  void lock();
  void unlock();

  std::string node_name_;
  ActuatorSharedState * actuators_state_;
};

#endif
