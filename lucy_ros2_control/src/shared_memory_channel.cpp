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

#include "include/shared_memory_channel.hpp"

#include <sys/mman.h>
#include <algorithm>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

namespace
{
/// POSIX shm names must start with ``/``. Xacro may pass a bare node name (``so_arm``).
std::string posix_shm_name(const std::string & node_name)
{
  if (node_name.empty() || node_name.front() == '/') {
    return node_name;
  }
  return "/" + node_name;
}
}  // namespace

SharedMemoryChannel::~SharedMemoryChannel()
{
  release();
}

std::optional<SharedMemoryChannel> SharedMemoryChannel::create(
  const std::string & node_name, std::size_t max_actuators)
{
  const std::string shm_name = posix_shm_name(node_name);
  const std::size_t shm_size = ActuatorSharedState::size_for(max_actuators);
  int fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
  if (fd == -1) {
    shm_unlink(shm_name.c_str());
    return std::nullopt;
  }

  if (ftruncate(fd, shm_size) == -1) {
    close(fd);
    shm_unlink(shm_name.c_str());
    return std::nullopt;
  }

  void * addr = mmap(
    nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
    0);
  close(fd);
  if (addr == MAP_FAILED) {
    shm_unlink(shm_name.c_str());
    return std::nullopt;
  }

  SharedMemoryChannel channel;
  channel.node_name_ = shm_name;
  channel.actuators_state_ = new (addr) ActuatorSharedState();
  channel.actuators_state_->max_actuators = static_cast<uint32_t>(max_actuators);
  std::fill_n(channel.actuators_state_->payload, 5 * max_actuators, 0.0);
  return channel;
}

SharedMemoryChannel::SharedMemoryChannel(SharedMemoryChannel && other) noexcept
: node_name_(std::move(other.node_name_)),
  actuators_state_(other.actuators_state_)
{
  other.actuators_state_ = nullptr;
}


SharedMemoryChannel & SharedMemoryChannel::operator=(SharedMemoryChannel && other) noexcept
{
  node_name_ = std::move(other.node_name_);
  actuators_state_ = other.actuators_state_;
  other.actuators_state_ = nullptr;
  return *this;
}

void SharedMemoryChannel::release()
{
  if (actuators_state_ != nullptr) {
    munmap(actuators_state_, ActuatorSharedState::size_for(actuators_state_->max_actuators));
    actuators_state_ = nullptr;
  }
}

void SharedMemoryChannel::lock()
{
  actuators_state_->command_seq.fetch_add(1, std::memory_order_acq_rel);
}

void SharedMemoryChannel::unlock()
{
  actuators_state_->command_seq.fetch_add(1, std::memory_order_acq_rel);
}

SharedMemoryChannel::SharedMemoryChannel()
{

}
