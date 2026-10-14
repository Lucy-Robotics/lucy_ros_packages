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
#include <thread>

#include <fcntl.h>
#include <unistd.h>

SharedMemoryChannel::~SharedMemoryChannel()
{
  release();
}

std::optional<SharedMemoryChannel> SharedMemoryChannel::create(const std::string & node_name)
{
  int fd = shm_open(node_name.c_str(), O_CREAT | O_RDWR, 0666);
  if (fd == -1) {
    shm_unlink(node_name.c_str());
    return std::nullopt;
  }

  if (ftruncate(fd, sizeof(ActuatorSharedState)) == -1) {
    close(fd);
    shm_unlink(node_name.c_str());
    return std::nullopt;
  }

  void * addr = mmap(
    nullptr, sizeof(ActuatorSharedState), PROT_READ | PROT_WRITE, MAP_SHARED, fd,
    0);
  close(fd);
  if (addr == MAP_FAILED) {
    shm_unlink(node_name.c_str());
    return std::nullopt;
  }

  SharedMemoryChannel channel;
  channel.node_name_ = node_name;
  channel.actuators_state_ = new (addr) ActuatorSharedState();
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
    munmap(actuators_state_, sizeof(ActuatorSharedState));
    actuators_state_ = nullptr;
  }
  shm_unlink(node_name_.c_str());
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
