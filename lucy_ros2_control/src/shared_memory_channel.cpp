// Copyright 2025 Lucy Robotics Team
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

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <string_view>
#include <thread>
#include <utility>

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

unsigned ros_domain_id()
{
  const char * env = std::getenv("ROS_DOMAIN_ID");
  if (env == nullptr) {
    return 0;
  }
  const std::string_view text(env);
  unsigned domain = 0;
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), domain);
  return (text.empty() || error != std::errc() || end != text.data() + text.size()) ? 0 : domain;
}
}  // namespace

std::string lucy_shm_name(const std::string & node_name)
{
  const auto start = node_name.find_first_not_of('/');
  const std::string node = start == std::string::npos ? "" : node_name.substr(start);
  return "/" + node + ".domain" + std::to_string(ros_domain_id());
}

SharedMemoryChannel::~SharedMemoryChannel()
{
  release();
}

std::optional<SharedMemoryChannel> SharedMemoryChannel::create(
  const std::string & node_name, std::size_t max_actuators)
{
  const std::string shm_name = posix_shm_name(node_name);
  const std::size_t shm_size = ActuatorSharedState::size_for(max_actuators);
  auto segment = SharedMemorySegment::create(shm_name, shm_size);
  if (!segment) {
    return std::nullopt;
  }

  SharedMemoryChannel channel;
  channel.node_name_ = shm_name;
  channel.actuators_state_ = new (segment->data()) ActuatorSharedState();
  channel.segment_ = std::move(segment);
  channel.actuators_state_->max_actuators = static_cast<uint32_t>(max_actuators);
  std::fill_n(
    channel.actuators_state_->payload, ActuatorSharedState::kArrays * max_actuators, 0.0f);
  std::ranges::fill(channel.actuators_state_->hw_positions(), std::nanf(""));
  return channel;
}

SharedMemoryChannel::SharedMemoryChannel(SharedMemoryChannel && other) noexcept
: node_name_(std::move(other.node_name_)),
  actuators_state_(other.actuators_state_),
  segment_(std::move(other.segment_))
{
  other.actuators_state_ = nullptr;
}


SharedMemoryChannel & SharedMemoryChannel::operator=(SharedMemoryChannel && other) noexcept
{
  node_name_ = std::move(other.node_name_);
  actuators_state_ = other.actuators_state_;
  segment_ = std::move(other.segment_);
  other.actuators_state_ = nullptr;
  return *this;
}

void SharedMemoryChannel::release()
{
  segment_.reset();
  actuators_state_ = nullptr;
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
