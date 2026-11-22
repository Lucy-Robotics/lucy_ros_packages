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

/// @file link_monitor.hpp
/// @brief Firmware liveness tracking and torque gating for LucySystemHardware.
///
/// Pure logic (no ROS, no shm): the caller feeds it the ``heartbeat`` and
/// ``link_state`` header fields written by the firmwares, with the current time.

#ifndef LUCY_ROS2_CONTROL__LINK_MONITOR_HPP_
#define LUCY_ROS2_CONTROL__LINK_MONITOR_HPP_

#include <chrono>
#include <cstdint>

namespace lucy_ros2_control
{

enum class LinkHealth
{
  Healthy,
  /// The firmware runs but reports its hardware link down (``link_state != 1``).
  Disconnected,
  /// ``heartbeat`` stopped changing: firmware hung, killed or not started yet.
  /// Takes precedence over ``link_state``, which a dead firmware no longer updates.
  FirmwareFrozen,
};

class LinkMonitor
{
public:
  using Clock = std::chrono::steady_clock;

  /// The firmwares beat every 50 ms: about 5 missed ticks.
  static constexpr std::chrono::milliseconds kHeartbeatTimeout{250};

  /// The shm header starts zeroed, so this is also the state before the first update().
  explicit LinkMonitor(Clock::time_point now)
  : last_beat_time_(now)
  {
  }

  LinkHealth update(std::uint64_t heartbeat, std::uint32_t link_state, Clock::time_point now)
  {
    if (heartbeat != last_heartbeat_) {
      last_heartbeat_ = heartbeat;
      last_beat_time_ = now;
    }

    if (now - last_beat_time_ > kHeartbeatTimeout) {
      health_ = LinkHealth::FirmwareFrozen;
    } else if (link_state != kLinkConnected) {
      health_ = LinkHealth::Disconnected;
    } else {
      health_ = LinkHealth::Healthy;
    }
    return health_;
  }

  LinkHealth health() const {return health_;}

  /// Whether torque may reach the firmware this cycle, given what ROS requests.
  ///
  /// Torque is off while the link is down. A request seen during that time stays
  /// refused after the link returns, until ROS has requested torque off on a healthy
  /// link: the controllers keep their last command through the outage, and the
  /// firmware would otherwise apply it at once (at maximum speed if velocity is 0),
  /// from wherever the joints ended up.
  bool torque_allowed(bool torque_requested)
  {
    if (health_ != LinkHealth::Healthy) {
      rearm_required_ = rearm_required_ || torque_requested;
      return false;
    }
    if (!torque_requested) {
      rearm_required_ = false;
    }
    return !rearm_required_;
  }

  bool rearm_required() const {return rearm_required_;}

private:
  static constexpr std::uint32_t kLinkConnected = 1;

  std::uint64_t last_heartbeat_{0};
  Clock::time_point last_beat_time_;
  LinkHealth health_{LinkHealth::Disconnected};
  bool rearm_required_{false};
};

}  // namespace lucy_ros2_control

#endif  // LUCY_ROS2_CONTROL__LINK_MONITOR_HPP_
