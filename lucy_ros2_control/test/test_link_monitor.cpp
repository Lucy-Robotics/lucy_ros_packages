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

/// @file test_link_monitor.cpp
/// @brief Unit tests for firmware liveness detection and torque gating.

#include <chrono>
#include <cstdint>

#include <gtest/gtest.h>

#include "link_monitor.hpp"

namespace
{
using lucy_ros2_control::LinkHealth;
using lucy_ros2_control::LinkMonitor;
using std::chrono::milliseconds;

constexpr std::uint32_t kDown = 0;
constexpr std::uint32_t kUp = 1;

class LinkMonitorTest : public ::testing::Test
{
protected:
  /// Advance the fake time, then run one update with these header values.
  LinkHealth step(milliseconds elapsed, std::uint64_t heartbeat, std::uint32_t link_state)
  {
    now_ += elapsed;
    return monitor_.update(heartbeat, link_state, now_);
  }

  /// One healthy cycle followed by a link loss, with torque requested throughout.
  void lose_link_with_torque_on()
  {
    ASSERT_EQ(step(milliseconds(50), 1, kUp), LinkHealth::Healthy);
    ASSERT_TRUE(monitor_.torque_allowed(true));
    ASSERT_EQ(step(milliseconds(50), 2, kDown), LinkHealth::Disconnected);
    ASSERT_FALSE(monitor_.torque_allowed(true));
  }

  LinkMonitor::Clock::time_point now_{};
  LinkMonitor monitor_{now_};
};
}  // namespace

// ---------------------------------------------------------------------------
// update(): health
// ---------------------------------------------------------------------------

TEST_F(LinkMonitorTest, StartsDisconnected)
{
  EXPECT_EQ(monitor_.health(), LinkHealth::Disconnected);
}

TEST_F(LinkMonitorTest, BeatingFirmwareFollowsLinkState)
{
  EXPECT_EQ(step(milliseconds(50), 1, kDown), LinkHealth::Disconnected);
  EXPECT_EQ(step(milliseconds(50), 2, kUp), LinkHealth::Healthy);
  EXPECT_EQ(step(milliseconds(50), 3, kDown), LinkHealth::Disconnected);
}

TEST_F(LinkMonitorTest, UnknownLinkStateIsNotConnected)
{
  EXPECT_EQ(step(milliseconds(50), 1, 2), LinkHealth::Disconnected);
}

TEST_F(LinkMonitorTest, HeartbeatFrozenOnlyPastTheTimeout)
{
  ASSERT_EQ(step(milliseconds(50), 1, kUp), LinkHealth::Healthy);
  EXPECT_EQ(step(LinkMonitor::kHeartbeatTimeout, 1, kUp), LinkHealth::Healthy);
  EXPECT_EQ(step(milliseconds(1), 1, kUp), LinkHealth::FirmwareFrozen);
}

TEST_F(LinkMonitorTest, FrozenTakesPrecedenceOverLinkState)
{
  // A dead firmware no longer updates link_state, whichever value it left behind.
  ASSERT_EQ(step(milliseconds(50), 1, kDown), LinkHealth::Disconnected);
  EXPECT_EQ(step(milliseconds(300), 1, kDown), LinkHealth::FirmwareFrozen);
  EXPECT_EQ(step(milliseconds(1), 1, kUp), LinkHealth::FirmwareFrozen);
}

TEST_F(LinkMonitorTest, FirmwareThatNeverStartsIsReportedFrozen)
{
  EXPECT_EQ(step(milliseconds(300), 0, kDown), LinkHealth::FirmwareFrozen);
}

TEST_F(LinkMonitorTest, HeartbeatResumingClearsFrozen)
{
  ASSERT_EQ(step(milliseconds(300), 0, kUp), LinkHealth::FirmwareFrozen);
  EXPECT_EQ(step(milliseconds(1), 1, kUp), LinkHealth::Healthy);
}

TEST_F(LinkMonitorTest, AnyHeartbeatChangeCountsAsAlive)
{
  // Not a monotonicity check: on_configure zeroes the counter under a running firmware.
  ASSERT_EQ(step(milliseconds(50), 100, kUp), LinkHealth::Healthy);
  EXPECT_EQ(step(milliseconds(200), 1, kUp), LinkHealth::Healthy);
  EXPECT_EQ(step(milliseconds(200), 1, kUp), LinkHealth::Healthy);
}

// ---------------------------------------------------------------------------
// torque_allowed(): gating and re-arming
// ---------------------------------------------------------------------------

TEST_F(LinkMonitorTest, TorquePassesWhileHealthy)
{
  ASSERT_EQ(step(milliseconds(50), 1, kUp), LinkHealth::Healthy);
  EXPECT_TRUE(monitor_.torque_allowed(true));
  EXPECT_TRUE(monitor_.torque_allowed(false));
  EXPECT_FALSE(monitor_.rearm_required());
}

TEST_F(LinkMonitorTest, TorqueRefusedWhileLinkDown)
{
  lose_link_with_torque_on();
  EXPECT_FALSE(monitor_.torque_allowed(true));
}

TEST_F(LinkMonitorTest, TorqueRefusedWhileFrozen)
{
  ASSERT_EQ(step(milliseconds(50), 1, kUp), LinkHealth::Healthy);
  ASSERT_EQ(step(milliseconds(300), 1, kUp), LinkHealth::FirmwareFrozen);
  EXPECT_FALSE(monitor_.torque_allowed(true));
}

TEST_F(LinkMonitorTest, ReconnectAloneDoesNotRestoreTorque)
{
  lose_link_with_torque_on();
  ASSERT_EQ(step(milliseconds(50), 3, kUp), LinkHealth::Healthy);
  EXPECT_TRUE(monitor_.rearm_required());
  EXPECT_FALSE(monitor_.torque_allowed(true));
  EXPECT_FALSE(monitor_.torque_allowed(true));
}

TEST_F(LinkMonitorTest, TorqueOffThenOnAfterReconnectRestoresTorque)
{
  lose_link_with_torque_on();
  ASSERT_EQ(step(milliseconds(50), 3, kUp), LinkHealth::Healthy);
  EXPECT_TRUE(monitor_.torque_allowed(false));
  EXPECT_FALSE(monitor_.rearm_required());
  EXPECT_TRUE(monitor_.torque_allowed(true));
}

TEST_F(LinkMonitorTest, TorqueToggledDuringTheOutageDoesNotRearm)
{
  lose_link_with_torque_on();
  EXPECT_FALSE(monitor_.torque_allowed(false));
  EXPECT_FALSE(monitor_.torque_allowed(true));

  ASSERT_EQ(step(milliseconds(50), 3, kUp), LinkHealth::Healthy);
  EXPECT_FALSE(monitor_.torque_allowed(true));
}

TEST_F(LinkMonitorTest, TorqueOffThroughoutTheOutageNeedsNoRearm)
{
  ASSERT_EQ(step(milliseconds(50), 1, kUp), LinkHealth::Healthy);
  ASSERT_TRUE(monitor_.torque_allowed(false));
  ASSERT_EQ(step(milliseconds(50), 2, kDown), LinkHealth::Disconnected);
  EXPECT_FALSE(monitor_.torque_allowed(false));
  EXPECT_FALSE(monitor_.rearm_required());

  ASSERT_EQ(step(milliseconds(50), 3, kUp), LinkHealth::Healthy);
  EXPECT_TRUE(monitor_.torque_allowed(true));
}

TEST_F(LinkMonitorTest, TorqueRequestedBeforeTheFirstLinkMustBeRenewed)
{
  EXPECT_FALSE(monitor_.torque_allowed(true));
  ASSERT_EQ(step(milliseconds(50), 1, kUp), LinkHealth::Healthy);
  EXPECT_FALSE(monitor_.torque_allowed(true));
  EXPECT_TRUE(monitor_.torque_allowed(false));
  EXPECT_TRUE(monitor_.torque_allowed(true));
}
