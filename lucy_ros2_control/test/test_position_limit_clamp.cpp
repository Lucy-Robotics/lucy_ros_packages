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

/// @file test_position_limit_clamp.cpp
/// @brief Unit tests for the URDF position-limit clamp in LucySystemHardware.
///
/// Real and mock hardware clamp incoming position commands to the ros2_control
/// ``[min, max]`` envelope. Gazebo uses stock ``gz_ros2_control`` without this
/// clamp; this helper documents the algorithm used on the Lucy plugin path.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "position_limit_clamp.hpp"

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

double clamp(double cmd, double min, double max)
{
  return lucy_ros2_control::clamp_position_command(cmd, min, max);
}
}  // namespace

TEST(PositionLimitClamp, BelowMinClampsToMin)
{
  EXPECT_DOUBLE_EQ(clamp(-1.0, 0.0, 1.5708), 0.0);
}

TEST(PositionLimitClamp, AboveMaxClampsToMax)
{
  EXPECT_DOUBLE_EQ(clamp(4.71239, 0.0, 1.5708), 1.5708);
}

TEST(PositionLimitClamp, WithinRangeIsUnchanged)
{
  EXPECT_DOUBLE_EQ(clamp(0.5, 0.0, 1.5708), 0.5);
}

TEST(PositionLimitClamp, OnExactBoundsIsUnchanged)
{
  EXPECT_DOUBLE_EQ(clamp(0.0, 0.0, 1.5708), 0.0);
  EXPECT_DOUBLE_EQ(clamp(1.5708, 0.0, 1.5708), 1.5708);
}

TEST(PositionLimitClamp, DegeneratePointRangePinsToValue)
{
  EXPECT_DOUBLE_EQ(clamp(-2.0, 0.5, 0.5), 0.5);
  EXPECT_DOUBLE_EQ(clamp(2.0, 0.5, 0.5), 0.5);
}

TEST(PositionLimitClamp, NonFiniteMinDisablesLowerBound)
{
  EXPECT_DOUBLE_EQ(clamp(-1.0e9, -kInf, 1.5708), -1.0e9);
  EXPECT_DOUBLE_EQ(clamp(2.0, -kInf, 1.5708), 1.5708);
  EXPECT_DOUBLE_EQ(clamp(0.5, kNan, 1.5708), 0.5);
}

TEST(PositionLimitClamp, NonFiniteMaxDisablesUpperBound)
{
  EXPECT_DOUBLE_EQ(clamp(1.0e9, 0.0, kInf), 1.0e9);
  EXPECT_DOUBLE_EQ(clamp(-2.0, 0.0, kInf), 0.0);
  EXPECT_DOUBLE_EQ(clamp(0.5, 0.0, kNan), 0.5);
}

TEST(PositionLimitClamp, BothBoundsNonFiniteIsNoOp)
{
  EXPECT_DOUBLE_EQ(clamp(1.0e9, -kInf, kInf), 1.0e9);
  EXPECT_DOUBLE_EQ(clamp(-1.0e9, -kInf, kInf), -1.0e9);
  EXPECT_DOUBLE_EQ(clamp(0.0, kNan, kNan), 0.0);
}

/// Representative grid pins the clamp algorithm used by LucySystemHardware.
TEST(PositionLimitClamp, ClampAlgorithmOnRepresentativeGrid)
{
  // (lower, upper) drawn from a real ``left_shoulder_y_link_joint`` envelope.
  const double lo = 0.0;
  const double hi = 1.5708;
  const double grid[] = {-3.14159, -0.0001, 0.0, 0.5, 1.0, 1.5707, 1.5708, 1.5709, 4.71239};
  for (double cmd : grid) {
    const double expected = std::min(std::max(cmd, lo), hi);
    EXPECT_DOUBLE_EQ(clamp(cmd, lo, hi), expected) << "cmd=" << cmd;
  }
}
