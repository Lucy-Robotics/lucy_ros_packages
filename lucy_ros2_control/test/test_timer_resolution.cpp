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

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "timer_resolution.hpp"

namespace
{
std::chrono::duration<double, std::milli> average_one_millisecond_sleep()
{
  constexpr int kSleeps = 50;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kSleeps; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return (std::chrono::steady_clock::now() - start) / kSleeps;
}
}  // namespace

TEST(OneMillisecondTimerResolution, IsGrantedWhileHeld)
{
  OneMillisecondTimerResolution resolution;
  EXPECT_TRUE(resolution.granted());
}

TEST(OneMillisecondTimerResolution, OneMillisecondSleepsWakeWithinFiveMillisecondsWhileHeld)
{
  OneMillisecondTimerResolution resolution;
  EXPECT_LT(average_one_millisecond_sleep().count(), 5.0);
}
