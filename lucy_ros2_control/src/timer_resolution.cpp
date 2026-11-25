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

#include "include/timer_resolution.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <timeapi.h>

OneMillisecondTimerResolution::OneMillisecondTimerResolution()
: granted_(timeBeginPeriod(1) == TIMERR_NOERROR)
{
}

OneMillisecondTimerResolution::~OneMillisecondTimerResolution()
{
  if (granted_) {
    timeEndPeriod(1);
  }
}

#else

OneMillisecondTimerResolution::OneMillisecondTimerResolution() = default;
OneMillisecondTimerResolution::~OneMillisecondTimerResolution() = default;

#endif
