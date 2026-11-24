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

#ifndef LUCY_ROS2_CONTROL__TEST__PROCESS_ENV_HPP_
#define LUCY_ROS2_CONTROL__TEST__PROCESS_ENV_HPP_

#include <cstdlib>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

inline int process_id()
{
#ifdef _WIN32
  return _getpid();
#else
  return getpid();
#endif
}

inline void set_env_var(const char * name, const char * value)
{
#ifdef _WIN32
  _putenv_s(name, value);
#else
  setenv(name, value, 1);
#endif
}

inline void unset_env_var(const char * name)
{
#ifdef _WIN32
  _putenv_s(name, "");
#else
  unsetenv(name);
#endif
}

#endif  // LUCY_ROS2_CONTROL__TEST__PROCESS_ENV_HPP_
