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

#ifndef LUCY_ROS2_CONTROL__SHARED_MEMORY_SEGMENT_HPP_
#define LUCY_ROS2_CONTROL__SHARED_MEMORY_SEGMENT_HPP_

#include <cstddef>
#include <optional>
#include <string>

class SharedMemorySegment
{
public:
  static std::optional<SharedMemorySegment> create(const std::string & name, std::size_t size);
  static std::optional<SharedMemorySegment> open(const std::string & name, std::size_t size);
  static void remove(const std::string & name);

  ~SharedMemorySegment();
  SharedMemorySegment(const SharedMemorySegment &) = delete;
  SharedMemorySegment & operator=(const SharedMemorySegment &) = delete;
  SharedMemorySegment(SharedMemorySegment && other) noexcept;
  SharedMemorySegment & operator=(SharedMemorySegment && other) noexcept;

  void * data() const {return data_;}

private:
  SharedMemorySegment(void * data, std::size_t size, void * handle);
  void unmap();

  void * data_ = nullptr;
  std::size_t size_ = 0;
  void * handle_ = nullptr;
};

#endif  // LUCY_ROS2_CONTROL__SHARED_MEMORY_SEGMENT_HPP_
