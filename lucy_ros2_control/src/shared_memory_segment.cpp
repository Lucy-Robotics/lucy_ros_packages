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

#include "shared_memory_segment.hpp"

#include <utility>

SharedMemorySegment::SharedMemorySegment(void * data, std::size_t size, void * handle)
: data_(data), size_(size), handle_(handle)
{
}

SharedMemorySegment::~SharedMemorySegment()
{
  unmap();
}

SharedMemorySegment::SharedMemorySegment(SharedMemorySegment && other) noexcept
: data_(std::exchange(other.data_, nullptr)),
  size_(other.size_),
  handle_(std::exchange(other.handle_, nullptr))
{
}

SharedMemorySegment & SharedMemorySegment::operator=(SharedMemorySegment && other) noexcept
{
  if (this != &other) {
    unmap();
    data_ = std::exchange(other.data_, nullptr);
    size_ = other.size_;
    handle_ = std::exchange(other.handle_, nullptr);
  }
  return *this;
}
