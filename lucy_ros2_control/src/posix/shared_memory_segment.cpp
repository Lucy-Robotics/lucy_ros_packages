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

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

std::optional<SharedMemorySegment> SharedMemorySegment::create(
  const std::string & name, std::size_t size)
{
  int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
  if (fd == -1) {
    shm_unlink(name.c_str());
    return std::nullopt;
  }

  struct stat st {};
  if (fstat(fd, &st) == -1 ||
    (static_cast<std::size_t>(st.st_size) < size && ftruncate(fd, size) == -1))
  {
    close(fd);
    shm_unlink(name.c_str());
    return std::nullopt;
  }

  void * addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (addr == MAP_FAILED) {
    shm_unlink(name.c_str());
    return std::nullopt;
  }
  return SharedMemorySegment(addr, size, nullptr);
}

std::optional<SharedMemorySegment> SharedMemorySegment::open(
  const std::string & name, std::size_t size)
{
  const int fd = shm_open(name.c_str(), O_RDWR, 0);
  if (fd == -1) {
    return std::nullopt;
  }
  void * addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (addr == MAP_FAILED) {
    return std::nullopt;
  }
  return SharedMemorySegment(addr, size, nullptr);
}

void SharedMemorySegment::remove(const std::string & name)
{
  shm_unlink(name.c_str());
}

void SharedMemorySegment::unmap()
{
  if (data_ != nullptr) {
    munmap(data_, size_);
    data_ = nullptr;
  }
}
