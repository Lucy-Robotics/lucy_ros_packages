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

#include "include/shared_memory_segment.hpp"

#include <cstdint>
#include <utility>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#ifdef _WIN32
namespace
{
std::string windows_mapping_name(const std::string & name)
{
  const auto start = name.find_first_not_of('/');
  return "Local\\" + (start == std::string::npos ? std::string() : name.substr(start));
}

void * map_view_or_close(HANDLE mapping, std::size_t size)
{
  if (mapping == nullptr) {
    return nullptr;
  }
  void * addr = MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, size);
  if (addr == nullptr) {
    CloseHandle(mapping);
  }
  return addr;
}
}  // namespace
#endif

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

#ifdef _WIN32

std::optional<SharedMemorySegment> SharedMemorySegment::create(
  const std::string & name, std::size_t size)
{
  const auto wide_size = static_cast<std::uint64_t>(size);
  HANDLE mapping = CreateFileMappingA(
    INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
    static_cast<DWORD>(wide_size >> 32), static_cast<DWORD>(wide_size & 0xFFFFFFFFu),
    windows_mapping_name(name).c_str());
  void * addr = map_view_or_close(mapping, size);
  if (addr == nullptr) {
    return std::nullopt;
  }
  return SharedMemorySegment(addr, size, mapping);
}

std::optional<SharedMemorySegment> SharedMemorySegment::open(
  const std::string & name, std::size_t size)
{
  HANDLE mapping = OpenFileMappingA(
    FILE_MAP_ALL_ACCESS, FALSE, windows_mapping_name(name).c_str());
  void * addr = map_view_or_close(mapping, size);
  if (addr == nullptr) {
    return std::nullopt;
  }
  return SharedMemorySegment(addr, size, mapping);
}

void SharedMemorySegment::remove(const std::string &)
{
}

void SharedMemorySegment::unmap()
{
  if (data_ != nullptr) {
    UnmapViewOfFile(data_);
    data_ = nullptr;
  }
  if (handle_ != nullptr) {
    CloseHandle(handle_);
    handle_ = nullptr;
  }
}

#else

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

#endif
