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

#include <cstdint>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

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
