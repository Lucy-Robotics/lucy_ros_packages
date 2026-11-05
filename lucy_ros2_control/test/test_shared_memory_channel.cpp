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

/// @file test_shared_memory_channel.cpp
/// @brief Unit tests for the ActuatorSharedState layout and SharedMemoryChannel.
///
/// The layout is the ABI shared with embed-rs (``crates/core/src/data.rs``):
/// a 144-byte header followed by ``kArrays`` arrays of ``max_actuators`` floats,
/// in the order commands, velocities, accelerations, torque_enabled, positions,
/// temperatures, efforts, currents, loads. These tests pin that contract so a
/// change on one side fails here before it corrupts data at runtime.

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "shared_memory_channel.hpp"

namespace
{
/// Mirrors ``SHM_PAYLOAD_OFFSET`` / ``SHM_ARRAYS`` in embed-rs ``crates/core/src/data.rs``.
constexpr std::size_t kRustPayloadOffset = 144;
constexpr std::size_t kRustArrays = 9;
/// ``size_of::<f32>()``: stride of the payload arrays.
constexpr std::size_t kRustElementSize = 4;

/// Array slot indices, in ABI order.
enum Slot : std::size_t
{
  kCommands = 0,
  kVelocities,
  kAccelerations,
  kTorqueEnabled,
  kPositions,
  kTemperatures,
  kEfforts,
  kCurrents,
  kLoads,
};

/// Raw byte offset of ``payload[slot * n + i]`` from the start of the segment.
std::size_t raw_offset(std::size_t slot, std::size_t n, std::size_t i)
{
  return kRustPayloadOffset + (slot * n + i) * kRustElementSize;
}

/// Second mapping of an existing segment, standing in for the firmware process.
class ExternalMapping
{
public:
  ExternalMapping(const std::string & name, std::size_t size)
  : size_(size)
  {
    const int fd = shm_open(name.c_str(), O_RDWR, 0);
    if (fd == -1) {
      return;
    }
    void * addr = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (addr != MAP_FAILED) {
      base_ = static_cast<std::uint8_t *>(addr);
    }
  }
  ~ExternalMapping()
  {
    if (base_ != nullptr) {
      munmap(base_, size_);
    }
  }
  ExternalMapping(const ExternalMapping &) = delete;
  ExternalMapping & operator=(const ExternalMapping &) = delete;

  bool ok() const {return base_ != nullptr;}

  float read(std::size_t offset) const
  {
    float v;
    std::memcpy(&v, base_ + offset, sizeof(v));
    return v;
  }
  void write(std::size_t offset, float v) {std::memcpy(base_ + offset, &v, sizeof(v));}
  std::uint32_t max_actuators() const
  {
    std::uint32_t v;
    std::memcpy(&v, base_ + offsetof(ActuatorSharedState, max_actuators), sizeof(v));
    return v;
  }

private:
  std::size_t size_;
  std::uint8_t * base_{nullptr};
};

class SharedMemoryChannelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    name_ = "/lucy_test_shm_" + std::to_string(getpid()) + "_" + info->name();
    shm_unlink(name_.c_str());
  }
  void TearDown() override {shm_unlink(name_.c_str());}

  std::string name_;
};
}  // namespace

// ---------------------------------------------------------------------------
// Static layout (ABI with embed-rs)
// ---------------------------------------------------------------------------

TEST(ActuatorSharedStateLayout, ArrayCountMatchesFirmware)
{
  EXPECT_EQ(ActuatorSharedState::kArrays, kRustArrays);
}

TEST(ActuatorSharedStateLayout, PayloadElementSizeMatchesFirmware)
{
  EXPECT_EQ(sizeof(ActuatorSharedState::payload[0]), kRustElementSize);
}

TEST(ActuatorSharedStateLayout, HeaderOffsetsMatchFirmware)
{
  EXPECT_EQ(offsetof(ActuatorSharedState, command_seq), 0u);
  EXPECT_EQ(offsetof(ActuatorSharedState, state_seq), 64u);
  EXPECT_EQ(offsetof(ActuatorSharedState, heartbeat), 128u);
  EXPECT_EQ(offsetof(ActuatorSharedState, max_actuators), 136u);
  EXPECT_EQ(offsetof(ActuatorSharedState, payload), kRustPayloadOffset);
}

TEST(ActuatorSharedStateLayout, SizeForCoversAllArrays)
{
  // Rust: (SHM_PAYLOAD_OFFSET + 63) / 64 * 64 + SHM_ARRAYS * n * 4
  for (std::size_t n : {0u, 1u, 6u, 32u}) {
    const std::size_t rust_size =
      (kRustPayloadOffset + 63) / 64 * 64 + kRustArrays * n * kRustElementSize;
    EXPECT_EQ(ActuatorSharedState::size_for(n), rust_size) << "n=" << n;
    // The last array must end inside the segment.
    EXPECT_LE(raw_offset(kLoads, n, n), ActuatorSharedState::size_for(n)) << "n=" << n;
  }
}

// ---------------------------------------------------------------------------
// Accessor spans
// ---------------------------------------------------------------------------

class ActuatorSharedStateSpans : public ::testing::Test
{
protected:
  static constexpr std::size_t kN = 4;

  void SetUp() override
  {
    state_ = new (buffer_) ActuatorSharedState();
    state_->max_actuators = kN;
  }

  alignas(64) std::uint8_t buffer_[ActuatorSharedState::size_for(kN)] {};
  ActuatorSharedState * state_{nullptr};
};

TEST_F(ActuatorSharedStateSpans, CurrentsAndLoadsSitAtSlots7And8)
{
  EXPECT_EQ(state_->hw_currents().data(), state_->payload + kCurrents * kN);
  EXPECT_EQ(state_->hw_loads().data(), state_->payload + kLoads * kN);
  EXPECT_EQ(state_->hw_currents().size(), kN);
  EXPECT_EQ(state_->hw_loads().size(), kN);
}

TEST_F(ActuatorSharedStateSpans, ConstAccessorsMatchMutable)
{
  const ActuatorSharedState & cstate = *state_;
  EXPECT_EQ(cstate.hw_currents().data(), state_->hw_currents().data());
  EXPECT_EQ(cstate.hw_loads().data(), state_->hw_loads().data());
  EXPECT_EQ(cstate.hw_currents().size(), kN);
  EXPECT_EQ(cstate.hw_loads().size(), kN);
}

TEST_F(ActuatorSharedStateSpans, ArraysAreContiguousAndDisjoint)
{
  const float * expected[] = {
    state_->hw_commands().data(),
    state_->hw_velocities().data(),
    state_->hw_accelerations().data(),
    state_->hw_torque_enabled().data(),
    state_->hw_positions().data(),
    state_->hw_temperatures().data(),
    state_->hw_efforts().data(),
    state_->hw_currents().data(),
    state_->hw_loads().data(),
  };
  static_assert(std::size(expected) == ActuatorSharedState::kArrays);
  for (std::size_t slot = 0; slot < ActuatorSharedState::kArrays; ++slot) {
    EXPECT_EQ(expected[slot], state_->payload + slot * kN) << "slot=" << slot;
  }
}

TEST_F(ActuatorSharedStateSpans, WritingCurrentsAndLoadsLeavesOtherArraysUntouched)
{
  for (std::size_t i = 0; i < kN; ++i) {
    state_->hw_currents()[i] = 1.0f + static_cast<float>(i);
    state_->hw_loads()[i] = -1.0f - static_cast<float>(i);
  }
  for (std::size_t slot = 0; slot < kCurrents; ++slot) {
    for (std::size_t i = 0; i < kN; ++i) {
      EXPECT_EQ(state_->payload[slot * kN + i], 0.0) << "slot=" << slot << " i=" << i;
    }
  }
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_EQ(state_->payload[kCurrents * kN + i], 1.0f + static_cast<float>(i));
    EXPECT_EQ(state_->payload[kLoads * kN + i], -1.0f - static_cast<float>(i));
  }
}

// ---------------------------------------------------------------------------
// SharedMemoryChannel over a real POSIX segment
// ---------------------------------------------------------------------------

TEST_F(SharedMemoryChannelTest, CreateSizesAndZeroFillsEveryArray)
{
  constexpr std::size_t kN = 3;
  auto channel = SharedMemoryChannel::create(name_, kN);
  ASSERT_TRUE(channel.has_value());

  ExternalMapping ext(name_, ActuatorSharedState::size_for(kN));
  ASSERT_TRUE(ext.ok());
  EXPECT_EQ(ext.max_actuators(), kN);
  for (std::size_t slot = 0; slot < ActuatorSharedState::kArrays; ++slot) {
    for (std::size_t i = 0; i < kN; ++i) {
      EXPECT_EQ(ext.read(raw_offset(slot, kN, i)), 0.0) << "slot=" << slot << " i=" << i;
    }
  }
}

TEST_F(SharedMemoryChannelTest, CreateClearsStaleCurrentsAndLoads)
{
  constexpr std::size_t kN = 2;
  {
    auto first = SharedMemoryChannel::create(name_, kN);
    ASSERT_TRUE(first.has_value());
    auto access = SharedMemoryChannel::Access(first.value());
    (*access).hw_currents()[1] = 4.2f;
    (*access).hw_loads()[1] = 0.7f;
  }
  // The segment is not unlinked on release: a restart must not report stale data.
  auto second = SharedMemoryChannel::create(name_, kN);
  ASSERT_TRUE(second.has_value());
  auto access = SharedMemoryChannel::Access(second.value());
  EXPECT_EQ((*access).hw_currents()[1], 0.0);
  EXPECT_EQ((*access).hw_loads()[1], 0.0);
}

TEST_F(SharedMemoryChannelTest, FirmwareWritesAreVisibleThroughCurrentsAndLoads)
{
  constexpr std::size_t kN = 3;
  auto channel = SharedMemoryChannel::create(name_, kN);
  ASSERT_TRUE(channel.has_value());

  ExternalMapping ext(name_, ActuatorSharedState::size_for(kN));
  ASSERT_TRUE(ext.ok());
  for (std::size_t i = 0; i < kN; ++i) {
    ext.write(raw_offset(kEfforts, kN, i), 10.0f + static_cast<float>(i));
    ext.write(raw_offset(kCurrents, kN, i), 0.25f * static_cast<float>(i + 1));
    ext.write(raw_offset(kLoads, kN, i), -0.5f * static_cast<float>(i + 1));
  }

  auto access = SharedMemoryChannel::Access(channel.value());
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_FLOAT_EQ((*access).hw_efforts()[i], 10.0f + static_cast<float>(i));
    EXPECT_FLOAT_EQ((*access).hw_currents()[i], 0.25f * static_cast<float>(i + 1));
    EXPECT_FLOAT_EQ((*access).hw_loads()[i], -0.5f * static_cast<float>(i + 1));
  }
}

TEST_F(SharedMemoryChannelTest, AccessBumpsCommandSeqTwice)
{
  auto channel = SharedMemoryChannel::create(name_, 1);
  ASSERT_TRUE(channel.has_value());
  std::uint64_t before = 0;
  {
    auto access = SharedMemoryChannel::Access(channel.value());
    before = (*access).command_seq.load();
    // Odd while held (seqlock writer side).
    EXPECT_EQ(before % 2, 1u);
  }
  auto access = SharedMemoryChannel::Access(channel.value());
  EXPECT_EQ((*access).command_seq.load(), before + 2);
}

TEST_F(SharedMemoryChannelTest, BareNodeNameGetsLeadingSlash)
{
  const std::string bare = name_.substr(1);
  auto channel = SharedMemoryChannel::create(bare, 1);
  ASSERT_TRUE(channel.has_value());
  ExternalMapping ext(name_, ActuatorSharedState::size_for(1));
  EXPECT_TRUE(ext.ok());
}
