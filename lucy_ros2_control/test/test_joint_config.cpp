// Copyright 2025 Sentience Robotics Team
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

/// @file test_joint_config.cpp
/// @brief Unit tests for the pure on_init helpers in joint_config.hpp.
///
/// Exercises parsing, interface validation, actuator-mapping construction and
/// the joint<->servo angle math without a live ROS node, using plain
/// hardware_interface structs.

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "joint_config.hpp"

namespace
{
using hardware_interface::ComponentInfo;
using hardware_interface::InterfaceInfo;
using lucy_ros2_control::ActuatedJointMapping;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;

InterfaceInfo position_interface()
{
  InterfaceInfo info;
  info.name = hardware_interface::HW_IF_POSITION;
  return info;
}

/// A fully valid position-controlled actuator joint with sane calibration.
ComponentInfo make_valid_joint(const std::string & name = "joint_a")
{
  ComponentInfo joint;
  joint.name = name;
  joint.command_interfaces = {position_interface()};
  joint.state_interfaces = {position_interface()};
  joint.parameters = {
    {"virtual_pin", "3"},
    {"offset_rad", "1.570796327"},
    {"direction", "1"},
    {"scale", "1"},
    {"servo_min_rad", "0"},
    {"servo_max_rad", "3.141592654"},
    {"servo_default_rad", "1.570796327"},
  };
  return joint;
}

ActuatedJointMapping make_mapping(int virtual_pin)
{
  ActuatedJointMapping m;
  m.virtual_pin = virtual_pin;
  return m;
}
}  // namespace

// ---------------------------------------------------------------------------
// Angle conversions
// ---------------------------------------------------------------------------

TEST(JointConfigConversions, DegRadRoundTrip)
{
  EXPECT_NEAR(lucy_ros2_control::deg_to_rad(180.0), kPi, 1e-12);
  EXPECT_NEAR(lucy_ros2_control::rad_to_deg(kPi), 180.0, 1e-12);
  EXPECT_NEAR(lucy_ros2_control::rad_to_deg(lucy_ros2_control::deg_to_rad(42.0)), 42.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Numeric parsing
// ---------------------------------------------------------------------------

TEST(JointConfigParse, ParsesValidDouble)
{
  EXPECT_DOUBLE_EQ(lucy_ros2_control::parse_double_string_or_throw("1.5", "ctx"), 1.5);
  EXPECT_DOUBLE_EQ(lucy_ros2_control::parse_double_string_or_throw("-3", "ctx"), -3.0);
}

TEST(JointConfigParse, ThrowsOnNonNumeric)
{
  EXPECT_THROW(
    lucy_ros2_control::parse_double_string_or_throw("sdhjsdf", "ctx"),
    std::runtime_error);
}

TEST(JointConfigParse, RequiredDoublePresent)
{
  const ComponentInfo joint = make_valid_joint();
  EXPECT_NEAR(
    lucy_ros2_control::parse_required_double(joint, "offset_rad"),
    1.570796327, 1e-12);
}

TEST(JointConfigParse, RequiredDoubleMissingThrows)
{
  const ComponentInfo joint = make_valid_joint();
  EXPECT_THROW(lucy_ros2_control::parse_required_double(joint, "no_such_key"), std::runtime_error);
}

TEST(JointConfigParse, RequiredDoubleEmptyThrows)
{
  ComponentInfo joint = make_valid_joint();
  joint.parameters["offset_rad"] = "";
  EXPECT_THROW(lucy_ros2_control::parse_required_double(joint, "offset_rad"), std::runtime_error);
}

TEST(JointConfigParse, RequiredDoubleNonNumericThrows)
{
  ComponentInfo joint = make_valid_joint();
  joint.parameters["scale"] = "abc";
  EXPECT_THROW(lucy_ros2_control::parse_required_double(joint, "scale"), std::runtime_error);
}

TEST(JointConfigParse, OptionalInterfaceLimitFallsBackWhenEmpty)
{
  EXPECT_DOUBLE_EQ(
    lucy_ros2_control::parse_optional_interface_limit(std::nullopt, "ctx", -kInf), -kInf);
  EXPECT_DOUBLE_EQ(
    lucy_ros2_control::parse_optional_interface_limit(std::string(""), "ctx", 7.0), 7.0);
}

TEST(JointConfigParse, OptionalInterfaceLimitParsesAndThrows)
{
  EXPECT_DOUBLE_EQ(
    lucy_ros2_control::parse_optional_interface_limit(std::string("1.25"), "ctx", 0.0), 1.25);
  EXPECT_THROW(
    lucy_ros2_control::parse_optional_interface_limit(std::string("nope"), "ctx", 0.0),
    std::runtime_error);
}

// ---------------------------------------------------------------------------
// Interface validation
// ---------------------------------------------------------------------------

TEST(JointConfigValidate, ValidJointReturnsEmpty)
{
  EXPECT_TRUE(lucy_ros2_control::validate_joint_interfaces(make_valid_joint()).empty());
}

TEST(JointConfigValidate, WrongCommandInterfaceCountFails)
{
  ComponentInfo joint = make_valid_joint();
  joint.command_interfaces.clear();
  EXPECT_FALSE(lucy_ros2_control::validate_joint_interfaces(joint).empty());
}

TEST(JointConfigValidate, WrongCommandInterfaceNameFails)
{
  ComponentInfo joint = make_valid_joint();
  joint.command_interfaces[0].name = hardware_interface::HW_IF_VELOCITY;
  EXPECT_FALSE(lucy_ros2_control::validate_joint_interfaces(joint).empty());
}

TEST(JointConfigValidate, WrongStateInterfaceCountFails)
{
  ComponentInfo joint = make_valid_joint();
  joint.state_interfaces.push_back(position_interface());
  EXPECT_FALSE(lucy_ros2_control::validate_joint_interfaces(joint).empty());
}

TEST(JointConfigValidate, WrongStateInterfaceNameFails)
{
  ComponentInfo joint = make_valid_joint();
  joint.state_interfaces[0].name = hardware_interface::HW_IF_EFFORT;
  EXPECT_FALSE(lucy_ros2_control::validate_joint_interfaces(joint).empty());
}

// ---------------------------------------------------------------------------
// URDF command_interface limits
// ---------------------------------------------------------------------------

TEST(JointConfigLimits, UnsetLimitsDefaultToInfinity)
{
  const lucy_ros2_control::JointLimits limits =
    lucy_ros2_control::parse_joint_limits(make_valid_joint());
  EXPECT_FALSE(std::isfinite(limits.min_rad));
  EXPECT_FALSE(std::isfinite(limits.max_rad));
  EXPECT_LT(limits.min_rad, 0.0);
  EXPECT_GT(limits.max_rad, 0.0);
}

TEST(JointConfigLimits, ParsesProvidedLimits)
{
  ComponentInfo joint = make_valid_joint();
  joint.command_interfaces[0].min = "-1.5708";
  joint.command_interfaces[0].max = "1.5708";
  const lucy_ros2_control::JointLimits limits = lucy_ros2_control::parse_joint_limits(joint);
  EXPECT_DOUBLE_EQ(limits.min_rad, -1.5708);
  EXPECT_DOUBLE_EQ(limits.max_rad, 1.5708);
}

TEST(JointConfigLimits, NonNumericLimitThrows)
{
  ComponentInfo joint = make_valid_joint();
  joint.command_interfaces[0].min = "not_a_number";
  EXPECT_THROW(lucy_ros2_control::parse_joint_limits(joint), std::runtime_error);
}

// ---------------------------------------------------------------------------
// Actuator mapping construction
// ---------------------------------------------------------------------------

TEST(JointConfigMapping, BuildsValidMapping)
{
  const ComponentInfo joint = make_valid_joint();
  const auto m = lucy_ros2_control::build_actuated_joint_mapping(joint, 7, -1.0, 2.0);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->joint_index, 7u);
  EXPECT_EQ(m->virtual_pin, 3);
  EXPECT_NEAR(m->offset_rad, 1.570796327, 1e-12);
  EXPECT_NEAR(m->servo_default_rad, 1.570796327, 1e-12);
  EXPECT_DOUBLE_EQ(m->min_rad, -1.0);
  EXPECT_DOUBLE_EQ(m->max_rad, 2.0);
}

TEST(JointConfigMapping, NoVirtualPinReturnsNullopt)
{
  ComponentInfo joint = make_valid_joint();
  joint.parameters.erase("virtual_pin");
  EXPECT_FALSE(
    lucy_ros2_control::build_actuated_joint_mapping(joint, 0, -kInf, kInf).has_value());
}

TEST(JointConfigMapping, NonFiniteLimitsStillBuild)
{
  const ComponentInfo joint = make_valid_joint();
  const auto m = lucy_ros2_control::build_actuated_joint_mapping(joint, 0, -kInf, kInf);
  ASSERT_TRUE(m.has_value());
  EXPECT_FALSE(std::isfinite(m->min_rad));
  EXPECT_FALSE(std::isfinite(m->max_rad));
}

TEST(JointConfigMapping, NegativeVirtualPinThrows)
{
  ComponentInfo joint = make_valid_joint();
  joint.parameters["virtual_pin"] = "-1";
  EXPECT_THROW(
    lucy_ros2_control::build_actuated_joint_mapping(joint, 0, -kInf, kInf), std::runtime_error);
}

TEST(JointConfigMapping, NonNumericVirtualPinThrows)
{
  ComponentInfo joint = make_valid_joint();
  joint.parameters["virtual_pin"] = "pin5";
  EXPECT_THROW(
    lucy_ros2_control::build_actuated_joint_mapping(joint, 0, -kInf, kInf), std::runtime_error);
}

TEST(JointConfigMapping, ZeroDirectionOrScaleThrows)
{
  ComponentInfo zero_dir = make_valid_joint();
  zero_dir.parameters["direction"] = "0";
  EXPECT_THROW(
    lucy_ros2_control::build_actuated_joint_mapping(zero_dir, 0, -kInf, kInf), std::runtime_error);

  ComponentInfo zero_scale = make_valid_joint();
  zero_scale.parameters["scale"] = "0";
  EXPECT_THROW(
    lucy_ros2_control::build_actuated_joint_mapping(zero_scale, 0, -kInf, kInf),
    std::runtime_error);
}

TEST(JointConfigMapping, InvertedServoRangeThrows)
{
  ComponentInfo joint = make_valid_joint();
  joint.parameters["servo_min_rad"] = "2.1";
  joint.parameters["servo_max_rad"] = "0.5";
  EXPECT_THROW(
    lucy_ros2_control::build_actuated_joint_mapping(joint, 0, -kInf, kInf), std::runtime_error);
}

TEST(JointConfigMapping, InvertedFiniteUrdfRangeThrows)
{
  const ComponentInfo joint = make_valid_joint();
  EXPECT_THROW(
    lucy_ros2_control::build_actuated_joint_mapping(joint, 0, 2.0, -2.0), std::runtime_error);
}

TEST(JointConfigMapping, MissingCalibrationParamThrows)
{
  ComponentInfo joint = make_valid_joint();
  joint.parameters.erase("offset_rad");
  EXPECT_THROW(
    lucy_ros2_control::build_actuated_joint_mapping(joint, 0, -kInf, kInf), std::runtime_error);
}

// ---------------------------------------------------------------------------
// Joint <-> servo math
// ---------------------------------------------------------------------------

TEST(JointConfigMath, DefaultJointPositionRad)
{
  ActuatedJointMapping m;
  m.offset_rad = 1.5707963267948966;
  m.direction = 1.0;
  m.scale = 1.0;
  m.servo_default_rad = 1.5707963267948966;
  // (π/2 - π/2) * 1 * 1 = 0
  EXPECT_NEAR(lucy_ros2_control::default_joint_position_rad(m), 0.0, 1e-12);

  m.servo_default_rad = 1.5707963267948966 + kPi / 4.0;
  // (π/2 + π/4 - π/2) = π/4
  EXPECT_NEAR(lucy_ros2_control::default_joint_position_rad(m), kPi / 4.0, 1e-12);
}

TEST(JointConfigMath, ActuatorCommandToServoRadWithinRange)
{
  ActuatedJointMapping m;
  m.offset_rad = 1.5707963267948966;
  m.direction = 1.0;
  m.scale = 1.0;
  m.servo_min_rad = 0.0;
  m.servo_max_rad = 3.141592653589793;
  // cmd 0 rad -> servo = 0 + π/2
  EXPECT_NEAR(lucy_ros2_control::actuator_command_to_servo_rad(m, 0.0), kPi / 2.0, 1e-12);
}

TEST(JointConfigMath, ActuatorCommandToServoRadClampsHigh)
{
  ActuatedJointMapping m;
  m.offset_rad = 1.5707963267948966;
  m.direction = 1.0;
  m.scale = 1.0;
  m.servo_min_rad = 0.0;
  m.servo_max_rad = 3.141592653589793;
  // cmd π -> servo = π + π/2 = 3π/2 -> clamps to π
  EXPECT_NEAR(lucy_ros2_control::actuator_command_to_servo_rad(m, kPi), kPi, 1e-12);
}

TEST(JointConfigMath, ActuatorCommandHonoursDirection)
{
  ActuatedJointMapping m;
  m.offset_rad = 1.5707963267948966;
  m.direction = -1.0;
  m.scale = 1.0;
  m.servo_min_rad = 0.0;
  m.servo_max_rad = 3.141592653589793;
  // cmd π/2 -> servo = (π/2)/(-1) + π/2 = 0
  EXPECT_NEAR(lucy_ros2_control::actuator_command_to_servo_rad(m, kPi / 2.0), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Duplicate virtual_pin detection / sort
// ---------------------------------------------------------------------------

TEST(JointConfigDuplicates, UniquePinsSortAscendingNoDuplicate)
{
  std::vector<ActuatedJointMapping> mappings = {
    make_mapping(5), make_mapping(1), make_mapping(3)};
  const auto dup = lucy_ros2_control::sort_and_find_duplicate_virtual_pin(mappings);
  EXPECT_FALSE(dup.has_value());
  ASSERT_EQ(mappings.size(), 3u);
  EXPECT_EQ(mappings[0].virtual_pin, 1);
  EXPECT_EQ(mappings[1].virtual_pin, 3);
  EXPECT_EQ(mappings[2].virtual_pin, 5);
}

TEST(JointConfigDuplicates, DetectsDuplicatePin)
{
  std::vector<ActuatedJointMapping> mappings = {
    make_mapping(2), make_mapping(4), make_mapping(2)};
  const auto dup = lucy_ros2_control::sort_and_find_duplicate_virtual_pin(mappings);
  ASSERT_TRUE(dup.has_value());
  EXPECT_EQ(dup.value(), 2);
}

TEST(JointConfigDuplicates, EmptyListHasNoDuplicate)
{
  std::vector<ActuatedJointMapping> mappings;
  EXPECT_FALSE(lucy_ros2_control::sort_and_find_duplicate_virtual_pin(mappings).has_value());
}
