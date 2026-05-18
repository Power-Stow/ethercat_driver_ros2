// Copyright 2026 Power Stow
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "ethercat_driver/loader_backed_transmission_coupling.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{

hardware_interface::ComponentInfo make_joint(const std::string & name)
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";

  hardware_interface::InterfaceInfo position;
  position.name = hardware_interface::HW_IF_POSITION;
  hardware_interface::InterfaceInfo velocity;
  velocity.name = hardware_interface::HW_IF_VELOCITY;

  joint.state_interfaces = {position, velocity};
  joint.command_interfaces = {position, velocity};
  return joint;
}

hardware_interface::JointInfo make_transmission_joint(
  const std::string & name,
  const std::string & role = "")
{
  hardware_interface::JointInfo joint;
  joint.name = name;
  joint.role = role;
  joint.state_interfaces = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
  joint.command_interfaces = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
  return joint;
}

hardware_interface::ActuatorInfo make_transmission_actuator(
  const std::string & name,
  const std::string & role = "")
{
  hardware_interface::ActuatorInfo actuator;
  actuator.name = name;
  actuator.role = role;
  actuator.state_interfaces = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
  actuator.command_interfaces = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
  return actuator;
}

}  // namespace

TEST(LoaderBackedTransmissionCouplingTest, simple_transmission_maps_state_and_command)
{
  ethercat_driver::LoaderBackedTransmissionCoupling coupling;
  hardware_interface::TransmissionInfo transmission;
  transmission.name = "simple";
  transmission.type = "transmission_interface/SimpleTransmission";
  transmission.joints = {make_transmission_joint("wheel_joint")};
  transmission.actuators = {make_transmission_actuator("wheel_joint")};

  std::vector<hardware_interface::ComponentInfo> joints = {make_joint("wheel_joint")};

  coupling.configure(transmission, joints);

  std::vector<std::vector<double>> raw_joint_states{{3.2, -1.7}};
  std::vector<std::vector<double>> hw_joint_states{{0.0, 0.0}};
  coupling.actuator_to_joint(raw_joint_states, hw_joint_states);
  EXPECT_DOUBLE_EQ(hw_joint_states[0][0], 3.2);
  EXPECT_DOUBLE_EQ(hw_joint_states[0][1], -1.7);

  std::vector<std::vector<double>> hw_joint_commands{{1.2, 0.4}};
  std::vector<std::vector<double>> raw_joint_commands{{0.0, 0.0}};
  coupling.joint_to_actuator(hw_joint_commands, raw_joint_commands);
  EXPECT_DOUBLE_EQ(raw_joint_commands[0][0], 1.2);
  EXPECT_DOUBLE_EQ(raw_joint_commands[0][1], 0.4);
}

TEST(LoaderBackedTransmissionCouplingTest, differential_transmission_maps_state_and_command)
{
  ethercat_driver::LoaderBackedTransmissionCoupling coupling;
  hardware_interface::TransmissionInfo transmission;
  transmission.name = "diff";
  transmission.type = "transmission_interface/DifferentialTransmission";
  transmission.joints = {
    make_transmission_joint("left_joint", "joint1"),
    make_transmission_joint("right_joint", "joint2")};
  transmission.actuators = {
    make_transmission_actuator("left_motor", "actuator1"),
    make_transmission_actuator("right_motor", "actuator2")};

  std::vector<hardware_interface::ComponentInfo> joints = {
    make_joint("left_motor"),
    make_joint("right_motor"),
    make_joint("left_joint"),
    make_joint("right_joint")};

  coupling.configure(transmission, joints);

  std::vector<std::vector<double>> raw_joint_states{{4.0, 2.0}, {2.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  std::vector<std::vector<double>> hw_joint_states{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  coupling.actuator_to_joint(raw_joint_states, hw_joint_states);

  EXPECT_DOUBLE_EQ(hw_joint_states[2][0], 3.0);
  EXPECT_DOUBLE_EQ(hw_joint_states[3][0], 1.0);
  EXPECT_DOUBLE_EQ(hw_joint_states[2][1], 1.0);
  EXPECT_DOUBLE_EQ(hw_joint_states[3][1], 1.0);

  std::vector<std::vector<double>> hw_joint_commands{{0.0, 0.0}, {0.0, 0.0}, {3.0, 1.0}, {1.0, 1.0}};
  std::vector<std::vector<double>> raw_joint_commands{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  coupling.joint_to_actuator(hw_joint_commands, raw_joint_commands);

  EXPECT_DOUBLE_EQ(raw_joint_commands[0][0], 4.0);
  EXPECT_DOUBLE_EQ(raw_joint_commands[1][0], 2.0);
  EXPECT_DOUBLE_EQ(raw_joint_commands[0][1], 2.0);
  EXPECT_DOUBLE_EQ(raw_joint_commands[1][1], 0.0);
}

TEST(LoaderBackedTransmissionCouplingTest, unsupported_transmission_type_throws)
{
  ethercat_driver::LoaderBackedTransmissionCoupling coupling;
  hardware_interface::TransmissionInfo transmission;
  transmission.name = "unsupported";
  transmission.type = "transmission_interface/NotARealTransmission";
  transmission.joints = {make_transmission_joint("joint")};
  transmission.actuators = {make_transmission_actuator("joint")};
  std::vector<hardware_interface::ComponentInfo> joints = {make_joint("joint")};

  EXPECT_THROW(coupling.configure(transmission, joints), std::runtime_error);
}
