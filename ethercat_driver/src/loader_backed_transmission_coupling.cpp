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

#include "ethercat_driver/loader_backed_transmission_coupling.hpp"

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <transmission_interface/differential_transmission_loader.hpp>
#include <transmission_interface/exception.hpp>
#include <transmission_interface/handle.hpp>
#include <transmission_interface/simple_transmission_loader.hpp>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace ethercat_driver
{

namespace
{

std::size_t role_to_slot(const std::string & role, const std::string & prefix, std::size_t default_slot)
{
  if (role.empty()) {
    return default_slot;
  }

  if (role.rfind(prefix, 0) != 0) {
    throw std::runtime_error(
            "Unsupported transmission role '" + role + "'. Expected prefix '" + prefix + "'.");
  }

  const auto suffix = role.substr(prefix.size());
  if (suffix.empty()) {
    throw std::runtime_error("Unsupported transmission role '" + role + "'. Missing slot suffix.");
  }

  const auto slot_index = std::stoul(suffix);
  if (slot_index == 0U) {
    throw std::runtime_error("Unsupported transmission role '" + role + "'. Slots are 1-based.");
  }

  return slot_index - 1U;
}

}  // namespace

void LoaderBackedTransmissionCoupling::configure(
  const hardware_interface::TransmissionInfo & transmission_info,
  const std::vector<hardware_interface::ComponentInfo> & joints)
{
  if (transmission_info.type == "transmission_interface/SimpleTransmission") {
    transmission_interface::SimpleTransmissionLoader loader;
    transmission_ = loader.load(transmission_info);
  } else if (transmission_info.type == "transmission_interface/DifferentialTransmission") {
    transmission_interface::DifferentialTransmissionLoader loader;
    transmission_ = loader.load(transmission_info);
  } else {
    throw std::runtime_error("Unsupported transmission type: " + transmission_info.type);
  }

  if (!transmission_) {
    throw std::runtime_error("Failed to load transmission: " + transmission_info.name);
  }

  joint_names_.clear();
  actuator_names_.clear();
  joint_indices_.clear();
  actuator_indices_.clear();
  joint_state_interface_indices_.clear();
  joint_command_interface_indices_.clear();
  actuator_state_interface_indices_.clear();
  actuator_command_interface_indices_.clear();

  std::vector<std::size_t> joint_storage_by_slot(
    transmission_info.joints.size(), std::numeric_limits<std::size_t>::max());

  for (std::size_t joint_order = 0; joint_order < transmission_info.joints.size(); ++joint_order) {
    const auto & joint = transmission_info.joints[joint_order];
    joint_names_.push_back(joint.name);
    const auto joint_index = find_joint_index(joints, joint.name);
    joint_indices_.push_back(joint_index);

    const auto joint_slot = role_to_slot(joint.role, "joint", joint_order);
    if (joint_slot >= joint_storage_by_slot.size()) {
      throw std::runtime_error(
              "Transmission joint role slot out of range for joint '" + joint.name + "'.");
    }
    if (joint_storage_by_slot[joint_slot] != std::numeric_limits<std::size_t>::max()) {
      throw std::runtime_error(
              "Duplicate transmission joint role slot for joint '" + joint.name + "'.");
    }
    joint_storage_by_slot[joint_slot] = joint_index;

    joint_state_interface_indices_.push_back(
      InterfaceIndices{
        find_interface_index(joints[joint_index].state_interfaces, hardware_interface::HW_IF_POSITION),
        find_interface_index(joints[joint_index].state_interfaces, hardware_interface::HW_IF_VELOCITY),
        find_interface_index(joints[joint_index].state_interfaces, hardware_interface::HW_IF_EFFORT)});
    joint_command_interface_indices_.push_back(
      InterfaceIndices{
        find_interface_index(joints[joint_index].command_interfaces, hardware_interface::HW_IF_POSITION),
        find_interface_index(joints[joint_index].command_interfaces, hardware_interface::HW_IF_VELOCITY),
        find_interface_index(joints[joint_index].command_interfaces, hardware_interface::HW_IF_EFFORT)});
  }

  std::vector<std::size_t> actuator_storage_by_slot(
    transmission_info.actuators.size(), std::numeric_limits<std::size_t>::max());

  for (std::size_t actuator_order = 0; actuator_order < transmission_info.actuators.size(); ++actuator_order) {
    const auto & actuator = transmission_info.actuators[actuator_order];
    actuator_names_.push_back(actuator.name);
    const auto actuator_slot = role_to_slot(actuator.role, "actuator", actuator_order);
    if (actuator_slot >= actuator_storage_by_slot.size()) {
      throw std::runtime_error(
              "Transmission actuator role slot out of range for actuator '" + actuator.name + "'.");
    }
    if (actuator_storage_by_slot[actuator_slot] != std::numeric_limits<std::size_t>::max()) {
      throw std::runtime_error(
              "Duplicate transmission actuator role slot for actuator '" + actuator.name + "'.");
    }

    const auto actuator_index = resolve_actuator_index(
      joints, actuator, actuator_slot, actuator_storage_by_slot.size());
    actuator_storage_by_slot[actuator_slot] = actuator_index;
    actuator_indices_.push_back(actuator_index);
    actuator_state_interface_indices_.push_back(
      InterfaceIndices{
        find_interface_index(joints[actuator_index].state_interfaces, hardware_interface::HW_IF_POSITION),
        find_interface_index(joints[actuator_index].state_interfaces, hardware_interface::HW_IF_VELOCITY),
        find_interface_index(joints[actuator_index].state_interfaces, hardware_interface::HW_IF_EFFORT)});
    actuator_command_interface_indices_.push_back(
      InterfaceIndices{
        find_interface_index(joints[actuator_index].command_interfaces, hardware_interface::HW_IF_POSITION),
        find_interface_index(joints[actuator_index].command_interfaces, hardware_interface::HW_IF_VELOCITY),
        find_interface_index(joints[actuator_index].command_interfaces, hardware_interface::HW_IF_EFFORT)});
  }
}

void LoaderBackedTransmissionCoupling::actuator_to_joint(
  const std::vector<std::vector<double>> & raw_joint_states,
  std::vector<std::vector<double>> & hw_joint_states)
{
  std::vector<std::vector<double>> joint_values;
  std::vector<std::vector<double>> actuator_values;
  joint_values.reserve(joint_indices_.size());
  actuator_values.reserve(actuator_indices_.size());

  for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
    const auto index = joint_indices_[i];
    hw_joint_states[index] = raw_joint_states[index];
    joint_values.push_back(hw_joint_states[index]);
  }
  for (const auto index : actuator_indices_) {
    actuator_values.push_back(raw_joint_states[index]);
    hw_joint_states[index] = raw_joint_states[index];
  }

  std::vector<transmission_interface::JointHandle> joint_handles;
  std::vector<transmission_interface::ActuatorHandle> actuator_handles;
  for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
    append_joint_handles(joint_handles, joint_names_[i], joint_state_interface_indices_[i], joint_values[i]);
  }
  for (std::size_t i = 0; i < actuator_indices_.size(); ++i) {
    append_actuator_handles(
      actuator_handles, actuator_names_[i], actuator_state_interface_indices_[i], actuator_values[i]);
  }

  try {
    transmission_->configure(joint_handles, actuator_handles);
    transmission_->actuator_to_joint();
  } catch (const transmission_interface::Exception & ex) {
    throw std::runtime_error(std::string("Failed actuator_to_joint mapping: ") + ex.what());
  }

  for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
    hw_joint_states[joint_indices_[i]] = joint_values[i];
  }
}

void LoaderBackedTransmissionCoupling::joint_to_actuator(
  const std::vector<std::vector<double>> & hw_joint_commands,
  std::vector<std::vector<double>> & raw_joint_commands)
{
  std::vector<std::vector<double>> joint_values;
  std::vector<std::vector<double>> actuator_values;
  joint_values.reserve(joint_indices_.size());
  actuator_values.reserve(actuator_indices_.size());

  for (const auto index : joint_indices_) {
    joint_values.push_back(hw_joint_commands[index]);
  }
  for (const auto index : actuator_indices_) {
    actuator_values.push_back(raw_joint_commands[index]);
  }

  std::vector<transmission_interface::JointHandle> joint_handles;
  std::vector<transmission_interface::ActuatorHandle> actuator_handles;
  for (std::size_t i = 0; i < joint_indices_.size(); ++i) {
    append_joint_handles(joint_handles, joint_names_[i], joint_command_interface_indices_[i], joint_values[i]);
  }
  for (std::size_t i = 0; i < actuator_indices_.size(); ++i) {
    append_actuator_handles(
      actuator_handles, actuator_names_[i], actuator_command_interface_indices_[i], actuator_values[i]);
  }

  try {
    transmission_->configure(joint_handles, actuator_handles);
    transmission_->joint_to_actuator();
  } catch (const transmission_interface::Exception & ex) {
    throw std::runtime_error(std::string("Failed joint_to_actuator mapping: ") + ex.what());
  }

  for (std::size_t i = 0; i < actuator_indices_.size(); ++i) {
    raw_joint_commands[actuator_indices_[i]] = actuator_values[i];
  }
}

const std::vector<std::size_t> & LoaderBackedTransmissionCoupling::joint_indices() const
{
  return joint_indices_;
}

const std::vector<std::size_t> & LoaderBackedTransmissionCoupling::actuator_indices() const
{
  return actuator_indices_;
}

std::size_t LoaderBackedTransmissionCoupling::find_joint_index(
  const std::vector<hardware_interface::ComponentInfo> & joints,
  const std::string & joint_name)
{
  const auto joint_it = std::find_if(
    joints.begin(), joints.end(),
    [&joint_name](const auto & joint_info) { return joint_info.name == joint_name; });
  if (joint_it == joints.end()) {
    throw std::runtime_error("Transmission joint/actuator '" + joint_name + "' is not declared as a ros2_control joint.");
  }
  return static_cast<std::size_t>(std::distance(joints.begin(), joint_it));
}

std::size_t LoaderBackedTransmissionCoupling::resolve_actuator_index(
  const std::vector<hardware_interface::ComponentInfo> & joints,
  const hardware_interface::ActuatorInfo & actuator,
  std::size_t actuator_slot,
  std::size_t actuator_slot_count)
{
  if (actuator_slot >= actuator_slot_count) {
    throw std::runtime_error(
            "Transmission actuator role slot out of range for transmission actuator '" +
            actuator.name + "'.");
  }

  return find_joint_index(joints, actuator.name);
}

int LoaderBackedTransmissionCoupling::find_interface_index(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & interface_name)
{
  for (std::size_t i = 0; i < interfaces.size(); ++i) {
    if (interfaces[i].name == interface_name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void LoaderBackedTransmissionCoupling::append_joint_handles(
  std::vector<transmission_interface::JointHandle> & handles,
  const std::string & name,
  const InterfaceIndices & indices,
  std::vector<double> & values)
{
  if (indices.position >= 0) {
    handles.emplace_back(name, hardware_interface::HW_IF_POSITION, &values[static_cast<std::size_t>(indices.position)]);
  }
  if (indices.velocity >= 0) {
    handles.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &values[static_cast<std::size_t>(indices.velocity)]);
  }
  if (indices.effort >= 0) {
    handles.emplace_back(name, hardware_interface::HW_IF_EFFORT, &values[static_cast<std::size_t>(indices.effort)]);
  }
}

void LoaderBackedTransmissionCoupling::append_actuator_handles(
  std::vector<transmission_interface::ActuatorHandle> & handles,
  const std::string & name,
  const InterfaceIndices & indices,
  std::vector<double> & values)
{
  if (indices.position >= 0) {
    handles.emplace_back(name, hardware_interface::HW_IF_POSITION, &values[static_cast<std::size_t>(indices.position)]);
  }
  if (indices.velocity >= 0) {
    handles.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &values[static_cast<std::size_t>(indices.velocity)]);
  }
  if (indices.effort >= 0) {
    handles.emplace_back(name, hardware_interface::HW_IF_EFFORT, &values[static_cast<std::size_t>(indices.effort)]);
  }
}

}  // namespace ethercat_driver
