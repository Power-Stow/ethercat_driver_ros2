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

#ifndef ETHERCAT_DRIVER__LOADER_BACKED_TRANSMISSION_COUPLING_HPP_
#define ETHERCAT_DRIVER__LOADER_BACKED_TRANSMISSION_COUPLING_HPP_

#include <ethercat_driver/transmission_coupling_base.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <transmission_interface/handle.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>


namespace transmission_interface
{
class Transmission;
}  // namespace transmission_interface

namespace ethercat_driver
{

class LoaderBackedTransmissionCoupling : public TransmissionCouplingBase
{
public:
  void configure(
    const hardware_interface::TransmissionInfo & transmission_info,
    const std::vector<hardware_interface::ComponentInfo> & joints) override;

  void actuator_to_joint(
    const std::vector<std::vector<double>> & raw_joint_states,
    std::vector<std::vector<double>> & hw_joint_states) override;

  void joint_to_actuator(
    const std::vector<std::vector<double>> & hw_joint_commands,
    std::vector<std::vector<double>> & raw_joint_commands) override;

  const std::vector<std::size_t> & joint_indices() const override;

  const std::vector<std::size_t> & actuator_indices() const override;

private:
  struct InterfaceIndices
  {
    int position = -1;
    int velocity = -1;
    int effort = -1;
  };

  static std::size_t find_joint_index(
    const std::vector<hardware_interface::ComponentInfo> & joints,
    const std::string & joint_name);

  static std::size_t resolve_actuator_index(
    const std::vector<hardware_interface::ComponentInfo> & joints,
    const hardware_interface::ActuatorInfo & actuator,
    std::size_t actuator_slot,
    std::size_t actuator_slot_count);

  static int find_interface_index(
    const std::vector<hardware_interface::InterfaceInfo> & interfaces,
    const std::string & interface_name);

  static void append_joint_handles(
    std::vector<transmission_interface::JointHandle> & handles,
    const std::string & name,
    const InterfaceIndices & indices,
    std::vector<double> & values);

  static void append_actuator_handles(
    std::vector<transmission_interface::ActuatorHandle> & handles,
    const std::string & name,
    const InterfaceIndices & indices,
    std::vector<double> & values);

  std::shared_ptr<transmission_interface::Transmission> transmission_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> actuator_names_;
  std::vector<std::size_t> joint_indices_;
  std::vector<std::size_t> actuator_indices_;
  std::vector<InterfaceIndices> joint_state_interface_indices_;
  std::vector<InterfaceIndices> joint_command_interface_indices_;
  std::vector<InterfaceIndices> actuator_state_interface_indices_;
  std::vector<InterfaceIndices> actuator_command_interface_indices_;
};

}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__LOADER_BACKED_TRANSMISSION_COUPLING_HPP_
