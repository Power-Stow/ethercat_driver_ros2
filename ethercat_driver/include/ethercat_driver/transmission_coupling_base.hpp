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

#ifndef ETHERCAT_DRIVER__TRANSMISSION_COUPLING_BASE_HPP_
#define ETHERCAT_DRIVER__TRANSMISSION_COUPLING_BASE_HPP_

#include <cstddef>
#include <vector>

#include "hardware_interface/hardware_info.hpp"

namespace ethercat_driver
{

class TransmissionCouplingBase
{
public:
  virtual ~TransmissionCouplingBase() = default;

  virtual void configure(
    const hardware_interface::TransmissionInfo & transmission_info,
    const std::vector<hardware_interface::ComponentInfo> & joints) = 0;

  virtual void actuator_to_joint(
    const std::vector<std::vector<double>> & raw_joint_states,
    std::vector<std::vector<double>> & hw_joint_states) = 0;

  virtual void joint_to_actuator(
    const std::vector<std::vector<double>> & hw_joint_commands,
    std::vector<std::vector<double>> & raw_joint_commands) = 0;

  virtual const std::vector<std::size_t> & joint_indices() const = 0;
  virtual const std::vector<std::size_t> & actuator_indices() const = 0;
};

}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__TRANSMISSION_COUPLING_BASE_HPP_
