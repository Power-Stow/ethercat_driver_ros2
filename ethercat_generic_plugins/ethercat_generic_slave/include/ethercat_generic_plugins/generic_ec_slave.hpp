// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_SLAVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_SLAVE_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_interface/ec_sync_manager.hpp"

namespace ethercat_generic_plugins
{

class GenericEcSlave : public ethercat_interface::EcSlave
{
public:
  GenericEcSlave();
  virtual ~GenericEcSlave();
  virtual int assign_activate_dc_sync();

  virtual void processData(size_t entry_idx, uint8_t * domain_address);

  virtual const ec_sync_info_t * syncs();
  virtual size_t syncSize();
  virtual const ec_pdo_entry_info_t * channels();
  virtual void domains(DomainMap & domains) const;

  /// Read the factor of every channel that asks for one from the drive. See
  /// `EcSlave::resolve_sdo_factors`.
  bool resolve_sdo_factors(const ethercat_interface::EcSlave::SdoReader & read_sdo) override;

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_parameters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

protected:
  uint32_t counter_ = 0;
  std::vector<ec_pdo_info_t> rpdos_;
  std::vector<ec_pdo_info_t> tpdos_;
  std::vector<bool> all_channels_skip_list_;
  std::vector<ec_pdo_entry_info_t> all_channels_;
  std::vector<ethercat_interface::EcPdoChannelManager *> pdo_channels_info_;
  std::vector<ethercat_interface::SMConfig> sm_configs_;
  std::vector<ec_sync_info_t> syncs_;
  std::vector<unsigned int> domain_map_;
  YAML::Node slave_config_;
  uint32_t assign_activate_ = 0;
  /// RPDO channel mapped to the `position` command interface, which holds the read position while
  /// its command is NaN.
  ethercat_interface::EcPdoSingleInterfaceChannelManager * position_command_channel_ = nullptr;
  /// State interface index of the TPDO channel mapped to the `position` state interface.
  std::optional<size_t> position_state_index_;

  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node slave_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);

  void setup_syncs();

  void setup_interface_mapping();

  /// Write the position reading to the position command channel, if its command is released.
  /// @return false when the command is set or no position reading exists, so the channel's own
  ///   update applies, which writes the command or else the configured default.
  bool hold_read_position(uint8_t * domain_address);
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_SLAVE_HPP_
