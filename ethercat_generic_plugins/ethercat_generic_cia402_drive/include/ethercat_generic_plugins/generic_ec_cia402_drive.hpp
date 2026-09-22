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

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_

#include <chrono>
#include <fstream>
#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>
#include <limits>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "ethercat_generic_plugins/cia402_common_defs.hpp"

namespace ethercat_generic_plugins
{

class EcCiA402Drive : public GenericEcSlave
{
public:
  EcCiA402Drive();
  virtual ~EcCiA402Drive();
  /** Returns true if drive has reached "operation enabled" state.
   *  The transition through the state machine is handled automatically. */
  bool initialized();

  virtual void processData(size_t entry_idx, uint8_t * domain_address);

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_parameters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  /** Start walking the CiA-402 state machine down to Switch On Disabled.
   *  Drives whose slave config sets `quick_stop_supported` are commanded Quick Stop, so they
   *  decelerate on their quick stop ramp; every other drive is disabled, dropping the power stage
   *  so the axis is held by its brake. While the wind-down runs, every command channel but the
   *  control word falls back to its configured default. */
  virtual void start_wind_down(double cycle_period_s, double timeout_s);

  /** True once the drive has reached a de-energised state, or was never operational. */
  virtual bool wind_down_complete();

  /** Release the control word and the other command channels, so the drive can be taken back up
   *  to Operation Enabled. Called on activation: the same plugin instance is reused across a
   *  deactivate/activate cycle, and a wind-down left in place would keep commanding the drive
   *  down forever. */
  virtual void reset_wind_down();

  /// @brief Setup CSV dumping internals from plugin parameters.
  void setup_csv_dump();

  /// @brief Dump one CSV row for the current cycle.
  void dump_cycle_csv_row();

  int8_t mode_of_operation_display_ = 0;
  int8_t mode_of_operation_ = -1;

  void updateState();

protected:
  uint32_t counter_ = 0;
  uint16_t last_status_word_ = -1;
  uint16_t status_word_ = 0;
  uint16_t control_word_ = 0;
  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;
  bool initialized_ = false;
  bool auto_fault_reset_ = false;
  bool auto_state_transitions_ = true;
  bool fault_reset_ = false;
  int fault_reset_command_interface_index_ = -1;
  bool last_fault_reset_command_ = false;
  bool initialization_position_logged_ = false;
  double last_raw_position_ = std::numeric_limits<double>::quiet_NaN();
  bool joint_offset_startup_wrap_enabled_ = false;
  bool joint_offset_startup_wrap_applied_ = false;
  bool wind_down_requested_ = false;
  bool wind_down_complete_ = true;
  bool quick_stop_supported_ = false;
  uint32_t wind_down_cycles_ = 0;
  uint32_t quick_stop_hold_cycles_ = 0;
  /** Each channel's configured override_command, taken when the wind-down starts and put back
   *  when it is reset. The wind-down forces them all true, and that is channel state which
   *  outlives the wind-down itself. */
  std::vector<bool> pre_wind_down_override_command_;
  double joint_offset_ = 0.0;
  double last_position_ = std::numeric_limits<double>::quiet_NaN();

  bool csv_dump_enabled_ = false;
  bool csv_header_written_ = false;
  std::string csv_dump_path_;
  std::size_t csv_flush_every_n_ = 1;
  std::uint64_t csv_cycle_counter_ = 0;
  std::ofstream csv_dump_file_;
  std::vector<std::size_t> csv_rpdo_domain_indices_;
  std::vector<std::size_t> csv_tpdo_domain_indices_;
  std::chrono::steady_clock::time_point csv_t0_ = std::chrono::steady_clock::now();

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word);
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word);
  /** returns the control word that walks the device down towards Switch On Disabled */
  uint16_t wind_down_transition(DeviceState state);
  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node drive_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
