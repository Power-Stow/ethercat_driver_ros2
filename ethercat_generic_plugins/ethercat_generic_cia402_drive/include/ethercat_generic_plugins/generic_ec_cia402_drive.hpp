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
  virtual void start_wind_down(double timeout_s);

  /** True once Disable Voltage has gone out and the drive reads as de-energised.
   *  Judged on the status word alone, never on the cached operational flag. */
  virtual bool wind_down_complete() const noexcept;

  /** Release the control word and the other command channels, so the drive can be taken back up
   *  to Operation Enabled. Called on activation: the same plugin instance is reused across a
   *  deactivate/activate cycle, and a wind-down left in place would keep commanding the drive
   *  down forever. Also re-arms the startup fault reset, which is per activation. */
  virtual void reset_wind_down();

  /// @brief Setup CSV dumping internals from plugin parameters.
  void setup_csv_dump();

  /// @brief Dump one CSV row for the current cycle.
  void dump_cycle_csv_row();

  int8_t mode_of_operation_display_ = 0;
  int8_t mode_of_operation_ = -1;

  void updateState();

  /** The error code the drive reported for the fault it is in, or the last one it was in.
   *  Survives the reset that clears 0x603F on the drive itself, so a fault that was acknowledged
   *  automatically still leaves something to read. Zero until the drive has faulted once. */
  uint16_t last_fault_error_code() const noexcept {return last_fault_error_code_;}

protected:
  uint32_t counter_ = 0;
  uint16_t last_status_word_ = -1;
  uint16_t status_word_ = 0;
  uint16_t control_word_ = 0;
  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;
  bool initialized_ = false;
  bool auto_fault_reset_ = false;
  /** Clear a fault the drive came up in, once, before it is first seen in a non-fault state.
   *  Separate from auto_fault_reset, which clears every fault for as long as the driver runs: a drive
   *  that was left in Fault by a previous session has nothing to do with the session about to start, and
   *  with auto_fault_reset off there is nothing else to clear it. No controller exists yet during
   *  bring-up, so the command interface cannot be used either. */
  bool reset_fault_on_startup_ = true;
  /** Set once the drive is seen in a non-fault state past Not Ready to Switch On. From then on a
   *  fault is this session's, and the startup reset no longer applies to it. */
  bool startup_fault_window_closed_ = false;
  bool startup_fault_reset_logged_ = false;
  bool auto_state_transitions_ = true;
  bool fault_reset_ = false;
  int fault_reset_command_interface_index_ = -1;
  bool last_fault_reset_command_ = false;
  uint16_t error_code_ = 0;
  /** Latched at the fault edge rather than read live, because a fault reset clears 0x603F on the
   *  drive within a cycle or two and an automatic reset gets there before anything can read it. */
  uint16_t last_fault_error_code_ = 0;
  uint16_t last_fault_status_word_ = 0;
  bool fault_error_code_logged_ = false;
  int last_error_code_state_interface_index_ = -1;
  bool initialization_position_logged_ = false;
  double last_raw_position_ = std::numeric_limits<double>::quiet_NaN();
  bool joint_offset_startup_wrap_enabled_ = false;
  bool joint_offset_startup_wrap_applied_ = false;
  bool wind_down_requested_ = false;
  bool wind_down_complete_ = true;
  /** Whether the control word written on this cycle's RPDO pass was Disable Voltage. */
  bool wind_down_disable_voltage_sent_ = false;
  bool quick_stop_supported_ = false;
  /** Until when a drive in Quick Stop Active is left on its quick stop ramp before Disable Voltage.
   *  A steady_clock deadline, so a scheduling stall cannot use up the ramp in a burst of cycles. */
  std::chrono::steady_clock::time_point quick_stop_hold_until_{};
  /** Each channel's override_command, taken when the wind-down starts and put back when it is
   *  reset. The wind-down forces them all true, and that is channel state which outlives the
   *  wind-down itself. Invariant: empty, or exactly one entry per element of pdo_channels_info_. */
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

  /** Record the error code of the fault the drive is in, if it is in one. */
  void latch_fault_error_code();
  /** Write the latched error code to its state interface, when the URDF declares one. */
  void publish_last_error_code();
  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word);
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word);
  /** returns the control word that walks the device down towards Switch On Disabled */
  uint16_t wind_down_transition(DeviceState state) const noexcept;
  /** Mark the wind-down complete once Disable Voltage has gone out and the state read on this cycle
   *  is one with the drive function disabled. Called at the end of each cycle, after updateState(). */
  void update_wind_down_complete();
  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node drive_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
