// Copyright 2026 Power Stow A/S
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

#ifndef ETHERCAT_INTERFACE__EC_DIAGNOSTICS_HPP_
#define ETHERCAT_INTERFACE__EC_DIAGNOSTICS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace ethercat_interface
{

/// @brief CiA 402 device-state summary a drive slave can expose for health monitoring.
///
/// Populated by the drive plugin from its already-decoded status word so that the
/// diagnostics layer stays decoupled from the CiA 402 state-machine definitions.
struct Cia402Diagnostics
{
  int device_state = 0;              //< DeviceState enum value from the drive plugin.
  std::string device_state_label;    //< Human-readable device-state string.
  uint16_t status_word = 0;          //< Raw CiA 402 status word (object 0x6041).
  bool in_fault = false;             //< True while in a fault / fault-reaction state.
};

/// @brief Health snapshot for a single EtherCAT slave.
struct SlaveDiagnostics
{
  uint16_t alias = 0;            //< Slave alias.
  uint16_t position = 0;         //< Ring position after the alias.
  uint32_t vendor_id = 0;        //< Slave vendor ID.
  uint32_t product_id = 0;       //< Slave product code.

  uint8_t al_state = 0;          //< Application-layer state (1 INIT, 2 PREOP, 4 SAFEOP, 8 OP).
  bool online = false;           //< Slave responds on the bus.
  bool operational = false;      //< Slave reached OP state.

  uint16_t al_status_code = 0;       //< ESC register 0x0134: reason a slave left OP.
  bool al_status_code_valid = false;

  int32_t dc_system_time_diff__ns = 0;   //< ESC register 0x092C: system-time difference (signed).
  bool dc_system_time_diff_valid = false;

  uint32_t dc_propagation_delay__ns = 0;  //< ESC register 0x0928: system-time transmission delay.
  bool dc_propagation_delay_valid = false;

  bool has_cia402 = false;       //< True when @ref cia402 is populated (drive slaves).
  Cia402Diagnostics cia402;
};

/// @brief Master- and bus-level EtherCAT health snapshot.
///
/// Assembled inside the cyclic loop and copied out (under lock) by the non-real-time
/// diagnostics publisher. All fields mirror values reported by the IgH master.
struct MasterDiagnostics
{
  // Master state (ecrt_master_state).
  uint32_t slaves_responding = 0;   //< Number of slaves responding on the bus.
  uint8_t al_states = 0;            //< Bitwise OR of the AL states of all slaves.
  bool link_up = false;             //< Physical link status.

  // Domain state (ecrt_domain_state).
  uint32_t working_counter = 0;     //< Last domain working counter.
  uint8_t wc_state = 0;             //< 0 ZERO, 1 INCOMPLETE, 2 COMPLETE (ec_wc_state_t).
  uint64_t incomplete_cycle_count = 0;  //< Cumulative cycles with wc_state != COMPLETE (lost-frame proxy).
  uint64_t update_count = 0;        //< Cumulative EtherCAT cycles executed.

  std::vector<SlaveDiagnostics> slaves;
};

}  // namespace ethercat_interface

#endif  // ETHERCAT_INTERFACE__EC_DIAGNOSTICS_HPP_
