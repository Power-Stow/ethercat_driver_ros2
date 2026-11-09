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

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <vector>

namespace ethercat_interface
{

/// @brief Lock @p mutex from a non-real-time thread without ever sleeping inside it.
///
/// The real-time loop only try-locks the mutexes it shares with the diagnostics thread, so it never
/// waits. Polling here too means no thread is ever queued on the mutex, so the real-time loop's
/// unlock never has a waiter to wake, which would cost it a futex syscall. The real-time loop holds
/// these mutexes for well under a microsecond per cycle, so a retry is rare.
std::unique_lock<std::mutex> lock_without_waiting(std::mutex & mutex);

/// @brief CiA 402 device-state summary a drive slave can expose for health monitoring.
///
/// Populated by the drive plugin from its already-decoded status word so that the
/// diagnostics layer stays decoupled from the CiA 402 state-machine definitions.
/// Filled from the cyclic (real-time) loop, so it holds only trivially copyable data;
/// the label is turned into a string by the non-real-time diagnostics publisher.
struct Cia402Diagnostics
{
  int device_state = 0;              //< DeviceState enum value from the drive plugin.
  /// Human-readable device-state label; must point to storage outliving the diagnostics publisher.
  const char * device_state_label = "Undefined State";
  uint16_t status_word = 0;          //< Raw CiA 402 status word (object 0x6041).
  bool in_fault = false;             //< True while in a fault / fault-reaction state.
};

/// @brief Optional interface a slave plugin implements to expose its CiA 402 device state.
///
/// Kept separate from EcSlave, so that adding it leaves the EcSlave vtable, and hence the ABI of
/// existing out-of-tree slave plugins, unchanged. The master discovers it once per slave with a
/// dynamic_cast when the slave is added.
class Cia402DiagnosticsProvider
{
public:
  virtual ~Cia402DiagnosticsProvider();

  /// @brief Current CiA 402 device-state summary.
  ///
  /// Called from the cyclic (real-time) loop, so implementations must not allocate or block.
  virtual Cia402Diagnostics cia402Diagnostics() const = 0;
};

/// @brief Optional interface a slave plugin implements to receive the SYNC0 margin of its frames.
///
/// Kept separate from EcSlave for the same reason as Cia402DiagnosticsProvider: the EcSlave
/// layout, and hence the ABI of existing out-of-tree slave plugins, stays unchanged. The master
/// discovers it once per slave with a dynamic_cast when the slave is added.
class Sync0MarginConsumer
{
public:
  virtual ~Sync0MarginConsumer();

  /// @brief Take the time from sending the frame whose inputs are read next to the following SYNC0,
  /// in ns, or -1 while it is unknown, which includes the bring-up and wind-down passes.
  ///
  /// Called from the cyclic (real-time) loop before every read and update pass, so implementations
  /// must not allocate or block.
  virtual void setSync0Margin(int64_t margin_ns) = 0;
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

  int32_t dc_system_time_diff_ns = 0;   //< ESC register 0x092C: system-time difference (signed).
  bool dc_system_time_diff_valid = false;

  uint32_t dc_propagation_delay_ns = 0;  //< ESC register 0x0928: system-time transmission delay.
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
  /// False until the cyclic loop has published its first snapshot.
  bool valid = false;

  // Master state (ecrt_master_state).
  uint32_t slaves_responding = 0;   //< Number of slaves responding on the bus.
  uint8_t al_states = 0;            //< Bitwise OR of the AL states of all slaves.
  bool link_up = false;             //< Physical link status.

  // Domain state (ecrt_domain_state).
  uint32_t working_counter = 0;     //< Last domain working counter.
  uint8_t wc_state = 0;             //< 0 ZERO, 1 INCOMPLETE, 2 COMPLETE (ec_wc_state_t).
  /// Cumulative cycles with wc_state != COMPLETE after the domain first reached COMPLETE,
  /// used as a lost-frame proxy.
  uint64_t incomplete_cycle_count = 0;
  uint64_t update_count = 0;        //< Cumulative EtherCAT cycles executed.

  std::vector<SlaveDiagnostics> slaves;
};

/// @brief Timing of the cyclic frames relative to the SYNC0 event of the DC slaves.
///
/// A DC slave latches the outputs of the last frame it received at SYNC0. The send time of a frame
/// varies from cycle to cycle, so a frame sent close to a SYNC0, on either side of it, can be
/// latched one cycle early or late. The distance to the nearest SYNC0 measures that risk, and a
/// slip is the event itself.
struct Sync0Timing
{
  /// False until the SYNC0 phase has been read back from a DC slave in OP and a frame was sent.
  bool valid = false;
  /// Time from sending the last frame to the next SYNC0, in [0, cycle).
  int64_t margin_ns = 0;
  /// Smallest distance between a frame's send and its nearest SYNC0 since the previous
  /// EcMaster::takeSync0Timing(), or -1 when no frame was sent in between.
  int64_t window_min_distance_ns = -1;
  /// Cumulative frames that did not reach the SYNC0 right after the one the previous frame reached.
  /// Each one made the slaves latch one frame's outputs at two SYNC0 events, or skip one frame's
  /// outputs entirely, whether the frame crossed a SYNC0 or a cycle sent no frame at all.
  uint64_t slip_count = 0;
  /// True when a slave is configured for distributed clocks, which makes it run SYNC0.
  bool uses_sync0 = false;
  /// Time since a DC slave reached OP while the SYNC0 phase has not been read back yet, or -1 when
  /// it has been read or no DC slave reached OP.
  int64_t phase_wait_ns = -1;
  /// False until the SYNC0 phase has been read back.
  bool phase_error_valid = false;
  /// SYNC0 phase read back minus the phase IgH configured, see sync0_phase_error_ns().
  int64_t phase_error_ns = 0;
};

/// @brief SYNC0 phase within the cycle, from a slave's next SYNC0 time (ESC register 0x0990).
///
/// A slave with a 32-bit system time implements only the low half of the register, so the value
/// is unwrapped around @p app_time_ns, which has to lie within about 2 s of it.
/// @param next_sync0_ns Register value in the slave's system time, which IgH keeps on the
/// application time.
/// @param app_time_ns A recent application time.
/// @param cycle_ns SYNC0 cycle time, greater than 0.
/// @return Phase in [0, cycle_ns).
inline int64_t sync0_phase_ns(uint64_t next_sync0_ns, uint64_t app_time_ns, uint32_t cycle_ns)
{
  const auto ahead_ns = static_cast<int32_t>(
    static_cast<uint32_t>(next_sync0_ns) - static_cast<uint32_t>(app_time_ns));
  const uint64_t unwrapped_ns = app_time_ns + static_cast<uint64_t>(static_cast<int64_t>(ahead_ns));
  return static_cast<int64_t>(unwrapped_ns % cycle_ns);
}

/// @brief SYNC0 phase read back from a slave minus the phase IgH configured.
///
/// IgH starts SYNC0 at the DC reference time, the first application time it received, plus whole
/// cycles plus the SYNC0 shift, so a slave that follows its configuration has the phase
/// (reference time + shift) mod cycle and a difference close to 0.
/// @param phase_ns SYNC0 phase from sync0_phase_ns().
/// @param dc_reference_time_ns First application time passed to IgH.
/// @param shift_ns SYNC0 shift the DC slaves were configured with.
/// @param cycle_ns SYNC0 cycle time, greater than 0.
/// @return Difference in [-cycle_ns / 2, cycle_ns / 2).
inline int64_t sync0_phase_error_ns(
  int64_t phase_ns, uint64_t dc_reference_time_ns, int32_t shift_ns, uint32_t cycle_ns)
{
  const auto cycle = static_cast<int64_t>(cycle_ns);
  const int64_t configured_phase_ns =
    static_cast<int64_t>(dc_reference_time_ns % cycle_ns) + shift_ns;
  const int64_t error_ns = (phase_ns - configured_phase_ns) % cycle;
  if (error_ns < -cycle / 2) {
    return error_ns + cycle;
  }
  return (error_ns >= cycle - cycle / 2) ? error_ns - cycle : error_ns;
}

/// @brief Time from sending a frame to the next SYNC0.
/// @param send_time_ns Time the frame was sent, in CLOCK_MONOTONIC like the application time.
/// @param phase_ns SYNC0 phase from sync0_phase_ns().
/// @param cycle_ns SYNC0 cycle time, greater than 0.
/// @return Margin in [0, cycle_ns).
inline int64_t sync0_margin_ns(uint64_t send_time_ns, int64_t phase_ns, uint32_t cycle_ns)
{
  const int64_t margin_ns = phase_ns - static_cast<int64_t>(send_time_ns % cycle_ns);
  return (margin_ns < 0) ? margin_ns + static_cast<int64_t>(cycle_ns) : margin_ns;
}

/// @brief Distance from a frame's send to the nearest SYNC0, before or after it.
inline int64_t sync0_distance_ns(int64_t margin_ns, uint32_t cycle_ns)
{
  return std::min(margin_ns, static_cast<int64_t>(cycle_ns) - margin_ns);
}

/// @brief Start of the first cycle after @p now_ns on the grid `anchor_ns + n * cycle_ns`.
/// @param cycle_ns Cycle time, greater than 0.
/// @return The next grid point, or @p anchor_ns itself while it lies in the future.
inline uint64_t next_cycle_start_ns(uint64_t now_ns, uint64_t anchor_ns, uint32_t cycle_ns)
{
  if (now_ns < anchor_ns) {
    return anchor_ns;
  }
  return anchor_ns + ((now_ns - anchor_ns) / cycle_ns + 1) * cycle_ns;
}

/// @brief Index of the SYNC0 that latches a frame sent at @p send_time_ns, counted from time 0.
///
/// Consecutive frames reach consecutive SYNC0 events. Any other step means a SYNC0 latched one
/// frame twice or a frame was never latched, which a margin alone cannot tell after a cycle that
/// sent no frame.
inline uint64_t sync0_index(uint64_t send_time_ns, int64_t phase_ns, uint32_t cycle_ns)
{
  const int64_t margin_ns = sync0_margin_ns(send_time_ns, phase_ns, cycle_ns);
  return (send_time_ns + static_cast<uint64_t>(margin_ns)) / cycle_ns;
}

}  // namespace ethercat_interface

#endif  // ETHERCAT_INTERFACE__EC_DIAGNOSTICS_HPP_
