// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_INTERFACE__EC_MASTER_HPP_
#define ETHERCAT_INTERFACE__EC_MASTER_HPP_

#include <ecrt.h>

#include <time.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include "ethercat_interface/ec_diagnostics.hpp"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_transfer.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ethercat_interface
{

inline uint64_t EC_NEWTIMEVAL2NANO(struct timespec & TV)
{
  return (TV.tv_sec - 946684800ULL) * 1000000000ULL + TV.tv_nsec;
}

inline uint64_t EC_MONOTONIC2NANO(const struct timespec & TV)
{
  return static_cast<uint64_t>(TV.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(TV.tv_nsec);
}

class EcMemoryEntry
{
public:
  std::string module_name;   //< Module.
  uint16_t alias;            //< Slave alias.
  uint16_t position;         //< Slave position.
  uint16_t index;            //< Channel index.
  uint16_t subindex;         //< Channel subindex.

public:
  inline
  std::string to_simple_string() const
  {
    return "( name= " + module_name + ", index= " +
           std::to_string(index) + ", subindex= " + std::to_string(subindex) + " )";
  }
};

class EcTransferEntry
{
public:
  EcMemoryEntry input;
  EcMemoryEntry output;
  size_t size;   //< Size of the exchange data.

public:
  inline
  std::string to_simple_string() const
  {
    return input.to_simple_string() + " -> " + output.to_simple_string() + " /  ( size= " +
           std::to_string(size) + " )";
  }
};

class EcTransferNet
{
public:
  std::string name;                    //< transfer net name (e.g. a safety net)
  std::string master;                  //< transfer master (e.g. the safety master of the net)
  std::vector<EcTransferEntry> transfers;   //< Data transfers (e.g. safety data transfers)

public:
  void reset(const std::string & new_name)
  {
    name = new_name;
    master = "";
    transfers.clear();
  }
};

// Forward declarations
class EcSlave;

/** Data for a single domain */
struct DomainInfo
{
  explicit DomainInfo(ec_master_t * master);
  ~DomainInfo();

  ec_domain_t * domain = NULL;
  ec_domain_state_t domain_state = {};
  uint8_t * domain_pd = NULL;  //< pointer to process domain data

  /** domain pdo registration array.
   *  do not modify after active(), or may invalidate */
  std::vector<ec_pdo_entry_reg_t> domain_regs;

  /** slave's pdo and in memory data entries in the domain */
  struct Entry
  {
    EcSlave * slave = NULL;
    int num_pdos = 0;
    uint32_t * offset = NULL;
    uint32_t * bit_position = NULL;
    int num_in_memory_data = 0;
    uint32_t * offset_in_memory = NULL;
  };

  std::vector<Entry> entries;
};

class EcMaster
{
public:
  explicit EcMaster(const unsigned int master = 0);
  virtual ~EcMaster();

  inline bool isValid() const {return master_ != nullptr;}

  /** \brief add a slave device to the master
    * alias and position can be found by running the following command
    * /opt/etherlab/bin$ sudo ./ethercat slaves
    * look for the "A B:C STATUS DEVICE" (e.g. B=alias, C=position)
    */
  void addSlave(uint16_t alias, uint16_t position, EcSlave * slave);

  /** \brief add a slave device to the master
    * alias and position should have been set
    * before calling this function.
    */
  void addSlave(EcSlave * slave);

  /** \brief configure slave using SDO
    */
  int configSlaveSdo(uint16_t slave_position, SdoConfigEntry sdo_config, uint32_t * abort_code);

  /** call after adding all slaves, and before update */
  bool activate();

  /** perform one EtherCAT cycle, passing the domain to the slaves */
  virtual void update(uint32_t domain = 0);

  /** run a control loop of update() and user_callback(), blocking.
   *  call activate and setThreadHighPriority/RealTime first. */
  typedef void (* SIMPLECAT_CONTRL_CALLBACK)(void);
  virtual void run(SIMPLECAT_CONTRL_CALLBACK user_callback);

  /** stop the control loop. use within callback, or from a separate thread. */
  virtual void stop() {running_ = false;}

  /** stop and release the IgH master. Do not call from realtime context. */
  virtual void shutdown();

  /** time of last ethercat update, since calling run. stops if stop called.
   *  returns actual time. use elapsedCycles()/frequency for discrete time at last update. */
  virtual double elapsedTime();

  /** number of EtherCAT updates since calling run. */
  virtual uint64_t elapsedCycles();

  /** add ctr-c exit callback.
    * default exits the run loop and prints timing */
  typedef void (* SIMPLECAT_EXIT_CALLBACK)(int);
  static void setCtrlCHandler(SIMPLECAT_EXIT_CALLBACK user_callback = NULL);

  /** set the thread to a priority of -19
   *  priority range is -20 (highest) to 19 (lowest) */
  static void setThreadHighPriority();

  /** set the thread to real time (FIFO)
   *  thread cannot be preempted.
   *  set priority as 49 (kernel and interrupts are 50) */
  static void setThreadRealTime();

  void setCtrlFrequency(double frequency)
  {
    if (frequency <= 0.0) {
      throw std::runtime_error("EtherCAT control frequency must be > 0");
    }
    interval_ = static_cast<uint32_t>(1000000000.0 / frequency);
  }

  /** SYNC0 shift in ns. Start with 0, or set this to the value used by your C program. */
  void setDcSync0Shift(int32_t shift_time_ns) {dc_sync0_shift_ns_ = shift_time_ns;}

  uint32_t getInterval() {return interval_;}

  virtual void readData(uint32_t domain = 0);
  virtual void writeData(uint32_t domain = 0);

  /** Enable EtherCAT health-diagnostics collection. Must be called before activate() so
   *  per-slave ESC register requests can be created. Disabled by default (no overhead). */
  void setDiagnosticsEnabled(bool enabled) {diagnostics_enabled_ = enabled;}

  /** @brief Thread-safe copy of the latest EtherCAT health snapshot.
   *  Intended for a non-real-time publisher; safe to call concurrently with the cyclic loop. */
  MasterDiagnostics getDiagnostics() const;

  /** @brief Fill in the EcTransferInfo structures
  *
  * @param transfer_nets transfer nets
  *
  * \pre DomainInfo and domain_regs vectors must have been initialized and
  * activated. A call to EcMaster::activate() is required before calling
  * this function, to fill in the domain_regs vector offsets. Specifically
  * with IgH EtherCAT Master, the offset must have been initialized with the
  * ecrt_domain_reg_pdo_entry_list function.
  *
  * @throw std::runtime_error if some domain_info or some pdo_entry_reg are
  *  not valid
  */
  void registerTransferInDomain(const std::vector<EcTransferNet> & transfer_nets);

  /** @brief Proceed to the transfer of all the data declared in transfers_.
   */
  void transferAll();

protected:
  /** @brief Output the memory content of the all the domains
   * (available for pedagogic and debug purposes)
   *
   * @param[out] os Output stream
  */
  void printMemoryFrames(std::ostream & os);

  /** @brief Get pointer on memory frame for a certain point
   * in the frame defined by a slave position, an index and a subindex
   */
  uint8_t * getMemoryStart(
    const uint16_t position,
    const uint16_t index,
    const uint16_t subindex);

  /** @brief Output memory n bytes of memory from a certain point in
   * the frame defined by a slave position, an index, subindex */
  void printMemoryFrame(
    const uint16_t position,
    const uint16_t index,
    const uint16_t subindex,
    const size_t n,
    bool binary = false,
    std::ostream & os = std::cout);

protected:
  /** true if running */
  volatile bool running_ = false;

  /** start and current time */
  std::chrono::time_point<std::chrono::system_clock> start_t_, curr_t_;

  // EtherCAT Control

  /** register a domain of the slave */
  void registerPDOInDomain(
    std::vector<uint32_t> & channel_indices,
    DomainInfo * domain_info,
    EcSlave * slave);

  /** check for change in the domain state */
  void checkDomainState(uint32_t domain);

  /** check for change in the master state */
  void checkMasterState();

  /** check for change in the slave states */
  void checkSlaveStates();

  /** create the per-slave ESC register requests used for diagnostics (call before activate) */
  void createRegisterRequests();

  /** service the per-slave register requests at a low cadence from the cyclic loop */
  void serviceRegisterRequests();

  /** copy the current master/domain/slave state into the diagnostics snapshot */
  void updateDiagnosticsSnapshot(uint32_t domain);

  /** call IgH DC sync functions from a consistent point in the cycle */
  void syncDistributedClocks();

  /** monotonic application time used for IgH DC synchronization */
  static uint64_t applicationTimeNs();

  /** print warning message to terminal */
  inline
  static void printWarning(const std::string & message)
  {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "WARNING. Master. %s", message.c_str());
  }


  /** @brief Check the validity of the domain info and the ec_pdo_entry_reg_t
   * and throw an exception if not valid.
   *
   * @param domain_info Domain info
   * @param pdo_entry_reg PDO entry registration
   *
   * @throw std::runtime_error if domain_info or pdo_entry_reg is not valid
  */
  void checkDomainInfoValidity(
    const DomainInfo & domain_info,
    const ec_pdo_entry_reg_t & pdo_entry_reg);

  /** EtherCAT master data */
  ec_master_t * master_ = NULL;
  ec_master_state_t master_state_ = {};

  /** map from domain index to domain info */
  std::map<uint32_t, DomainInfo *> domain_info_;

  /** data needed to check slave state */
  struct SlaveInfo
  {
    EcSlave * slave = NULL;
    ec_slave_config_t * config = NULL;
    ec_slave_config_state_t config_state = {0, 0, 0};

    // Diagnostics: per-slave ESC register requests and their latest decoded values.
    ec_reg_request_t * al_status_reg = NULL;    //< ESC register 0x0134 request.
    ec_reg_request_t * dc_time_diff_reg = NULL;  //< ESC register 0x092C request.
    ec_reg_request_t * dc_delay_reg = NULL;      //< ESC register 0x0928 request.

    uint16_t al_status_code = 0;
    bool al_status_code_valid = false;
    int32_t dc_system_time_diff_ns = 0;
    bool dc_system_time_diff_valid = false;
    uint32_t dc_propagation_delay_ns = 0;
    bool dc_propagation_delay_valid = false;
  };

  std::vector<SlaveInfo> slave_info_;

  /** counter of control loops */
  uint64_t update_counter_ = 0;

  /** frequency to check for master or slave state change.
   *  state checked every frequency_ control loops */
  uint32_t check_state_frequency_ = 10;

  /** whether health-diagnostics collection is active (set before activate()) */
  bool diagnostics_enabled_ = false;

  /** cumulative cycles with an incomplete domain working counter (lost-frame proxy) */
  std::atomic<uint64_t> incomplete_cycle_count_{0};

  /** guards diagnostics_ against concurrent access by the non-real-time publisher */
  mutable std::mutex diagnostics_mutex_;

  /** latest health snapshot, published under diagnostics_mutex_ */
  MasterDiagnostics diagnostics_;

  /** reusable scratch buffer so snapshot assembly is allocation-free in steady state */
  MasterDiagnostics diagnostics_scratch_;

  uint32_t interval_;

  int32_t dc_sync0_shift_ns_ = 0;

  /** Data transfers (necessary for transfer communication) */
  std::vector<EcTransferInfo> transfers_;

protected:
  friend struct DomainInfo;
  friend struct EcTransferInfo;
};

}  // namespace ethercat_interface

#endif  // ETHERCAT_INTERFACE__EC_MASTER_HPP_
