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

#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_slave.hpp"

#include <unistd.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <bitset>
#include <cstring>
#include <utility>

namespace ethercat_interface
{

DomainInfo::DomainInfo(ec_master_t * master)
{
  domain = ecrt_master_create_domain(master);
  if (domain == NULL) {
    EcMaster::printWarning("Failed to create domain");
    return;
  }

  const ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_regs.push_back(empty);
}


DomainInfo::~DomainInfo()
{
  for (Entry & entry : entries) {
    delete[] entry.offset;
    delete[] entry.bit_position;
  }
}


EcMaster::EcMaster(const unsigned int master)
{
  master_ = ecrt_request_master(master);
  if (master_ == NULL) {
    printWarning("Failed to obtain master.");
    return;
  }
  interval_ = 0;
}

EcMaster::~EcMaster()
{
  /*
  for (SlaveInfo & slave : slave_info_) {
    //TODO verify what this piece of code was here for
  }
  */
  shutdown();
}

void EcMaster::shutdown()
{
  std::cout << "[EcMaster] shutdown(): begin" << std::endl;

  running_ = false;

  for (auto & domain : domain_info_) {
    if (domain.second != NULL) {
      delete domain.second;
    }
  }

  domain_info_.clear();
  slave_info_.clear();
  transfers_.clear();

  if (master_ != NULL) {
    std::cout << "[EcMaster] shutdown(): releasing IgH master" << std::endl;
    ecrt_release_master(master_);
    master_ = NULL;
    std::cout << "[EcMaster] shutdown(): IgH master released" << std::endl;
  } else {
    std::cout << "[EcMaster] shutdown(): master already null" << std::endl;
  }

  std::cout << "[EcMaster] shutdown(): done" << std::endl;
}

uint64_t EcMaster::applicationTimeNs()
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return EC_MONOTONIC2NANO(t);
}

void EcMaster::syncDistributedClocks()
{
  ecrt_master_application_time(master_, applicationTimeNs());
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);
}

void EcMaster::addSlave(uint16_t alias, uint16_t position, EcSlave * slave)
{
  slave->setAliasAndPosition(alias, position);
  addSlave(slave);
}

void EcMaster::addSlave(EcSlave * slave)
{
  if (master_ == nullptr) {
    printWarning("Add slave. Master is not available.");
    return;
  }

  if (false == slave->isAliasAndPositionSet()) {
    std::string error_message = "Alias and position not set for slave (vendor id=" + std::to_string(
      slave->vendor_id_) + ",product_code=" + std::to_string(slave->product_id_) + ").";
    throw std::runtime_error(error_message);
  }

  // configure slave in master
  SlaveInfo slave_info;
  slave_info.slave = slave;
  slave_info.config = ecrt_master_slave_config(
    master_,
    slave->alias_, slave->position_,
    slave->vendor_id_, slave->product_id_);
  if (slave_info.config == NULL) {
    printWarning("Add slave. Failed to get slave configuration.");
    return;
  }

  // check and setup dc

  if (slave->assign_activate_dc_sync()) {
    if (interval_ == 0) {
      throw std::runtime_error(
        "EtherCAT DC requested, but control frequency/interval is not configured");
    }

    ecrt_master_application_time(master_, applicationTimeNs());

    ecrt_slave_config_dc(
      slave_info.config,
      slave->assign_activate_dc_sync(),
      interval_,
      dc_sync0_shift_ns_,
      0,
      0);
  }

  slave_info_.push_back(slave_info);

  // Setup PDOs registered by the slave.
  // For each slave, PDOs are grouped by sync manager.
  // For each active sync manager of the slave,
  // register the associated set of PDOs.
  size_t num_syncs = slave->syncSize();
  const ec_sync_info_t * syncs = slave->syncs();
  if (num_syncs > 0) {
    // configure pdos in slave
    int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
    if (pdos_status) {
      printWarning("Add slave. Failed to configure PDOs");
      return;
    }
  } else {
    printWarning(
      "Add slave. Sync size is zero for " +
      std::to_string(slave->alias_) + ":" + std::to_string(slave->position_));
  }

  // Get all domains and associated pdos that the slave registers
  EcSlave::DomainMap domain_map;
  slave->domains(domain_map);
  for (auto & iter : domain_map) {
    // get the domain info, create if necessary
    uint32_t domain_index = iter.first;
    DomainInfo * domain = NULL;
    if (domain_info_.count(domain_index)) {
      domain = domain_info_.at(domain_index);
    }
    if (domain == NULL) {
      domain = new DomainInfo(master_);
      domain_info_[domain_index] = domain;
    }

    registerPDOInDomain(
      iter.second, domain,
      slave);
  }
}

int EcMaster::configSlaveSdo(
  uint16_t slave_position, SdoConfigEntry sdo_config,
  uint32_t * abort_code)
{
  if (master_ == nullptr) {
    printWarning("Config SDO. Master is not available.");
    return -1;
  }

  uint8_t buffer[8];
  sdo_config.buffer_write(buffer);
  int ret = ecrt_master_sdo_download(
    master_,
    slave_position,
    sdo_config.index,
    sdo_config.sub_index,
    buffer,
    sdo_config.data_size(),
    abort_code
  );
  return ret;
}

void EcMaster::registerPDOInDomain(
  std::vector<uint32_t> & channel_indices,
  DomainInfo * domain_info,
  EcSlave * slave)
{
  // expand the size of the domain
  uint32_t num_pdo_regs = channel_indices.size();
  size_t start_index = domain_info->domain_regs.size() - 1;  // empty element at end
  domain_info->domain_regs.resize(domain_info->domain_regs.size() + num_pdo_regs);

  // create a new entry in the domain
  DomainInfo::Entry domain_entry;
  domain_entry.slave = slave;
  domain_entry.num_pdos = num_pdo_regs;
  domain_entry.offset = new uint32_t[num_pdo_regs];
  domain_entry.bit_position = new uint32_t[num_pdo_regs];
  domain_info->entries.push_back(domain_entry);

  EcSlave::DomainMap domain_map;
  slave->domains(domain_map);

  // add to array of pdos registrations
  const ec_pdo_entry_info_t * pdo_regs = slave->channels();
  for (size_t i = 0; i < num_pdo_regs; ++i) {
    // create pdo entry in the domain
    ec_pdo_entry_reg_t & pdo_reg = domain_info->domain_regs[start_index + i];
    pdo_reg.alias = slave->alias_;
    pdo_reg.position = slave->position_;
    pdo_reg.vendor_id = slave->vendor_id_;
    pdo_reg.product_code = slave->product_id_;
    pdo_reg.index = pdo_regs[channel_indices[i]].index;
    pdo_reg.subindex = pdo_regs[channel_indices[i]].subindex;
    pdo_reg.offset = &(domain_entry.offset[i]);
    pdo_reg.bit_position = &(domain_entry.bit_position[i]);


    // print the domain pdo entry
    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "{ %d, %d, 0x%x, 0x%x, 0x%x, 0x%x }",
      pdo_reg.alias,
      pdo_reg.position,
      pdo_reg.vendor_id,
      pdo_reg.product_code,
      pdo_reg.index,
      static_cast<int>(pdo_reg.subindex)
    );
  }

  // set the last element to null
  ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_info->domain_regs.back() = empty;
}

bool EcMaster::activate()
{
  if (master_ == nullptr) {
    printWarning("Activate. Master is not available.");
    return false;
  }

  // Register requests must be created before ecrt_master_activate().
  if (diagnostics_enabled_) {
    createRegisterRequests();
  }

  // register domain
  for (auto & iter : domain_info_) {
    DomainInfo * domain_info = iter.second;
    if (domain_info == NULL) {
      throw std::runtime_error("Null domain info: " + std::to_string(iter.first));
    }
    bool domain_status = ecrt_domain_reg_pdo_entry_list(
      domain_info->domain,
      &(domain_info->domain_regs[0]));
    if (domain_status) {
      printWarning("Activate. Failed to register domain PDO entries.");
      return false;
    }
  }
  // set application time
  ecrt_master_application_time(master_, applicationTimeNs());

  // activate master
  bool activate_status = ecrt_master_activate(master_);
  if (activate_status) {
    printWarning("Activate. Failed to activate master.");
    return false;
  }

  // retrieve domain data
  for (auto & iter : domain_info_) {
    DomainInfo * domain_info = iter.second;
    if (domain_info == NULL) {
      throw std::runtime_error("Null domain info: " + std::to_string(iter.first));
    }
    domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
    if (domain_info->domain_pd == NULL) {
      printWarning("Activate. Failed to retrieve domain process data.");
      return false;
    }
  }
  return true;
}

void EcMaster::update(uint32_t domain)
{
  if (master_ == nullptr) {
    return;
  }

  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // Transfer data if configured
  // TODO(@yguel) make transfer per domain ? Quid of transfers across domains ?
  transferAll();

  // check process data state (optional)
  checkDomainState(domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // refresh the health snapshot (cheap; no-op unless diagnostics are enabled)
  if (diagnostics_enabled_ && update_counter_ % check_state_frequency_ == 0) {
    // Register requests self-pace via their own BUSY state, so servicing them at the snapshot
    // cadence (rather than a slow fixed gate) keeps register-derived values, such as the DC
    // system-time difference, refreshing promptly instead of lagging by hundreds of cycles.
    serviceRegisterRequests();
    updateDiagnosticsSnapshot(domain);
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    entry.slave->set_process_phase("update");
    for (int i = 0; i < entry.num_pdos; ++i) {
      (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
    }
  }

  syncDistributedClocks();

  // send process data
  ecrt_domain_queue(domain_info->domain);
  ecrt_master_send(master_);

  ++update_counter_;
}

void EcMaster::readData(uint32_t domain)
{
  if (master_ == nullptr) {
    return;
  }

  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // Transfer data if configured
  // TODO(@yguel) make transfer per domain ? Quid of transfers across domains ?
  transferAll();

  // check process data state (optional)
  checkDomainState(domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // refresh the health snapshot (cheap; no-op unless diagnostics are enabled)
  if (diagnostics_enabled_ && update_counter_ % check_state_frequency_ == 0) {
    // Register requests self-pace via their own BUSY state, so servicing them at the snapshot
    // cadence (rather than a slow fixed gate) keeps register-derived values, such as the DC
    // system-time difference, refreshing promptly instead of lagging by hundreds of cycles.
    serviceRegisterRequests();
    updateDiagnosticsSnapshot(domain);
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    entry.slave->set_process_phase("read");
    for (int i = 0; i < entry.num_pdos; ++i) {
      (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
    }
  }

  ++update_counter_;
}

void EcMaster::writeData(uint32_t domain)
{
  if (master_ == nullptr) {
    return;
  }

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    entry.slave->set_process_phase("write");
    for (int i = 0; i < entry.num_pdos; ++i) {
      (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
    }
  }

  syncDistributedClocks();

  // send process data
  ecrt_domain_queue(domain_info->domain);
  ecrt_master_send(master_);
}

void EcMaster::setCtrlCHandler(SIMPLECAT_EXIT_CALLBACK user_callback)
{
  // ctrl c handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = user_callback;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

void EcMaster::run(SIMPLECAT_CONTRL_CALLBACK user_callback)
{
  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  running_ = true;
  start_t_ = std::chrono::system_clock::now();
  while (running_) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    // update EtherCAT bus
    this->update();

    // get actual time
    curr_t_ = std::chrono::system_clock::now();

    // user callback
    user_callback();

    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += interval_;
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }
}

double EcMaster::elapsedTime()
{
  std::chrono::duration<double> elapsed_seconds = curr_t_ - start_t_;
  return elapsed_seconds.count() - 1.0;  // started after 1 second
}

uint64_t EcMaster::elapsedCycles()
{
  return update_counter_;
}

void EcMaster::setThreadHighPriority()
{
  pid_t pid = getpid();
  int priority_status = setpriority(PRIO_PROCESS, pid, -19);
  if (priority_status) {
    printWarning("setThreadHighPriority. Failed to set priority.");
    return;
  }
}

void EcMaster::setThreadRealTime()
{
  /* Declare ourself as a real time task, priority 49.
      PRREMPT_RT uses priority 50
      for kernel tasklets and interrupt handler by default */
  struct sched_param param;
  param.sched_priority = 49;
  // pthread_t this_thread = pthread_self();
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed");
    exit(-1);
  }

  /* Lock memory */
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }

  /* Pre-fault our stack
      8*1024 is the maximum stack size
      which is guaranteed safe to access without faulting */
  constexpr unsigned int MAX_SAFE_STACK = 8 * 1024;
  unsigned char dummy[MAX_SAFE_STACK];
  memset(dummy, 0, MAX_SAFE_STACK);
}

void EcMaster::checkDomainState(uint32_t domain)
{
  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ec_domain_state_t ds;
  ecrt_domain_state(domain_info->domain, &ds);

  if (ds.working_counter != domain_info->domain_state.working_counter) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Domain: WC %d.", ds.working_counter);
  }
  if (ds.wc_state != domain_info->domain_state.wc_state) {
    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "Domain: State %s.",
      ds.wc_state == EC_WC_ZERO ? "ZERO" :
      (
        (ds.wc_state == EC_WC_INCOMPLETE) ? "INCOMPLETE" :
        (ds.wc_state == EC_WC_COMPLETE) ? "COMPLETE" : "UNKNOWN"
      )
    );
  }
  // Track incomplete cycles as a lost-frame proxy (the ecrt realtime API does not expose
  // tx-error / lost-frame counters directly).
  if (diagnostics_enabled_ && ds.wc_state != EC_WC_COMPLETE) {
    incomplete_cycle_count_.fetch_add(1, std::memory_order_relaxed);
  }
  domain_info->domain_state = ds;
}


void EcMaster::checkMasterState()
{
  if (master_ == nullptr) {
    return;
  }

  ec_master_state_t ms;
  ecrt_master_state(master_, &ms);

  if (ms.slaves_responding != master_state_.slaves_responding) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "%d slave(s).", ms.slaves_responding);
  }
  if (ms.al_states != master_state_.al_states) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Master AL states: 0x%02X.", ms.al_states);
  }
  if (ms.link_up != master_state_.link_up) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Link is %s.", ms.link_up ? "up" : "down");
  }
  master_state_ = ms;
}


void EcMaster::checkSlaveStates()
{
  for (SlaveInfo & slave : slave_info_) {
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slave.config, &s);

    if (s.al_state != slave.config_state.al_state) {
      // this spams the terminal at initialization.
      RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Slave: State 0x%02X.", s.al_state);
    }
    if (s.online != slave.config_state.online) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "EthercatDriver"), "Slave: %s.", s.online ? "online" : "offline");
    }
    if (s.operational != slave.config_state.operational) {
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Slave: (alias: %d, pos: %d, vendor_id: %d, prod_id: %d) --> %soperational.",
        slave.slave->alias_,
        slave.slave->position_,
        slave.slave->vendor_id_,
        slave.slave->product_id_,
        s.operational ? "" : "NOT ");
      slave.slave->set_state_is_operational(s.operational ? true : false);
    }
    slave.config_state = s;
  }
}

void EcMaster::createRegisterRequests()
{
  for (SlaveInfo & slave : slave_info_) {
    if (slave.config == nullptr) {
      continue;
    }
    slave.al_status_reg = ecrt_slave_config_create_reg_request(slave.config, 2);
    slave.dc_time_diff_reg = ecrt_slave_config_create_reg_request(slave.config, 4);
    slave.dc_delay_reg = ecrt_slave_config_create_reg_request(slave.config, 4);
    if (slave.al_status_reg == nullptr || slave.dc_time_diff_reg == nullptr ||
      slave.dc_delay_reg == nullptr)
    {
      printWarning("Diagnostics: failed to create a register request for a slave.");
    }
  }
}

void EcMaster::serviceRegisterRequests()
{
  for (SlaveInfo & slave : slave_info_) {
    // AL status code (ESC register 0x0134, 2 bytes): reason a slave left OP.
    if (slave.al_status_reg != nullptr) {
      const ec_request_state_t state = ecrt_reg_request_state(slave.al_status_reg);
      if (state == EC_REQUEST_SUCCESS) {
        slave.al_status_code = EC_READ_U16(ecrt_reg_request_data(slave.al_status_reg));
        slave.al_status_code_valid = true;
        ecrt_reg_request_read(slave.al_status_reg, 0x0134, 2);
      } else if (state != EC_REQUEST_BUSY) {
        ecrt_reg_request_read(slave.al_status_reg, 0x0134, 2);
      }
    }

    // DC system time difference (ESC register 0x092C, 4 bytes): bit 31 is the sign.
    if (slave.dc_time_diff_reg != nullptr) {
      const ec_request_state_t state = ecrt_reg_request_state(slave.dc_time_diff_reg);
      if (state == EC_REQUEST_SUCCESS) {
        const uint32_t raw = EC_READ_U32(ecrt_reg_request_data(slave.dc_time_diff_reg));
        const int32_t magnitude = static_cast<int32_t>(raw & 0x7FFFFFFFu);
        slave.dc_system_time_diff_ns = (raw & 0x80000000u) ? -magnitude : magnitude;
        slave.dc_system_time_diff_valid = true;
        ecrt_reg_request_read(slave.dc_time_diff_reg, 0x092C, 4);
      } else if (state != EC_REQUEST_BUSY) {
        ecrt_reg_request_read(slave.dc_time_diff_reg, 0x092C, 4);
      }
    }

    // DC propagation delay (ESC register 0x0928, 4 bytes): static after DC init, read once.
    if (slave.dc_delay_reg != nullptr && !slave.dc_propagation_delay_valid) {
      const ec_request_state_t state = ecrt_reg_request_state(slave.dc_delay_reg);
      if (state == EC_REQUEST_SUCCESS) {
        slave.dc_propagation_delay_ns = EC_READ_U32(ecrt_reg_request_data(slave.dc_delay_reg));
        slave.dc_propagation_delay_valid = true;
      } else if (state != EC_REQUEST_BUSY) {
        ecrt_reg_request_read(slave.dc_delay_reg, 0x0928, 4);
      }
    }
  }
}

void EcMaster::updateDiagnosticsSnapshot(uint32_t domain)
{
  // Assemble into a reusable scratch buffer so that, once warmed up, this does not allocate
  // (the vector keeps its capacity and the small strings reuse their buffers). Only an O(1)
  // swap happens under the lock, keeping the real-time critical section minimal.
  MasterDiagnostics & snapshot = diagnostics_scratch_;
  snapshot.slaves_responding = master_state_.slaves_responding;
  snapshot.al_states = master_state_.al_states;
  snapshot.link_up = master_state_.link_up != 0;
  snapshot.working_counter = 0;
  snapshot.wc_state = 0;

  auto domain_it = domain_info_.find(domain);
  if (domain_it != domain_info_.end() && domain_it->second != nullptr) {
    snapshot.working_counter = domain_it->second->domain_state.working_counter;
    snapshot.wc_state = static_cast<uint8_t>(domain_it->second->domain_state.wc_state);
  }
  snapshot.incomplete_cycle_count = incomplete_cycle_count_.load(std::memory_order_relaxed);
  snapshot.update_count = update_counter_;

  snapshot.slaves.resize(slave_info_.size());
  for (size_t i = 0; i < slave_info_.size(); ++i) {
    SlaveInfo & slave = slave_info_[i];
    SlaveDiagnostics & sd = snapshot.slaves[i];
    if (slave.slave != nullptr) {
      sd.alias = slave.slave->alias_;
      sd.position = slave.slave->position_;
      sd.vendor_id = slave.slave->vendor_id_;
      sd.product_id = slave.slave->product_id_;
    }
    sd.al_state = static_cast<uint8_t>(slave.config_state.al_state);
    sd.online = slave.config_state.online != 0;
    sd.operational = slave.config_state.operational != 0;
    sd.al_status_code = slave.al_status_code;
    sd.al_status_code_valid = slave.al_status_code_valid;
    sd.dc_system_time_diff__ns = slave.dc_system_time_diff_ns;
    sd.dc_system_time_diff_valid = slave.dc_system_time_diff_valid;
    sd.dc_propagation_delay__ns = slave.dc_propagation_delay_ns;
    sd.dc_propagation_delay_valid = slave.dc_propagation_delay_valid;
    sd.has_cia402 = false;
    if (slave.slave != nullptr) {
      const auto cia402 = slave.slave->cia402Diagnostics();
      if (cia402.has_value()) {
        sd.has_cia402 = true;
        sd.cia402 = cia402.value();
      }
    }
  }

  // Publish with a non-blocking try-lock so the cyclic (real-time) loop is never delayed by the
  // diagnostics reader. If the lock is momentarily held by getDiagnostics(), skip publishing this
  // round; the freshly built scratch is simply published on the next snapshot (a few cycles later).
  std::unique_lock<std::mutex> lock(diagnostics_mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::swap(diagnostics_, diagnostics_scratch_);
  }
}

MasterDiagnostics EcMaster::getDiagnostics() const
{
  const std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  return diagnostics_;
}

void EcMaster::checkDomainInfoValidity(
  const DomainInfo & domain_info,
  const ec_pdo_entry_reg_t & pdo_entry_reg)
{
  if (nullptr == domain_info.domain_pd) {
    throw std::runtime_error("Domain process data pointer not set.");
  }
  if (nullptr == pdo_entry_reg.offset) {
    throw std::runtime_error("Offset not set in pdo_entry_reg.");
  }
}

void EcMaster::registerTransferInDomain(const std::vector<EcTransferNet> & transfer_nets)
{
  // Fill in the EcTransferInfo structures

  // For each transfer of each net,
  for (auto & net : transfer_nets) {
    for (auto & transfer : net.transfers) {
      EcTransferInfo transfer_info;
      transfer_info.size = transfer.size;
      RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer size: %ld", transfer.size);
      /**
       * For the input and the output of the transfer find
       *   1. the process domain data pointer
       *   2. the offset in the process domain data
       * By iterating over the existing DomainInfo and domain_regs vector
       * to find the ec_pdo_entry_reg_t whose alias, position, index and subindex
       * match the transfer input and output memory entries
       */
      for (const auto & key_val : domain_info_) {
        const DomainInfo & domain = *(key_val.second);
        for (auto & domain_reg : domain.domain_regs) {
          // Find match for input
          if (domain_reg.alias == transfer.input.alias &&
            domain_reg.position == transfer.input.position &&
            domain_reg.index == transfer.input.index &&
            domain_reg.subindex == transfer.input.subindex)
          {
            transfer_info.input_domain = &domain;
            // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.in_ptr = domain.domain_pd + *(domain_reg.offset);
            RCLCPP_INFO(
              rclcpp::get_logger("EthercatDriver"),
              "Transfer input:  esclave position: %d / index: 0x%x / in offset:  %d",
              domain_reg.position,
              domain_reg.index,
              *(domain_reg.offset)
            );
          }
          // Find match for output
          if (domain_reg.alias == transfer.output.alias &&
            domain_reg.position == transfer.output.position &&
            domain_reg.index == transfer.output.index &&
            domain_reg.subindex == transfer.output.subindex)
          {
            transfer_info.output_domain = &domain;
            // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.out_ptr = domain.domain_pd + *(domain_reg.offset);
            RCLCPP_INFO(
              rclcpp::get_logger("EthercatDriver"),
              "Transfer output: slave position: %d / index: 0x%x / out offset: %d",
              domain_reg.position,
              domain_reg.index,
              *(domain_reg.offset)
            );
          }
        }
      }

      // Record the transfer
      transfers_.push_back(transfer_info);
    }
  }
}

void EcMaster::transferAll()
{
  // Proceed to the transfer of all the data declared in transfers_.
  for (auto & transfer : transfers_) {
    // Copy the data from the input to the output
    memcpy(transfer.out_ptr, transfer.in_ptr, transfer.size);
  }
}

void EcMaster::printMemoryFrames(std::ostream & os)
{
  for (auto & kv : domain_info_) {
    os << "Domain: " << kv.first << std::endl;
    auto & d = kv.second;
    size_t size = ecrt_domain_size(d->domain);
    // Display the memory
    for (size_t i = 0; i < size; i++) {
      os << std::hex << static_cast<int>(d->domain_pd[i]) << " ";
    }
    os << std::endl;
  }
}

uint8_t * EcMaster::getMemoryStart(
  const uint16_t position,
  const uint16_t index,
  const uint16_t subindex)
{
  for (auto & kv : domain_info_) {
    auto & d = kv.second;
    for (auto & reg : d->domain_regs) {
      if (reg.position == position && reg.index == index && reg.subindex == subindex) {
        return d->domain_pd + *(reg.offset);
      }
    }
  }
  return nullptr;
}

void EcMaster::printMemoryFrame(
  const uint16_t position,
  const uint16_t index,
  const uint16_t subindex,
  const size_t n,
  bool binary,
  std::ostream & os)
{
  uint8_t * pointer = getMemoryStart(position, index, subindex);
  if (pointer != nullptr) {
    for (size_t i = 0; i < n; i++) {
      if (binary) {
        os << std::bitset<8>(pointer[i]) << " ";
      } else {
        os << std::hex << static_cast<int>(pointer[i]) << " ";
      }
    }
    os << std::endl;
  }
}

}  // namespace ethercat_interface
