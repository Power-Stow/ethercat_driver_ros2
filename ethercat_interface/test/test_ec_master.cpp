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

#include <ecrt.h>
#include <gtest/gtest.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <thread>
#include <vector>

#include "ethercat_interface/ec_diagnostics.hpp"
#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_sdo_manager.hpp"
#include "ethercat_interface/ec_slave.hpp"

struct ec_reg_request
{
  size_t size = 0;
  uint16_t address = 0;
  int reads = 0;
  ec_request_state_t state = EC_REQUEST_UNUSED;
  uint8_t data[8] = {};
};

struct ec_slave_config
{
  std::vector<uint16_t> registered_indices;
  ec_slave_config_state_t state = {};
  // A deque, so the requests handed out keep their address when more are created.
  std::deque<ec_reg_request> reg_requests;
};

struct ec_domain
{
};

struct ec_master
{
  int download_result = 0;
  ec_slave_config config;
  ec_domain domain;
  std::vector<uint64_t> application_times;
  std::chrono::milliseconds send_duration{0};
  uint64_t send_returned_ns = 0;
};

namespace
{

/// Exposes the cyclic internals that the SYNC0 tests drive one at a time.
class TestMaster : public ethercat_interface::EcMaster
{
public:
  using EcMaster::EcMaster;
  using EcMaster::applicationTimeNs;
  using EcMaster::checkSlaveStates;
  using EcMaster::createRegisterRequests;
  using EcMaster::serviceRegisterRequests;
  using EcMaster::updateSync0Margin;

  void addDomain() {domain_info_[0] = new ethercat_interface::DomainInfo(master_);}
  void setSync0Phase(int64_t phase_ns) {sync0_phase_ns_ = phase_ns;}
};

/// A slave configured for SYNC0 that records the margin the master hands it.
class DcSlave : public ethercat_interface::EcSlave, public ethercat_interface::Sync0MarginConsumer
{
public:
  DcSlave()
  : EcSlave(0, 0, 0, 0) {}

  int assign_activate_dc_sync() override {return 0x0300;}  // SYNC0 active
  void setSync0Margin(int64_t margin_ns) override {margin_ns_ = margin_ns;}

  int64_t margin_ns_ = 0;
};

}  // namespace

extern "C" ec_master_t * ecrt_request_master(unsigned int)
{
  static ec_master master;
  return &master;
}

extern "C" void ecrt_release_master(ec_master_t *) {}

extern "C" ec_slave_config_t * ecrt_master_slave_config(
  ec_master_t * master, uint16_t, uint16_t,
  uint32_t, uint32_t)
{
  return &master->config;
}

extern "C" int ecrt_master_sdo_download(
  ec_master_t * master, uint16_t, uint16_t, uint8_t, const uint8_t *, size_t,
  uint32_t *)
{
  return master->download_result;
}

extern "C" int ecrt_slave_config_dc(
  ec_slave_config_t *, uint16_t, uint32_t, int32_t, uint32_t, int32_t)
{
  return 0;
}

extern "C" int ecrt_master_application_time(ec_master_t * master, uint64_t app_time)
{
  master->application_times.push_back(app_time);
  return 0;
}

extern "C" int ecrt_master_sync_reference_clock(ec_master_t *) {return 0;}

extern "C" int ecrt_master_sync_slave_clocks(ec_master_t *) {return 0;}

extern "C" ec_domain_t * ecrt_master_create_domain(ec_master_t * master) {return &master->domain;}

extern "C" int ecrt_domain_queue(ec_domain_t *) {return 0;}

extern "C" int ecrt_domain_process(ec_domain_t *) {return 0;}

extern "C" int ecrt_domain_state(const ec_domain_t *, ec_domain_state_t * state)
{
  *state = {};
  return 0;
}

extern "C" int ecrt_master_receive(ec_master_t *) {return 0;}

extern "C" int ecrt_master_send(ec_master_t * master)
{
  std::this_thread::sleep_for(master->send_duration);
  master->send_returned_ns = TestMaster::applicationTimeNs();
  return 0;
}

extern "C" int ecrt_master_state(const ec_master_t *, ec_master_state_t * state)
{
  *state = {};
  return 0;
}

extern "C" int ecrt_slave_config_state(
  const ec_slave_config_t * config, ec_slave_config_state_t * state)
{
  *state = config->state;
  return 0;
}

extern "C" ec_reg_request_t * ecrt_slave_config_create_reg_request(
  ec_slave_config_t * config, size_t size)
{
  config->reg_requests.emplace_back();
  config->reg_requests.back().size = size;
  return &config->reg_requests.back();
}

extern "C" ec_request_state_t ecrt_reg_request_state(const ec_reg_request_t * request)
{
  return request->state;
}

extern "C" uint8_t * ecrt_reg_request_data(const ec_reg_request_t * request)
{
  return const_cast<uint8_t *>(request->data);
}

extern "C" int ecrt_reg_request_read(ec_reg_request_t * request, uint16_t address, size_t)
{
  request->address = address;
  ++request->reads;
  request->state = EC_REQUEST_BUSY;
  return 0;
}

extern "C" int ecrt_slave_config_sdo(
  ec_slave_config_t * config, uint16_t index, uint8_t, const uint8_t * data,
  size_t size)
{
  EXPECT_EQ(size, sizeof(uint8_t));
  EXPECT_EQ(EC_READ_U8(data), 1);
  config->registered_indices.push_back(index);
  return 0;
}

TEST(EcMaster, replays_only_successful_startup_downloads_and_retries_on_reactivation)
{
  auto * fake_master = ecrt_request_master(0);
  *fake_master = ec_master{};
  ethercat_interface::EcMaster master(0);
  ethercat_interface::EcSlave slave(0, 0, 0, 0);
  ethercat_interface::SdoConfigEntry sdo;
  sdo.index = 0;
  sdo.sub_index = 0;
  sdo.data_type = "uint8";
  sdo.data = 1;
  slave.sdo_config.push_back(sdo);
  sdo.index = 1;
  slave.sdo_config.push_back(sdo);

  master.addSlave(&slave);
  EXPECT_TRUE(fake_master->config.registered_indices.empty());

  uint32_t abort_code = 0;
  ASSERT_EQ(master.configSlaveSdo(&slave, slave.sdo_config.front(), &abort_code, true), 0);
  fake_master->download_result = -EIO;
  EXPECT_EQ(master.configSlaveSdo(&slave, slave.sdo_config.back(), &abort_code, true), -EIO);
  EXPECT_EQ(fake_master->config.registered_indices, (std::vector<uint16_t>{0}));
  // Two synthetic entries, one accepted and one rejected.
  EXPECT_EQ(slave.sdo_config.size(), std::size_t{2});

  master.shutdown();
  fake_master->config.registered_indices.clear();
  fake_master->download_result = 0;
  ethercat_interface::EcMaster reactivated_master(0);
  reactivated_master.addSlave(&slave);
  for (const auto & entry : slave.sdo_config) {
    ASSERT_EQ(reactivated_master.configSlaveSdo(&slave, entry, &abort_code, true), 0);
  }
  EXPECT_EQ(fake_master->config.registered_indices, (std::vector<uint16_t>{0, 1}));

  ASSERT_EQ(reactivated_master.configSlaveSdo(&slave, slave.sdo_config.front(), &abort_code, false),
    0);
  EXPECT_EQ(fake_master->config.registered_indices, (std::vector<uint16_t>{0, 1}));
}

namespace
{

constexpr double FREQUENCY_HZ = 100.0;
constexpr uint64_t CYCLE_NS = 10000000;  // at FREQUENCY_HZ
constexpr int32_t SHIFT_NS = 5000000;

/// The register request the master reads the next SYNC0 time with, the only 8-byte one.
ec_reg_request & next_sync0_request(ec_master * fake_master)
{
  for (auto & request : fake_master->config.reg_requests) {
    if (request.size == 8) {
      return request;
    }
  }
  throw std::runtime_error("no next SYNC0 register request");
}

}  // namespace

TEST(EcMaster, reads_the_sync0_phase_once_a_dc_slave_is_in_op_and_retries_a_failed_read)
{
  auto * fake_master = ecrt_request_master(0);
  *fake_master = ec_master{};
  DcSlave slave;
  TestMaster master(0);
  master.setCtrlFrequency(FREQUENCY_HZ);
  master.setDcSync0Shift(SHIFT_NS);
  master.addSlave(&slave);
  master.createRegisterRequests();
  master.addDomain();
  // A cyclic frame sets the application time the register value is unwrapped around.
  master.writeData();
  ec_reg_request & request = next_sync0_request(fake_master);

  master.serviceRegisterRequests();
  EXPECT_EQ(request.reads, 0);
  auto timing = master.takeSync0Timing();
  EXPECT_TRUE(timing.uses_sync0);
  EXPECT_EQ(timing.phase_wait__ns, -1);

  fake_master->config.state.operational = 1;
  master.checkSlaveStates();
  master.serviceRegisterRequests();
  EXPECT_EQ(request.reads, 1);
  EXPECT_EQ(request.address, 0x0990);
  EXPECT_GE(master.takeSync0Timing().phase_wait__ns, 0);

  request.state = EC_REQUEST_ERROR;
  master.serviceRegisterRequests();
  EXPECT_EQ(request.reads, 2);

  // A SYNC0 30 us after the configured one, a few cycles after the last application time.
  const uint64_t reference_ns = fake_master->application_times.front();
  const uint64_t app_time_ns = fake_master->application_times.back();
  const uint64_t cycles = (app_time_ns - reference_ns) / CYCLE_NS + 3;
  EC_WRITE_U64(request.data, reference_ns + cycles * CYCLE_NS + SHIFT_NS + 30000);
  request.state = EC_REQUEST_SUCCESS;
  master.serviceRegisterRequests();
  timing = master.takeSync0Timing();
  EXPECT_TRUE(timing.phase_error_valid);
  EXPECT_EQ(timing.phase_error__ns, 30000);
  EXPECT_EQ(timing.phase_wait__ns, -1);
  // No frame was sent since the phase is known.
  EXPECT_FALSE(timing.valid);

  master.serviceRegisterRequests();
  EXPECT_EQ(request.reads, 2);
}

TEST(EcMaster, reports_when_no_slave_uses_sync0)
{
  auto * fake_master = ecrt_request_master(0);
  *fake_master = ec_master{};
  ethercat_interface::EcSlave slave(0, 0, 0, 0);
  TestMaster master(0);
  master.setCtrlFrequency(FREQUENCY_HZ);
  master.addSlave(&slave);

  const auto timing = master.takeSync0Timing();
  EXPECT_FALSE(timing.uses_sync0);
  EXPECT_FALSE(timing.valid);
  EXPECT_EQ(timing.phase_wait__ns, -1);
}

TEST(EcMaster, measures_the_margin_from_when_the_frame_was_sent)
{
  auto * fake_master = ecrt_request_master(0);
  *fake_master = ec_master{};
  DcSlave slave;
  TestMaster master(0);
  // A 1 s cycle, so the margins measured over this test do not wrap.
  master.setCtrlFrequency(1.0);
  constexpr uint32_t long_cycle_ns = 1000000000;
  master.addSlave(&slave);
  master.addDomain();
  const auto phase_ns =
    static_cast<int64_t>((TestMaster::applicationTimeNs() + long_cycle_ns / 2) % long_cycle_ns);
  master.setSync0Phase(phase_ns);
  fake_master->send_duration = std::chrono::milliseconds(20);

  master.writeData();
  const uint64_t after_ns = TestMaster::applicationTimeNs();

  const auto timing = master.takeSync0Timing();
  ASSERT_TRUE(timing.valid);
  // The margin shrinks as time passes, so a time sampled before the send would exceed the bound.
  using ethercat_interface::sync0_margin_ns;
  EXPECT_LE(
    timing.margin__ns,
    sync0_margin_ns(fake_master->send_returned_ns, phase_ns, long_cycle_ns));
  EXPECT_GE(timing.margin__ns, sync0_margin_ns(after_ns, phase_ns, long_cycle_ns));
}

TEST(EcMaster, counts_a_skipped_cycle_and_a_late_frame_as_slips)
{
  auto * fake_master = ecrt_request_master(0);
  *fake_master = ec_master{};
  TestMaster master(0);
  master.setCtrlFrequency(FREQUENCY_HZ);
  master.setSync0Phase(5000000);
  // Sent 1 ms into a cycle, 4 ms before its SYNC0.
  const uint64_t start_ns = 1000 * CYCLE_NS + 1000000;

  master.updateSync0Margin(start_ns);
  master.updateSync0Margin(start_ns + CYCLE_NS);
  EXPECT_EQ(master.takeSync0Timing().slip_count, 0U);

  // No frame in the cycle in between.
  master.updateSync0Margin(start_ns + 3 * CYCLE_NS);
  EXPECT_EQ(master.takeSync0Timing().slip_count, 1U);

  // Sent 0.5 ms after the SYNC0 it was meant for, so it reaches the next one.
  master.updateSync0Margin(start_ns + 4 * CYCLE_NS + 4500000);
  master.updateSync0Margin(start_ns + 5 * CYCLE_NS + 4400000);
  const auto timing = master.takeSync0Timing();
  EXPECT_EQ(timing.slip_count, 2U);
  EXPECT_EQ(timing.window_min_distance__ns, 400000);

  EXPECT_EQ(master.takeSync0Timing().window_min_distance__ns, -1);
}

TEST(EcMaster, hands_the_margin_to_a_read_but_not_to_an_update)
{
  auto * fake_master = ecrt_request_master(0);
  *fake_master = ec_master{};
  DcSlave slave;
  TestMaster master(0);
  master.setCtrlFrequency(FREQUENCY_HZ);
  master.addSlave(&slave);
  master.addDomain();
  master.setSync0Phase(5000000);
  master.updateSync0Margin(1000 * CYCLE_NS + 1000000);

  master.readData();
  EXPECT_EQ(slave.margin_ns_, 4000000);

  master.update();
  EXPECT_EQ(slave.margin_ns_, -1);
}
