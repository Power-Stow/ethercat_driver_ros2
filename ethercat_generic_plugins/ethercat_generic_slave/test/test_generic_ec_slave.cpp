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

#include <map>
#include <pluginlib/class_loader.hpp>
#include "ethercat_interface/ec_slave.hpp"
#include "test_generic_ec_slave.hpp"
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"

const char test_slave_config[] =
  R"(
# Configuration file for Test Slave
vendor_id: 0x00000011
product_id: 0x07030924
assign_activate: 0x0321  # DC Synch register
sdo:  # sdo data to be transferred at slave startup
  - {index: 0x60C2, sub_index: 1, type: int8, value: 10}
  - {index: 0x60C2, sub_index: 2, type: int8, value: -3}
  - {index: 0x6098, sub_index: 0, type: int8, value: 35}
  - {index: 0x6099, sub_index: 0, type: int32, value: 0}
rpdo:  # Receive PDO Mapping
  - index: 0x1607
    channels:
      - {index: 0x607a, sub_index: 0, type: int32, command_interface: position, default: .nan}
      - {index: 0x60ff, sub_index: 0, type: int32, command_interface: velocity, default: 0}
      - {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5, factor: 2, offset: 10}
      - {index: 0x6072, sub_index: 0, type: int16, command_interface: ~, default: 1000}
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: ~, default: 0}
      - {index: 0x6060, sub_index: 0, type: int8, command_interface: ~, default: 8}
tpdo:  # Transmit PDO Mapping
  - index: 0x1a07
    channels:
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position}
      - {index: 0x606c, sub_index: 0, type: int32, state_interface: velocity}
      - {index: 0x6077, sub_index: 0, type: int16, state_interface: effort, factor: 5, offset: 15}
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: ~}
      - {index: 0x6061, sub_index: 0, type: int8, state_interface: ~}
  - index: 0x1a45
    channels:
      - {index: 0x2205, sub_index: 1, type: int16, state_interface: analog_input1}
      - {index: 0x2205, sub_index: 2, type: int16, state_interface: analog_input2}
sm:  # Sync Manager
  - {index: 0, type: output, pdo: ~, watchdog: disable}
  - {index: 1, type: input, pdo: ~, watchdog: disable}
  - {index: 2, type: output, pdo: rpdo, watchdog: enable}
  - {index: 3, type: input, pdo: tpdo, watchdog: disable}
)";

void GenericEcSlaveTest::SetUp()
{
  plugin_ = std::make_unique<FriendGenericEcSlave>();
}

void GenericEcSlaveTest::TearDown()
{
  plugin_.reset(nullptr);
}

TEST_F(GenericEcSlaveTest, SlaveSetupNoSlaveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_parameters;
  // setup failed, 'slave_config' parameter not set
  ASSERT_EQ(
    plugin_->setupSlave(
      slave_parameters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(GenericEcSlaveTest, SlaveSetupMissingFileSlaveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_parameters;
  slave_parameters["slave_config"] = "slave_config.yaml";
  // setup failed, 'slave_config.yaml' file not set
  ASSERT_EQ(
    plugin_->setupSlave(
      slave_parameters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(GenericEcSlaveTest, SlaveSetupSlaveFromConfig)
{
  SetUp();
  ASSERT_EQ(
    plugin_->setup_from_config(YAML::Load(test_slave_config)),
    true
  );
  ASSERT_EQ(plugin_->vendor_id_, 0x00000011);
  ASSERT_EQ(plugin_->product_id_, 0x07030924);
  ASSERT_EQ(plugin_->assign_activate_, 0x0321);

  ASSERT_EQ(plugin_->rpdos_.size(), 1);
  ASSERT_EQ(plugin_->rpdos_[0].index, 0x1607);

  ASSERT_EQ(plugin_->tpdos_.size(), 2);
  ASSERT_EQ(plugin_->tpdos_[0].index, 0x1a07);
  ASSERT_EQ(plugin_->tpdos_[1].index, 0x1a45);

  auto channels = plugin_->pdo_channels_info_;
  ASSERT_EQ(channels[1]->interface_name(), "velocity");
  ASSERT_EQ(channels[2]->data().factor, 2);
  ASSERT_EQ(channels[2]->data().offset, 10);
  ASSERT_EQ(channels[3]->data().default_value, 1000);
  ASSERT_TRUE(std::isnan(channels[0]->data().default_value));
  ASSERT_EQ(channels[4]->interface_name(), "null");
  ASSERT_EQ(channels[12]->interface_name(), "analog_input2");
  ASSERT_EQ(channels[4]->data_type(), "uint16");
}

TEST_F(GenericEcSlaveTest, SlaveSetupPdoChannels)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  std::vector<ec_pdo_entry_info_t> channels(
    plugin_->channels(),
    plugin_->channels() + plugin_->all_channels_.size()
  );

  ASSERT_EQ(channels.size(), 13);
  ASSERT_EQ(channels[0].index, 0x607a);
  ASSERT_EQ(channels[11].index, 0x2205);
  ASSERT_EQ(channels[11].subindex, 0x01);
}

TEST_F(GenericEcSlaveTest, SlaveSetupSyncs)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  plugin_->setup_syncs();
  std::vector<ec_sync_info_t> syncs(
    plugin_->syncs(),
    plugin_->syncs() + plugin_->syncSize()
  );

  ASSERT_EQ(syncs.size(), 5);
  ASSERT_EQ(syncs[1].index, 1);
  ASSERT_EQ(syncs[1].dir, EC_DIR_INPUT);
  ASSERT_EQ(syncs[1].n_pdos, 0);
  ASSERT_EQ(syncs[1].watchdog_mode, EC_WD_DISABLE);
  ASSERT_EQ(syncs[2].dir, EC_DIR_OUTPUT);
  ASSERT_EQ(syncs[2].n_pdos, 1);
  ASSERT_EQ(syncs[3].index, 3);
  ASSERT_EQ(syncs[3].dir, EC_DIR_INPUT);
  ASSERT_EQ(syncs[3].n_pdos, 2);
  ASSERT_EQ(syncs[3].watchdog_mode, EC_WD_DISABLE);
}

TEST_F(GenericEcSlaveTest, SlaveSetupDomains)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  std::map<unsigned int, std::vector<unsigned int>> domains;
  plugin_->domains(domains);

  ASSERT_EQ(domains[0].size(), 13);
  ASSERT_EQ(domains[0][0], 0);
  ASSERT_EQ(domains[0][12], 12);
}

TEST_F(GenericEcSlaveTest, EcReadTPDOToStateInterface)
{
  SetUp();
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0, 0};
  plugin_->state_interface_ptr_ = &state_interface;
  slave_parameters["state_interface/effort"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  plugin_->setup_interface_mapping();
  ASSERT_EQ(plugin_->pdo_channels_info_[8]->state_interface_index(), 1);
  uint8_t domain_address[2];
  EC_WRITE_S16(domain_address, 42);
  plugin_->processData(8, domain_address);
  ASSERT_EQ(plugin_->state_interface_ptr_->at(1), 5 * 42 + 15);
}

TEST_F(GenericEcSlaveTest, EcWriteRPDOFromCommandInterface)
{
  SetUp();
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> command_interface = {0, 42};
  plugin_->command_interface_ptr_ = &command_interface;
  slave_parameters["command_interface/effort"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  plugin_->setup_interface_mapping();
  auto channels = plugin_->pdo_channels_info_;
  ASSERT_EQ(channels[2]->command_interface_index(), 1);
  uint8_t domain_address[2];
  plugin_->processData(2, domain_address);
  ASSERT_EQ(channels[2]->data().last_value, 2 * 42 + 10);
  ASSERT_EQ(EC_READ_S16(domain_address), 2 * 42 + 10);
}

TEST_F(GenericEcSlaveTest, EcWriteRPDODefaultValue)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  plugin_->setup_interface_mapping();
  uint8_t domain_address[2];
  plugin_->processData(2, domain_address);
  ASSERT_EQ(plugin_->pdo_channels_info_[2]->data().last_value, -5);
  ASSERT_EQ(EC_READ_S16(domain_address), -5);
}

TEST_F(GenericEcSlaveTest, SlaveSetupSDOConfig)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  ASSERT_EQ(plugin_->sdo_config[0].index, 0x60C2);
  ASSERT_EQ(plugin_->sdo_config[0].sub_index, 1);
  ASSERT_EQ(plugin_->sdo_config[1].sub_index, 2);
  ASSERT_EQ(plugin_->sdo_config[0].data_size(), 1);
  ASSERT_EQ(plugin_->sdo_config[0].data, 10);
  ASSERT_EQ(plugin_->sdo_config[2].index, 0x6098);
  ASSERT_EQ(plugin_->sdo_config[3].data_type, "int32");
  ASSERT_EQ(plugin_->sdo_config[3].data_size(), 4);
}

TEST_F(GenericEcSlaveTest, SlaveSetupSyncManagerConfig)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  ASSERT_EQ(plugin_->sm_configs_.size(), 4);
  ASSERT_EQ(plugin_->sm_configs_[0].index, 0);
  ASSERT_EQ(plugin_->sm_configs_[0].type, EC_DIR_OUTPUT);
  ASSERT_EQ(plugin_->sm_configs_[0].watchdog, EC_WD_DISABLE);
  ASSERT_EQ(plugin_->sm_configs_[0].pdo_name, "null");
  ASSERT_EQ(plugin_->sm_configs_[2].pdo_name, "rpdo");
  ASSERT_EQ(plugin_->sm_configs_[2].watchdog, EC_WD_ENABLE);
}

// A slave with one channel reading its factor from the drive, as CiA-402 0x6078 would: current in
// thousandths of the rated current held in 0x6075, in mA. `FALLBACK` is replaced per test.
const char factor_from_sdo_slave_config[] =
  R"(
vendor_id: 0x00000011
product_id: 0x07030924
tpdo:
  - index: 0x1a00
    channels:
      - {index: 0x6078, sub_index: 0, type: int16, state_interface: current, FALLBACK
         factor_from_sdo: {index: 0x6075, sub_index: 0, type: uint32, scale: 1.0e-6}}
sm:
  - {index: 3, type: input, pdo: tpdo, watchdog: disable}
)";

std::string factor_from_sdo_config(const std::string & fallback)
{
  std::string config(factor_from_sdo_slave_config);
  config.replace(config.find("FALLBACK"), std::string("FALLBACK").size(), fallback);
  return config;
}

TEST_F(GenericEcSlaveTest, ResolveSdoFactorFromDrive)
{
  SetUp();
  ASSERT_TRUE(plugin_->setup_from_config(YAML::Load(factor_from_sdo_config(""))));

  const auto read_sdo =
    [](uint16_t index, uint8_t sub_index, const std::string & data_type, double * value) {
      EXPECT_EQ(index, 0x6075);
      EXPECT_EQ(sub_index, 0);
      EXPECT_EQ(data_type, "uint32");
      *value = 10000.0;  // a rated current in mA
      return true;
    };
  ASSERT_TRUE(plugin_->resolve_sdo_factors(read_sdo));

  ethercat_interface::EcPdoSingleInterfaceChannelManager * channel = nullptr;
  for (auto * candidate : plugin_->pdo_channels_info_) {
    if (candidate->index == 0x6078) {
      channel =
        static_cast<ethercat_interface::EcPdoSingleInterfaceChannelManager *>(candidate);
    }
  }
  ASSERT_NE(channel, nullptr);
  ASSERT_DOUBLE_EQ(channel->factor, 0.01);
}

TEST_F(GenericEcSlaveTest, ResolveSdoFactorKeepsLiteralOnFailedRead)
{
  SetUp();
  ASSERT_TRUE(plugin_->setup_from_config(YAML::Load(factor_from_sdo_config("factor: 0.05,"))));

  const auto failing_read =
    [](uint16_t, uint8_t, const std::string &, double *) {return false;};
  ASSERT_TRUE(plugin_->resolve_sdo_factors(failing_read));

  ethercat_interface::EcPdoSingleInterfaceChannelManager * channel = nullptr;
  for (auto * candidate : plugin_->pdo_channels_info_) {
    if (candidate->index == 0x6078) {
      channel =
        static_cast<ethercat_interface::EcPdoSingleInterfaceChannelManager *>(candidate);
    }
  }
  ASSERT_NE(channel, nullptr);
  ASSERT_DOUBLE_EQ(channel->factor, 0.05);
}

// Without a literal the channel would be left at the default factor of 1, publishing raw
// thousandths of rated current under an interface that claims amperes. The caller has to be told.
TEST_F(GenericEcSlaveTest, ResolveSdoFactorFailsWithoutFallback)
{
  SetUp();
  ASSERT_TRUE(plugin_->setup_from_config(YAML::Load(factor_from_sdo_config(""))));

  const auto failing_read =
    [](uint16_t, uint8_t, const std::string &, double *) {return false;};
  ASSERT_FALSE(plugin_->resolve_sdo_factors(failing_read));
}

// A drive reporting a zero rating would turn every reading into zero, which looks like an idle
// motor.
TEST_F(GenericEcSlaveTest, ResolveSdoFactorRejectsZero)
{
  SetUp();
  ASSERT_TRUE(plugin_->setup_from_config(YAML::Load(factor_from_sdo_config(""))));

  const auto zero_read =
    [](uint16_t, uint8_t, const std::string &, double * value) {
      *value = 0.0;
      return true;
    };
  ASSERT_FALSE(plugin_->resolve_sdo_factors(zero_read));
}

// A malformed source is a config mistake. It fails resolution even beside a literal factor, and the
// drive is never asked, since there is no well-formed request to send.
TEST_F(GenericEcSlaveTest, ResolveSdoFactorRejectsInvalidSource)
{
  SetUp();
  const char invalid_config[] =
    R"(
vendor_id: 0x00000011
product_id: 0x07030924
tpdo:
  - index: 0x1a00
    channels:
      - {index: 0x6078, sub_index: 0, type: int16, state_interface: current, factor: 0.05,
         factor_from_sdo: {index: 0x6075, scale: 1.0e-6}}
sm:
  - {index: 3, type: input, pdo: tpdo, watchdog: disable}
)";
  plugin_->setup_from_config(YAML::Load(invalid_config));

  bool asked = false;
  const auto read_sdo =
    [&asked](uint16_t, uint8_t, const std::string &, double * value) {
      asked = true;
      *value = 10000.0;
      return true;
    };
  ASSERT_FALSE(plugin_->resolve_sdo_factors(read_sdo));
  ASSERT_FALSE(asked);
}

// Resolution runs on every activation. A read that fails on the second must fall back on the
// config's literal, not on what the first read, which could have come from a drive that has since
// been swapped.
TEST_F(GenericEcSlaveTest, ResolveSdoFactorFallsBackOnLiteralAfterEarlierRead)
{
  SetUp();
  ASSERT_TRUE(plugin_->setup_from_config(YAML::Load(factor_from_sdo_config("factor: 0.05,"))));

  ethercat_interface::EcPdoSingleInterfaceChannelManager * channel = nullptr;
  for (auto * candidate : plugin_->pdo_channels_info_) {
    if (candidate->index == 0x6078) {
      channel =
        static_cast<ethercat_interface::EcPdoSingleInterfaceChannelManager *>(candidate);
    }
  }
  ASSERT_NE(channel, nullptr);

  const auto first_activation =
    [](uint16_t, uint8_t, const std::string &, double * value) {
      *value = 10000.0;
      return true;
    };
  ASSERT_TRUE(plugin_->resolve_sdo_factors(first_activation));
  ASSERT_DOUBLE_EQ(channel->factor, 0.01);

  const auto second_activation =
    [](uint16_t, uint8_t, const std::string &, double *) {return false;};
  ASSERT_TRUE(plugin_->resolve_sdo_factors(second_activation));
  ASSERT_DOUBLE_EQ(channel->factor, 0.05);
}
