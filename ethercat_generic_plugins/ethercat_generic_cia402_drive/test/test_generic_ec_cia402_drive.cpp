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
#include <limits>
#include <pluginlib/class_loader.hpp>
#include "ethercat_interface/ec_slave.hpp"
#include "test_generic_ec_cia402_drive.hpp"

const char test_drive_config[] =
  R"(
# Configuration file for Test drive
vendor_id: 0x00000011
product_id: 0x07030924
assign_activate: 0x0321  # DC Synch register
period: 100  # Hz
auto_fault_reset: false  # true = automatic fault reset, false = fault reset on rising edge command interface "reset_fault"
sdo:  # sdo data to be transferred at drive startup
  - {index: 0x60C2, sub_index: 1, type: int8, value: 10} # Set interpolation time for cyclic modes to 10 ms
  - {index: 0x60C2, sub_index: 2, type: int8, value: -3} # Set base 10-3s
rpdo:  # RxPDO
  - index: 0x1607
    channels:
      - {index: 0x607a, sub_index: 0, type: int32, command_interface: position, default: .nan}  # Target position
      - {index: 0x60ff, sub_index: 0, type: int32, command_interface: velocity, default: 0}  # Target velocity
      - {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5}  # Target torque
      - {index: 0x6072, sub_index: 0, type: int16, command_interface: ~, default: 1000}  # Max torque
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: ~, default: 0}  # Control word
      - {index: 0x6060, sub_index: 0, type: int8, command_interface: mode_of_operation, default: 8}  # Mode of operation
tpdo:  # TxPDO
  - index: 0x1a07
    channels:
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position}  # Position actual value
      - {index: 0x606c, sub_index: 0, type: int32, state_interface: velocity}  # Velocity actual value
      - {index: 0x6077, sub_index: 0, type: int16, state_interface: effort}  # Torque actual value
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: ~}  # Status word
      - {index: 0x6061, sub_index: 0, type: int8, state_interface: mode_of_operation}  # Mode of operation display
  - index: 0x1a45
    channels:
      - {index: 0x2205, sub_index: 1, type: int16, state_interface: analog_input1}  # Analog input
      - {index: 0x2205, sub_index: 2, type: int16, state_interface: analog_input2}  # Analog input
)";

void EcCiA402DriveTest::SetUp()
{
  plugin_ = std::make_unique<FriendEcCiA402Drive>();
}

void EcCiA402DriveTest::TearDown()
{
  plugin_.reset(nullptr);
}

TEST_F(EcCiA402DriveTest, SlaveSetupNoDriveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_parameters;
  // setup failed, 'drive_config' parameter not set
  ASSERT_EQ(
    plugin_->setupSlave(
      slave_parameters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(EcCiA402DriveTest, SlaveSetupMissingFileDriveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_parameters;
  slave_parameters["drive_config"] = "drive_config.yaml";
  // setup failed, 'drive_config.yaml' file not set
  ASSERT_EQ(
    plugin_->setupSlave(
      slave_parameters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(EcCiA402DriveTest, SlaveSetupDriveFromConfig)
{
  SetUp();
  ASSERT_EQ(
    plugin_->setup_from_config(YAML::Load(test_drive_config)),
    true
  );
  ASSERT_EQ(plugin_->vendor_id_, 0x00000011);
  ASSERT_EQ(plugin_->product_id_, 0x07030924);
  ASSERT_EQ(plugin_->assign_activate_, 0x0321);
  ASSERT_EQ(plugin_->auto_fault_reset_, false);
  ASSERT_EQ(plugin_->quick_stop_supported_, false) << "Quick Stop must be opt-in per drive";

  ASSERT_EQ(plugin_->rpdos_.size(), 1);
  ASSERT_EQ(plugin_->rpdos_[0].index, 0x1607);

  ASSERT_EQ(plugin_->tpdos_.size(), 2);
  ASSERT_EQ(plugin_->tpdos_[0].index, 0x1a07);
  ASSERT_EQ(plugin_->tpdos_[1].index, 0x1a45);


  auto channels = plugin_->pdo_channels_info_;
  ASSERT_EQ(channels[1]->interface_name(), "velocity") << "Interface name is not 'velocity'";
  ASSERT_EQ(channels[3]->data().default_value, 1000) << "Default value is not 1000";
  ASSERT_TRUE(std::isnan(channels[0]->data().default_value)) << "Default value is not NaN";
  ASSERT_EQ(channels[4]->interface_name(), "null") << "Interface name is not 'null'";
  ASSERT_EQ(
    channels[12]->interface_name(),
    "analog_input2") << "Interface name is not 'analog_input2'";
  ASSERT_EQ(channels[4]->data_type(), "uint16") << "Data type is not 'uint16'";
}

TEST_F(EcCiA402DriveTest, SlaveSetupPdoChannels)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  std::vector<ec_pdo_entry_info_t> channels(
    plugin_->channels(),
    plugin_->channels() + plugin_->all_channels_.size()
  );

  ASSERT_EQ(channels.size(), 13);
  ASSERT_EQ(channels[0].index, 0x607a);
  ASSERT_EQ(channels[11].index, 0x2205);
  ASSERT_EQ(channels[11].subindex, 0x01);
}

TEST_F(EcCiA402DriveTest, SlaveSetupSyncs)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_drive_config));
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

TEST_F(EcCiA402DriveTest, SlaveSetupDomains)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  std::map<unsigned int, std::vector<unsigned int>> domains;
  plugin_->domains(domains);

  ASSERT_EQ(domains[0].size(), 13);
  ASSERT_EQ(domains[0][0], 0);
  ASSERT_EQ(domains[0][12], 12);
}

TEST_F(EcCiA402DriveTest, EcReadTPDOToStateInterface)
{
  SetUp();
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0, 0};
  plugin_->state_interface_ptr_ = &state_interface;
  slave_parameters["state_interface/effort"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  ASSERT_EQ(plugin_->pdo_channels_info_[8]->state_interface_index(), 1);
  uint8_t domain_address[2];
  EC_WRITE_S16(domain_address, 42);
  plugin_->processData(8, domain_address);
  ASSERT_EQ(plugin_->state_interface_ptr_->at(1), 42);
}

TEST_F(EcCiA402DriveTest, EcWriteRPDOFromCommandInterface)
{
  SetUp();
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> command_interface = {0, 42};
  plugin_->command_interface_ptr_ = &command_interface;
  slave_parameters["command_interface/effort"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  auto channels = plugin_->pdo_channels_info_;
  ASSERT_EQ(channels[2]->command_interface_index(), 1);
  plugin_->mode_of_operation_display_ = 10;
  uint8_t domain_address[2];
  plugin_->processData(2, domain_address);
  ASSERT_EQ(channels[2]->data().last_value, 42);
  ASSERT_EQ(EC_READ_S16(domain_address), 42);
}

TEST_F(EcCiA402DriveTest, EcWriteRPDODefaultValue)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->mode_of_operation_display_ = 10;
  uint8_t domain_address[2];
  plugin_->processData(2, domain_address);
  auto channels = plugin_->pdo_channels_info_;
  ASSERT_EQ(channels[2]->data().last_value, -5);
  ASSERT_EQ(EC_READ_S16(domain_address), -5);
}

// TEST_F(EcCiA402DriveTest, FaultReset)
// {
//   std::unordered_map<std::string, std::string> slave_parameters;
//   std::vector<double> command_interface = {0, 1};
//   plugin_->command_interface_ptr_ = &command_interface;
//   plugin_->setup_from_config(YAML::Load(test_drive_config));
//   plugin_->setup_interface_mapping();
//   plugin_->fault_reset_command_interface_index_ = 1;
//   plugin_->state_ = STATE_FAULT;
//   plugin_->is_operational_ = true;
//   uint8_t domain_address = 0;
//   plugin_->pdo_channels_info_[4].data_type = "";
//   ASSERT_FALSE(plugin_->last_fault_reset_command_);
//   ASSERT_FALSE(plugin_->fault_reset_);
//   ASSERT_EQ(plugin_->command_interface_ptr_->at(
//     plugin_->fault_reset_command_interface_index_), 1);
//   plugin_->processData(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b10000000);
//   plugin_->pdo_channels_info_[4].last_value = 0;
//   plugin_->processData(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b00000000);
//   command_interface[1] = 0;
//   plugin_->processData(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b00000000);
//   command_interface[1] = 2;  plugin_->processData(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b10000000);
// }

TEST_F(EcCiA402DriveTest, SwitchModeOfOperation)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> command_interface = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  slave_parameters["command_interface/mode_of_operation"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  uint8_t domain_address[2];
  plugin_->processData(5, domain_address);
  ASSERT_EQ(EC_READ_S8(domain_address), 8);
  command_interface[1] = 9;
  plugin_->processData(5, domain_address);
  plugin_->processData(10, domain_address);
  ASSERT_EQ(EC_READ_S8(domain_address), 9);
  ASSERT_EQ(plugin_->mode_of_operation_display_, 9);
}

TEST_F(EcCiA402DriveTest, EcWriteDefaultTargetPosition)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> command_interface = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  slave_parameters["command_interface/mode_of_operation"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->mode_of_operation_display_ = 8;
  uint8_t domain_address[4];
  uint8_t domain_address_moo[2];

  plugin_->processData(5, domain_address_moo);  // mode_of_operation
  plugin_->processData(10, domain_address_moo);  // mode_of_operation_display
  ASSERT_EQ(plugin_->mode_of_operation_display_, 8);

  EC_WRITE_S32(domain_address, 123456);
  plugin_->processData(6, domain_address);
  ASSERT_EQ(plugin_->last_position_, 123456);

  EC_WRITE_S32(domain_address, 0);
  plugin_->processData(0, domain_address);
  ASSERT_EQ(EC_READ_S32(domain_address), 123456);

  command_interface[1] = 9;
  plugin_->processData(5, domain_address_moo);
  plugin_->processData(10, domain_address_moo);
  ASSERT_EQ(plugin_->mode_of_operation_display_, 9);

  EC_WRITE_S32(domain_address, 0);
  plugin_->processData(0, domain_address);
  ASSERT_EQ(EC_READ_S32(domain_address), 123456);

  EC_WRITE_S32(domain_address, 654321);
  plugin_->processData(6, domain_address);
  ASSERT_EQ(plugin_->last_position_, 654321);

  EC_WRITE_S32(domain_address, 0);
  plugin_->processData(0, domain_address);
  ASSERT_EQ(EC_READ_S32(domain_address), 654321);
}

TEST_F(EcCiA402DriveTest, JointOffsetAppliesToTpdoAndDefaultRpdoPosition)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  slave_parameters["state_interface/position"] = "0";
  slave_parameters["command_interface/mode_of_operation"] = "1";
  slave_parameters["joint_offset"] = "2.5";
  plugin_->parameters_ = slave_parameters;
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->joint_offset_ = 2.5;
  plugin_->is_operational_ = true;
  plugin_->mode_of_operation_display_ = 8;

  uint8_t domain_address[4];
  EC_WRITE_S32(domain_address, 100);
  plugin_->processData(6, domain_address);

  ASSERT_EQ(plugin_->last_position_, 102.5);
  ASSERT_EQ(plugin_->state_interface_ptr_->at(0), 102.5);

  EC_WRITE_S32(domain_address, 0);
  plugin_->processData(0, domain_address);
  ASSERT_EQ(EC_READ_S32(domain_address), 100);
}

TEST_F(EcCiA402DriveTest, JointOffsetCompensatesCspCommandPosition)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {42.5, std::numeric_limits<double>::quiet_NaN()};
  slave_parameters["state_interface/position"] = "0";
  slave_parameters["command_interface/position"] = "0";
  slave_parameters["command_interface/mode_of_operation"] = "1";
  slave_parameters["joint_offset"] = "2.5";
  plugin_->parameters_ = slave_parameters;
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->joint_offset_ = 2.5;
  plugin_->is_operational_ = true;
  plugin_->mode_of_operation_display_ = 8;

  uint8_t domain_address[4];
  EC_WRITE_S32(domain_address, 0);
  plugin_->processData(0, domain_address);

  ASSERT_EQ(EC_READ_S32(domain_address), 40);
}

TEST_F(EcCiA402DriveTest, JointOffsetStartupWrapDisabledKeepsLegacyBehavior)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0.0, 0.0};
  slave_parameters["state_interface/position"] = "0";
  slave_parameters["joint_offset"] = "-3.0";
  plugin_->parameters_ = slave_parameters;
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->joint_offset_ = -3.0;

  uint8_t domain_address[4];
  EC_WRITE_S32(domain_address, -4);
  plugin_->processData(6, domain_address);

  EXPECT_NEAR(plugin_->last_position_, -7.0, 1e-9);
  EXPECT_NEAR(plugin_->state_interface_ptr_->at(0), -7.0, 1e-9);
  EXPECT_FALSE(plugin_->joint_offset_startup_wrap_applied_);
  EXPECT_NEAR(plugin_->joint_offset_, -3.0, 1e-9);
}

TEST_F(EcCiA402DriveTest, JointOffsetStartupWrapEnabledAdjustsFirstSampleOnly)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0.0, 0.0};
  slave_parameters["state_interface/position"] = "0";
  slave_parameters["joint_offset"] = "-3.0";
  slave_parameters["joint_offset_startup_wrap_enabled"] = "true";
  plugin_->parameters_ = slave_parameters;
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->joint_offset_ = -3.0;
  plugin_->joint_offset_startup_wrap_enabled_ = true;
  plugin_->status_word_ = 0x1237;

  uint8_t domain_address[4];
  EC_WRITE_S32(domain_address, -4);
  plugin_->processData(6, domain_address);

  EXPECT_TRUE(plugin_->joint_offset_startup_wrap_applied_);
  EXPECT_NEAR(plugin_->joint_offset_, 3.2831853071795862, 1e-9);
  EXPECT_NEAR(plugin_->last_position_, -0.7168146928204138, 1e-9);
  EXPECT_NEAR(plugin_->state_interface_ptr_->at(0), -0.7168146928204138, 1e-9);

  EC_WRITE_S32(domain_address, -3);
  plugin_->processData(6, domain_address);

  EXPECT_NEAR(plugin_->joint_offset_, 3.2831853071795862, 1e-9);
  EXPECT_NEAR(plugin_->last_position_, 0.28318530717958623, 1e-9);
  EXPECT_NEAR(plugin_->state_interface_ptr_->at(0), 0.28318530717958623, 1e-9);
}

TEST_F(EcCiA402DriveTest, JointOffsetStartupWrapWaitsForSlaveData)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {123.0, 0.0};
  slave_parameters["state_interface/position"] = "0";
  slave_parameters["joint_offset"] = "-3.0";
  slave_parameters["joint_offset_startup_wrap_enabled"] = "true";
  plugin_->parameters_ = slave_parameters;
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->joint_offset_ = -3.0;
  plugin_->joint_offset_startup_wrap_enabled_ = true;

  uint8_t domain_address[4];
  EC_WRITE_S32(domain_address, 0);
  plugin_->processData(6, domain_address);

  EXPECT_FALSE(plugin_->joint_offset_startup_wrap_applied_);
  EXPECT_NEAR(plugin_->joint_offset_, -3.0, 1e-9);
  EXPECT_TRUE(std::isnan(plugin_->last_position_));
  EXPECT_NEAR(plugin_->state_interface_ptr_->at(0), 123.0, 1e-9);

  plugin_->status_word_ = 0x1237;
  EC_WRITE_S32(domain_address, -4);
  plugin_->processData(6, domain_address);

  EXPECT_TRUE(plugin_->joint_offset_startup_wrap_applied_);
  EXPECT_NEAR(plugin_->joint_offset_, 3.2831853071795862, 1e-9);
  EXPECT_NEAR(plugin_->last_position_, -0.7168146928204138, 1e-9);
  EXPECT_NEAR(plugin_->state_interface_ptr_->at(0), -0.7168146928204138, 1e-9);
}

TEST_F(EcCiA402DriveTest, WindDownDisablesOperationFromOperationEnabled)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_OPERATION_ENABLED;

  plugin_->start_wind_down(0.01, 1.0);
  EXPECT_FALSE(plugin_->wind_down_complete());

  uint8_t domain_address[4];
  EC_WRITE_U16(domain_address, 0x000F);
  plugin_->processData(4, domain_address);

  // Disable Operation, never Quick Stop: a drive whose quick stop option code and deceleration are
  // not configured can respond to Quick Stop by abandoning the commanded position and accelerating.
  EXPECT_EQ(EC_READ_U16(domain_address), 0x0007);
  EXPECT_FALSE(plugin_->wind_down_complete());
}

TEST_F(EcCiA402DriveTest, WindDownQuickStopsWhenTheDriveSupportsIt)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_OPERATION_ENABLED;
  plugin_->quick_stop_supported_ = true;

  plugin_->start_wind_down(0.01, 1.0);

  uint8_t domain_address[4];
  EC_WRITE_U16(domain_address, 0x000F);
  plugin_->processData(4, domain_address);

  EXPECT_EQ(EC_READ_U16(domain_address), 0x000B);
}

TEST_F(EcCiA402DriveTest, WindDownDisablesVoltageWhenTheQuickStopHoldElapses)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_QUICK_STOP_ACTIVE;
  plugin_->quick_stop_supported_ = true;

  plugin_->start_wind_down(0.01, 1.0);

  uint8_t domain_address[4];
  EC_WRITE_U16(domain_address, 0x000B);
  plugin_->processData(4, domain_address);

  // Still on the quick stop ramp, so the command is held.
  EXPECT_EQ(EC_READ_U16(domain_address), 0x000B);

  // A drive whose quick stop option code holds position in Quick Stop Active never leaves it on
  // its own, so once the ramp budget is spent the voltage is disabled.
  plugin_->wind_down_cycles_ = plugin_->quick_stop_hold_cycles_;
  plugin_->processData(4, domain_address);

  EXPECT_EQ(EC_READ_U16(domain_address), 0x0000);
  EXPECT_FALSE(plugin_->wind_down_complete());
}

TEST_F(EcCiA402DriveTest, WindDownDisablesVoltageOnceTheDriveIsSwitchedOn)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_SWITCH_ON;

  plugin_->start_wind_down(0.01, 1.0);

  uint8_t domain_address[4];
  EC_WRITE_U16(domain_address, 0x0007);
  plugin_->processData(4, domain_address);

  // Switched On has the drive function disabled, so the wind-down is done: Disable Voltage goes
  // out on this same cycle and the drive carries it down to Switch On Disabled by itself.
  EXPECT_EQ(EC_READ_U16(domain_address), 0x0000);
  EXPECT_TRUE(plugin_->wind_down_complete());
}

TEST_F(EcCiA402DriveTest, WindDownCompletesWhenTheDriveParksInReadyToSwitchOn)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_READY_TO_SWITCH_ON;

  plugin_->start_wind_down(0.01, 1.0);

  uint8_t domain_address[4];
  EC_WRITE_U16(domain_address, 0x0000);
  plugin_->processData(4, domain_address);

  // A drive can park here rather than in Switch On Disabled while its DC bus is live. Waiting for
  // a transition it never makes would cost the caller its whole timeout for nothing.
  EXPECT_EQ(EC_READ_U16(domain_address), 0x0000);
  EXPECT_TRUE(plugin_->wind_down_complete());
}

TEST_F(EcCiA402DriveTest, WindDownLeavesQuickStopActiveRatherThanHoldingIt)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_QUICK_STOP_ACTIVE;

  plugin_->start_wind_down(0.01, 1.0);

  uint8_t domain_address[4];
  EC_WRITE_U16(domain_address, 0x000B);
  plugin_->processData(4, domain_address);

  // Whoever put the drive into Quick Stop Active, the wind-down's job is to get it out and
  // de-energised rather than to wait on a ramp it did not command.
  EXPECT_EQ(EC_READ_U16(domain_address), 0x0000);
}

TEST_F(EcCiA402DriveTest, WindDownCompletesOnceTheDriveIsDeEnergised)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_SWITCH_ON_DISABLED;

  plugin_->start_wind_down(0.01, 1.0);
  EXPECT_FALSE(plugin_->wind_down_complete());

  uint8_t domain_address[4];
  EC_WRITE_U16(domain_address, 0x0007);
  plugin_->processData(4, domain_address);

  EXPECT_EQ(EC_READ_U16(domain_address), 0x0000);
  EXPECT_TRUE(plugin_->wind_down_complete());
}

TEST_F(EcCiA402DriveTest, WindDownHoldsTheLastReadPositionInCsp)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {42.5, std::numeric_limits<double>::quiet_NaN()};
  slave_parameters["state_interface/position"] = "0";
  slave_parameters["command_interface/position"] = "0";
  slave_parameters["command_interface/mode_of_operation"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_OPERATION_ENABLED;
  plugin_->mode_of_operation_display_ = 8;
  plugin_->last_position_ = 10.0;

  uint8_t domain_address[4];
  EC_WRITE_S32(domain_address, 0);
  plugin_->processData(0, domain_address);

  // Before the wind-down the drive follows the commanded position.
  ASSERT_EQ(EC_READ_S32(domain_address), 42);

  plugin_->start_wind_down(0.01, 1.0);
  plugin_->processData(0, domain_address);

  // During the wind-down a setpoint left behind by a stopped controller is not replayed.
  EXPECT_EQ(EC_READ_S32(domain_address), 10);
}

TEST_F(EcCiA402DriveTest, WindDownCompletesImmediatelyWhenNotOperational)
{
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->is_operational_ = false;

  // Nothing is written to a drive that is not in OP, so the wind-down must not hold up the caller.
  plugin_->start_wind_down(0.01, 1.0);

  EXPECT_TRUE(plugin_->wind_down_complete());
}

TEST_F(EcCiA402DriveTest, ResetWindDownRestoresCommandChannelsForReactivation)
{
  std::unordered_map<std::string, std::string> slave_parameters;
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0, 42};
  slave_parameters["command_interface/effort"] = "1";
  plugin_->parameters_ = slave_parameters;
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_OPERATION_ENABLED;
  plugin_->mode_of_operation_display_ = 10;

  uint8_t torque_address[2];
  plugin_->processData(2, torque_address);
  ASSERT_EQ(EC_READ_S16(torque_address), 42);

  plugin_->start_wind_down(0.01, 1.0);
  plugin_->processData(2, torque_address);

  // While the wind-down runs the commanded torque is replaced by the configured default.
  ASSERT_EQ(EC_READ_S16(torque_address), -5);

  plugin_->reset_wind_down();

  // A deactivate -> activate cycle reuses this instance, so the override the wind-down forced onto
  // every command channel has to come back off: otherwise the drive spends the next run pinned to
  // its defaults.
  plugin_->processData(2, torque_address);
  EXPECT_EQ(EC_READ_S16(torque_address), 42);

  // And the control word has to follow the automatic transitions again rather than stay pinned to
  // a wind-down command, or the drive can never be taken back up to Operation Enabled.
  plugin_->state_ = STATE_SWITCH_ON_DISABLED;
  uint8_t control_word_address[4];
  EC_WRITE_U16(control_word_address, 0x0000);
  plugin_->processData(4, control_word_address);

  EXPECT_EQ(EC_READ_U16(control_word_address), 0x0006) << "expected Shutdown, not a wind-down word";
}

TEST_F(EcCiA402DriveTest, ResetWindDownLeavesTheWindDownAbleToRunAgain)
{
  std::vector<double> state_interface = {0.0, 0.0};
  std::vector<double> command_interface = {0.0, 0.0};
  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();
  plugin_->is_operational_ = true;
  plugin_->state_ = STATE_OPERATION_ENABLED;

  uint8_t domain_address[4];
  plugin_->start_wind_down(0.01, 1.0);
  EC_WRITE_U16(domain_address, 0x000F);
  plugin_->processData(4, domain_address);
  ASSERT_EQ(EC_READ_U16(domain_address), 0x0007);

  plugin_->reset_wind_down();

  // Idle again, so the next shutdown's loop is not told the wind-down is already finished.
  EXPECT_TRUE(plugin_->wind_down_complete());
  EXPECT_EQ(plugin_->wind_down_cycles_, 0u);

  plugin_->start_wind_down(0.01, 1.0);
  EXPECT_FALSE(plugin_->wind_down_complete());

  EC_WRITE_U16(domain_address, 0x000F);
  plugin_->processData(4, domain_address);

  EXPECT_EQ(EC_READ_U16(domain_address), 0x0007);
}
