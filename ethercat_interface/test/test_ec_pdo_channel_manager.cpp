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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <bitset>
#include <cstring>

#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_interface/ec_pdo_group_interface_channel_manager.hpp"
#include "yaml-cpp/yaml.h"

TEST(TestEcPdoSingleInterfaceChannelManager, LoadFromConfig)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5, factor: 2, offset: 10}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.index, 0x6071);
  ASSERT_EQ(pdo_manager.sub_index, 0);
  ASSERT_EQ(pdo_manager.data_type(), "int16");
  ASSERT_EQ(pdo_manager.interface_name(), "effort");
  ASSERT_EQ(pdo_manager.default_value, -5);
  ASSERT_EQ(pdo_manager.factor, 2);
  ASSERT_EQ(pdo_manager.offset, 10);
}

// A channel whose factor is read from the drive: CiA-402 0x6078 is current in thousandths of the
// rated current stored in 0x6075, in mA. The source is parsed here and resolved later, during
// network configuration, so the factor itself keeps its default until then.
TEST(TestEcPdoSingleInterfaceChannelManager, LoadFactorFromSdo)
{
  const char channel_config[] =
    R"(
      {index: 0x6078, sub_index: 0, type: int16, state_interface: current,
       factor_from_sdo: {index: 0x6075, sub_index: 0, type: uint32, scale: 1.0e-6}}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::TPDO;
  ASSERT_TRUE(pdo_manager.load_from_config(config));

  ASSERT_TRUE(pdo_manager.factor_source.configured);
  ASSERT_TRUE(pdo_manager.factor_source.valid);
  ASSERT_EQ(pdo_manager.factor_source.index, 0x6075);
  ASSERT_EQ(pdo_manager.factor_source.sub_index, 0);
  ASSERT_EQ(pdo_manager.factor_source.data_type, "uint32");
  ASSERT_DOUBLE_EQ(pdo_manager.factor_source.scale, 1.0e-6);
  ASSERT_FALSE(pdo_manager.factor_source.has_literal_fallback);
}

// A literal factor next to the source is the fallback for a drive that cannot be read, and the
// channel has to remember it was given one, because the default factor of 1 looks the same as no
// factor at all.
TEST(TestEcPdoSingleInterfaceChannelManager, LoadFactorFromSdoWithFallback)
{
  const char channel_config[] =
    R"(
      {index: 0x6078, sub_index: 0, type: int16, state_interface: current, factor: 0.01,
       factor_from_sdo: {index: 0x6075, sub_index: 0, type: uint32, scale: 1.0e-6}}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::TPDO;
  ASSERT_TRUE(pdo_manager.load_from_config(config));

  ASSERT_TRUE(pdo_manager.factor_source.has_literal_fallback);
  ASSERT_DOUBLE_EQ(pdo_manager.factor_source.literal_factor, 0.01);
  ASSERT_DOUBLE_EQ(pdo_manager.factor, 0.01);
}

// A source missing what it needs to be read is rejected, and stays known about as configured but
// invalid. The slave does not act on load_from_config's result, so a source that was simply dropped
// would leave the channel at its default factor of 1; kept like this, it fails the bring-up
// instead.
TEST(TestEcPdoSingleInterfaceChannelManager, RejectIncompleteFactorFromSdo)
{
  const char channel_config[] =
    R"(
      {index: 0x6078, sub_index: 0, type: int16, state_interface: current,
       factor_from_sdo: {index: 0x6075, scale: 1.0e-6}}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::TPDO;
  ASSERT_FALSE(pdo_manager.load_from_config(config));
  ASSERT_TRUE(pdo_manager.factor_source.configured);
  ASSERT_FALSE(pdo_manager.factor_source.valid);
}

// A zero scale would turn every reading into zero, which is a plausible current.
TEST(TestEcPdoSingleInterfaceChannelManager, RejectZeroFactorFromSdoScale)
{
  const char channel_config[] =
    R"(
      {index: 0x6078, sub_index: 0, type: int16, state_interface: current,
       factor_from_sdo: {index: 0x6075, sub_index: 0, type: uint32, scale: 0.0}}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::TPDO;
  ASSERT_FALSE(pdo_manager.load_from_config(config));
  ASSERT_TRUE(pdo_manager.factor_source.configured);
  ASSERT_FALSE(pdo_manager.factor_source.valid);
}

// An unsupported type is rejected when the config is parsed, even beside a literal factor. Left to
// the decoder, the read would fail only after reaching the drive,
// and resolution would take that for an unreadable drive
// and fall back on the literal without flagging the config.
TEST(TestEcPdoSingleInterfaceChannelManager, RejectUnsupportedFactorFromSdoType)
{
  const char channel_config[] =
    R"(
      {index: 0x6078, sub_index: 0, type: int16, state_interface: current, factor: 0.01,
       factor_from_sdo: {index: 0x6075, sub_index: 0, type: unit32, scale: 1.0e-6}}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::TPDO;
  ASSERT_FALSE(pdo_manager.load_from_config(config));
  ASSERT_TRUE(pdo_manager.factor_source.configured);
  ASSERT_FALSE(pdo_manager.factor_source.valid);
}

// The typed reader the master decodes an upload with.
TEST(TestSdoConfigEntry, BufferReadDecodesTypes)
{
  uint8_t buffer[8] = {0};
  double value = 0.0;

  EC_WRITE_U32(buffer, 10000);
  ASSERT_TRUE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 4, "uint32", &value));
  ASSERT_DOUBLE_EQ(value, 10000.0);

  EC_WRITE_S16(buffer, -250);
  ASSERT_TRUE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 2, "int16", &value));
  ASSERT_DOUBLE_EQ(value, -250.0);

  const float rated = 1.5f;
  uint32_t raw = 0;
  std::memcpy(&raw, &rated, sizeof(raw));
  EC_WRITE_U32(buffer, raw);
  ASSERT_TRUE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 4, "float", &value));
  ASSERT_FLOAT_EQ(static_cast<float>(value), 1.5f);
}

// A short read, or a type it does not know, fails rather than decoding garbage.
TEST(TestSdoConfigEntry, BufferReadRejectsShortAndUnknown)
{
  uint8_t buffer[8] = {0};
  double value = 0.0;
  ASSERT_FALSE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 2, "uint32", &value));
  ASSERT_FALSE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 8, "string", &value));
  ASSERT_FALSE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 4, "uint32", nullptr));
}

// An upload wider than the configured type fails too.
// Decoding only its low-order bytes would give a plausible wrong factor,
// and a successful read would bypass any literal fallback.
TEST(TestSdoConfigEntry, BufferReadRejectsOversizedUpload)
{
  uint8_t buffer[8] = {0};
  double value = 0.0;
  EC_WRITE_U32(buffer, 70000);
  ASSERT_FALSE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 4, "uint16", &value));
  ASSERT_FALSE(ethercat_interface::SdoConfigEntry::buffer_read(buffer, 8, "uint32", &value));
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadS16)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5, factor: 2, offset: 10}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  uint8_t buffer[16];
  EC_WRITE_S16(buffer, 42);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 2 * 42 + 10);
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBit2)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bit2, mask: 3}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.data_type(), "bit2");
  ASSERT_EQ(pdo_manager.mask, 3);
  ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 2);

  uint8_t buffer[1];
  EC_WRITE_U8(buffer, 0);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 0);
  EC_WRITE_U8(buffer, 3);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 3);
  EC_WRITE_U8(buffer, 5);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 1);

  pdo_manager.ec_write(buffer, 0);
  ASSERT_EQ(EC_READ_U8(buffer), 4);
  pdo_manager.ec_write(buffer, 2);
  ASSERT_EQ(EC_READ_U8(buffer), 6);
  EC_WRITE_U8(buffer, 0);
  pdo_manager.ec_write(buffer, 5);
  ASSERT_EQ(EC_READ_U8(buffer), 1);
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBoolMask1)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bool, mask: 1}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.data_type(), "bool");
  ASSERT_EQ(pdo_manager.mask, 1);
  ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 1);

  uint8_t buffer[1];
  EC_WRITE_U8(buffer, 3);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 1);
  EC_WRITE_U8(buffer, 0);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 0);

  pdo_manager.ec_write(buffer, 0);
  ASSERT_EQ(EC_READ_U8(buffer), 0);
  pdo_manager.ec_write(buffer, 5);
  ASSERT_EQ(EC_READ_U8(buffer), 1);
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBit8Mask5)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bit8, mask: 5}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.data_type(), "bit8");
  ASSERT_EQ(pdo_manager.mask, 5);  // < Set mask 0b00000101
  ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 8);

  uint8_t buffer[1];
  // Should only soft read the bit 5 and 1 that is both in the mask and in the buffer
  EC_WRITE_U8(buffer, 7);  // < Hard write 0b00000111
  ASSERT_EQ(pdo_manager.ec_read(buffer), 5);

  // Hard write 0, should soft read 0
  EC_WRITE_U8(buffer, 0);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 0);

  // Soft write 0, should hard read 0
  pdo_manager.ec_write(buffer, 0);
  ASSERT_EQ(EC_READ_U8(buffer), 0);

  // Soft write 3 (with mask applied is 1) should hard read 0b00000001
  pdo_manager.ec_write(buffer, 3);
  ASSERT_EQ(EC_READ_U8(buffer), 1);

  // Soft write 7 (with mask applied is 5) should hard read 0b00000101
  pdo_manager.ec_write(buffer, 7);
  ASSERT_EQ(EC_READ_U8(buffer), 5);

  // Soft write 5 (with mask applied is 5) should hard read 0b00000101
  pdo_manager.ec_write(buffer, 5);
  ASSERT_EQ(EC_READ_U8(buffer), 5);
}


// This test is very weird, the expected behaviour is somehow confusing
// since it writes the entire octet, but read only the bits set to 1 in the mask
// and converts the result to 1 or 0 (1 if at least one bit is set to 1)
TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBoolMask5)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bool, mask: 5}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  ASSERT_EQ(pdo_manager.load_from_config(config), false);

  return;
  // ASSERT_EQ(pdo_manager.data_type(), "bool");
  // ASSERT_EQ(pdo_manager.mask, 5);  // < Set mask 0b00000101
  // ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 1);

  // uint8_t buffer[1];
  // // Should only soft read the bit 1 that is both in the mask and in the buffer
  // EC_WRITE_U8(buffer, 7);  // < Hard write 0b00000111
  // ASSERT_EQ(pdo_manager.ec_read(buffer), 1);

  // // Hard write 0, should soft read 0
  // EC_WRITE_U8(buffer, 0);
  // ASSERT_EQ(pdo_manager.ec_read(buffer), 0);

  // // Soft write 0, should hard read 0
  // pdo_manager.ec_write(buffer, 0);
  // ASSERT_EQ(EC_READ_U8(buffer), 0);

  // // Soft write 3 (with mask applied is 1) should hard read 0b00000001
  // pdo_manager.ec_write(buffer, 3);
  // ASSERT_EQ(EC_READ_U8(buffer), 1);

  // // Soft write 7 (with mask applied is 5) should hard read 0b00000101
  // pdo_manager.ec_write(buffer, 7);
  // ASSERT_EQ(EC_READ_U8(buffer), 5);

  // // Soft write 5 (with mask applied is 5) should hard read 0b00000101
  // pdo_manager.ec_write(buffer, 5);
  // ASSERT_EQ(EC_READ_U8(buffer), 5);
}


TEST(TestEcPdoGroupInterfaceChannelManager, LoadConfigTest)
{
  const char channel_config[] =
    R"(
      {
        index: 0xf788,
        sub_index: 0x00,
        type: bit240,
        data_mapping: [
          {
            addr_offset: 60,
            type: int32,
            factor: 3.14,
            offset: 2.71,
            command_interface: effort
          },
          {
            addr_offset: 64,
            type: int16,
            factor: 1.1,
            offset: 0.1,
            state_interface: position
          },
          {
            addr_offset: 66,
            type: uint8,
            mask: 7,
          },
          {
            addr_offset: 67,
            type: bool,
            mask: 8,
          }
        ]
      }
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoGroupInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  ASSERT_EQ(pdo_manager.load_from_config(config), true);

  ASSERT_EQ(pdo_manager.number_of_interfaces(), 5);
  ASSERT_EQ(pdo_manager.number_of_managed_interfaces(), 2);

  ASSERT_EQ(pdo_manager.data_type(0), "bit240");
  ASSERT_EQ(pdo_manager.interface_name(0), "null");
  ASSERT_EQ(pdo_manager.index, 0xf788);
  ASSERT_EQ(pdo_manager.sub_index, 0);

  ASSERT_EQ(pdo_manager.data_type(1), "int32");
  ASSERT_EQ(pdo_manager.interface_name(1), "effort");
  ASSERT_EQ(pdo_manager.data(1).factor, 3.14);
  ASSERT_EQ(pdo_manager.data(1).offset, 2.71);
  ASSERT_EQ(pdo_manager.v_data[1].addr_offset, 60);

  ASSERT_EQ(pdo_manager.data_type(2), "int16");
  ASSERT_EQ(pdo_manager.interface_name(2), "position");
  ASSERT_EQ(pdo_manager.data(2).factor, 1.1);
  ASSERT_EQ(pdo_manager.data(2).offset, 0.1);
  ASSERT_EQ(pdo_manager.v_data[2].addr_offset, 64);

  ASSERT_EQ(pdo_manager.data_type(3), "uint8");
  ASSERT_EQ(pdo_manager.interface_name(3), "null");
  ASSERT_EQ(pdo_manager.data(3).mask, 7);
  ASSERT_EQ(pdo_manager.v_data[3].addr_offset, 66);

  ASSERT_EQ(pdo_manager.data_type(4), "bool");
  ASSERT_EQ(pdo_manager.interface_name(4), "null");
  ASSERT_EQ(pdo_manager.data(4).mask, 8);
}


TEST(TestEcPdoGroupInterfaceChannelManager, ReadWriteBits)
{
  const char channel_config[] =
    R"(
      {
        index: 0x6071,
        sub_index: 0x00,
        type: bit8,
        data_mapping: [
          {
            type: bool,
            mask: 1,
            command_interface: input1
          },
          {
            type: bool,
            mask: 2,
            state_interface: output1
          },
          {
            type: bool,
            mask: 4,
            command_interface: input2
          },
          {
            type: bool,
            mask: 8,
            state_interface: output2
          },
          {
            type: bool,
            mask: 16,
            command_interface: input3
          },
          {
            type: bool,
            mask: 32,
            state_interface: output3
          }
        ]
      }
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoGroupInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  ASSERT_EQ(pdo_manager.load_from_config(config), true);

  ASSERT_EQ(pdo_manager.number_of_interfaces(), 7);
  ASSERT_EQ(pdo_manager.number_of_managed_interfaces(), 6);

  ASSERT_EQ(pdo_manager.data_type(0), "bit8");
  ASSERT_EQ(pdo_manager.interface_name(0), "null");
  ASSERT_EQ(pdo_manager.index, 0x6071);
  ASSERT_EQ(pdo_manager.sub_index, 0);

  ASSERT_EQ(pdo_manager.data_type(1), "bool");
  ASSERT_EQ(pdo_manager.interface_name(1), "input1");
  ASSERT_EQ(pdo_manager.data(1).mask, 1);
  ASSERT_EQ(pdo_manager.v_data[1].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(2), "bool");
  ASSERT_EQ(pdo_manager.interface_name(2), "output1");
  ASSERT_EQ(pdo_manager.data(2).mask, 2);
  ASSERT_EQ(pdo_manager.v_data[2].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(3), "bool");
  ASSERT_EQ(pdo_manager.interface_name(3), "input2");
  ASSERT_EQ(pdo_manager.data(3).mask, 4);
  ASSERT_EQ(pdo_manager.v_data[3].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(4), "bool");
  ASSERT_EQ(pdo_manager.interface_name(4), "output2");
  ASSERT_EQ(pdo_manager.data(4).mask, 8);
  ASSERT_EQ(pdo_manager.v_data[4].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(5), "bool");
  ASSERT_EQ(pdo_manager.interface_name(5), "input3");
  ASSERT_EQ(pdo_manager.data(5).mask, 16);
  ASSERT_EQ(pdo_manager.v_data[5].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(6), "bool");
  ASSERT_EQ(pdo_manager.interface_name(6), "output3");
  ASSERT_EQ(pdo_manager.data(6).mask, 32);
  ASSERT_EQ(pdo_manager.v_data[6].addr_offset, 0);

  uint8_t buffer[1];
  std::vector<uint8_t> write_tests0 =
  {
    0, 1, 2, 4, 8, 16, 32,
    0b00111111,
    0b11000000,
    0b11111111,
    0b00101010,
    0b11010101,
    0b00001111
  };

  for (size_t n = 0; n < write_tests0.size(); ++n) {
    EC_WRITE_U8(buffer, write_tests0[n]);
    ASSERT_EQ(pdo_manager.ec_read(buffer), write_tests0[n]);

    std::bitset<8> bits(write_tests0[n]);
    for (size_t i = 1; i < 7; i++) {
      ASSERT_EQ(pdo_manager.ec_read(buffer, i), bits.test(i - 1));
    }
  }
}
