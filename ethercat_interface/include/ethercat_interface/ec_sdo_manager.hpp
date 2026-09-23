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

#ifndef ETHERCAT_INTERFACE__EC_SDO_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_SDO_MANAGER_HPP_

#include <ecrt.h>
#include <cstring>
#include <string>
#include <vector>
#include <limits>

#include "yaml-cpp/yaml.h"

namespace ethercat_interface
{

/** Where a PDO channel's conversion factor comes from, when the drive knows it and the config
 * does not.
 *
 * CiA-402 expresses several cyclic values as a fraction of a rating the drive stores in an SDO:
 * current actual value in thousandths of the motor rated current, torque actual value in
 * thousandths of the motor rated torque. Writing the resulting factor into the slave config means
 * restating a number the drive already holds, and a config that restates it is silently wrong on a
 * drive whose rating differs.
 *
 * `scale` converts the value read from the drive into the factor: for a current in mA read into an
 * interface in amperes, thousandths of it is `1e-6`.
 */
struct SdoFactorSource
{
  /// The slave config asked for a factor from the drive. Set as soon as the key is present, so a
  /// source that fails to parse is still known about and cannot quietly leave the channel at its
  /// default factor.
  bool configured = false;
  /// Every field the read needs was present and sensible.
  bool valid = false;
  uint16_t index = 0;
  uint8_t sub_index = 0;
  std::string data_type;
  double scale = 1.0;
  /// True when the slave config also declared a literal `factor`, which is then the fallback. A
  /// channel's factor defaults to 1, so "no literal" cannot be told from the value itself.
  bool has_literal_fallback = false;
  /// The literal itself, kept apart from the channel's factor. Resolution runs on every activation
  /// and overwrites the channel's factor, so without this a later failed read would fall back on
  /// whatever the previous activation read, possibly from a different drive, rather than on the
  /// config.
  double literal_factor = 1.0;
};

class SdoConfigEntry
{
public:
  SdoConfigEntry() {}
  ~SdoConfigEntry() {}

  void buffer_write(uint8_t * buffer)
  {
    if (data_type == "uint8") {
      EC_WRITE_U8(buffer, static_cast<uint8_t>(data));
    } else if (data_type == "int8") {
      EC_WRITE_S8(buffer, static_cast<int8_t>(data));
    } else if (data_type == "uint16") {
      EC_WRITE_U16(buffer, static_cast<uint16_t>(data));
    } else if (data_type == "int16") {
      EC_WRITE_S16(buffer, static_cast<int16_t>(data));
    } else if (data_type == "uint32" || data_type == "real32" || data_type == "float") {
      EC_WRITE_U32(buffer, static_cast<uint32_t>(data));
    } else if (data_type == "int32") {
      EC_WRITE_S32(buffer, static_cast<int32_t>(data));
    } else if (data_type == "uint64" || data_type == "real64" || data_type == "double") {
      EC_WRITE_U64(buffer, static_cast<uint64_t>(data));
    } else if (data_type == "int64") {
      EC_WRITE_S64(buffer, static_cast<int64_t>(data));
    }
  }

  bool load_from_config(YAML::Node sdo_config)
  {
    // index
    if (sdo_config["index"]) {
      index = sdo_config["index"].as<uint16_t>();
    } else {
      std::cerr << "missing sdo index info" << std::endl;
      return false;
    }
    // sub_index
    if (sdo_config["sub_index"]) {
      sub_index = sdo_config["sub_index"].as<uint8_t>();
    } else {
      std::cerr << "sdo " << index << ": missing sdo info" << std::endl;
      return false;
    }
    // data type
    if (sdo_config["type"]) {
      data_type = sdo_config["type"].as<std::string>();
    } else {
      std::cerr << "sdo " << index << ": missing sdo data type info" << std::endl;
      return false;
    }
    // value
    if (sdo_config["value"]) {
      if (data_type == "float" || data_type == "real32") {
        float floatvalue = sdo_config["value"].as<float>();
        data = *reinterpret_cast<int *>(&floatvalue);
      } else if (data_type == "double" || data_type == "real64") {
        double doublevalue = sdo_config["value"].as<double>();
        data = *reinterpret_cast<int *>(&doublevalue);
      } else {
        data = sdo_config["value"].as<int>();
      }
    } else {
      std::cerr << "sdo " << index << ": missing sdo value" << std::endl;
      return false;
    }

    return true;
  }

  size_t data_size()
  {
    return type2bytes(data_type);
  }

  /// True for a type buffer_read() decodes. For checking a type when the config is parsed, rather
  /// than finding out from a read that fails only once the drive has been reached.
  static bool is_supported_type(const std::string & data_type)
  {
    return type2bytes(data_type) != 0;
  }

  /// Decode `size` bytes of `buffer` as `data_type`. Returns false for a type this does not handle.
  static bool buffer_read(
    const uint8_t * buffer, size_t size, const std::string & data_type,
    double * value)
  {
    if (value == nullptr || buffer == nullptr) {
      return false;
    }
    if (data_type == "uint8" && size >= 1) {
      *value = static_cast<double>(EC_READ_U8(buffer));
    } else if (data_type == "int8" && size >= 1) {
      *value = static_cast<double>(EC_READ_S8(buffer));
    } else if (data_type == "uint16" && size >= 2) {
      *value = static_cast<double>(EC_READ_U16(buffer));
    } else if (data_type == "int16" && size >= 2) {
      *value = static_cast<double>(EC_READ_S16(buffer));
    } else if (data_type == "uint32" && size >= 4) {
      *value = static_cast<double>(EC_READ_U32(buffer));
    } else if (data_type == "int32" && size >= 4) {
      *value = static_cast<double>(EC_READ_S32(buffer));
    } else if ((data_type == "float" || data_type == "real32") && size >= 4) {
      const uint32_t raw = EC_READ_U32(buffer);
      float decoded = 0.0f;
      std::memcpy(&decoded, &raw, sizeof(decoded));
      *value = static_cast<double>(decoded);
    } else if (data_type == "uint64" && size >= 8) {
      *value = static_cast<double>(EC_READ_U64(buffer));
    } else if (data_type == "int64" && size >= 8) {
      *value = static_cast<double>(EC_READ_S64(buffer));
    } else if ((data_type == "double" || data_type == "real64") && size >= 8) {
      const uint64_t raw = EC_READ_U64(buffer);
      double decoded = 0.0;
      std::memcpy(&decoded, &raw, sizeof(decoded));
      *value = decoded;
    } else {
      return false;
    }
    return true;
  }

  uint16_t index;
  uint8_t sub_index;
  std::string data_type;
  int data;

private:
  static size_t type2bytes(std::string type)
  {
    if (type == "int8" || type == "uint8") {
      return 1;
    } else if (type == "int16" || type == "uint16") {
      return 2;
    } else if (type == "int32" || type == "uint32" || type == "float" || type == "real32") {
      return 4;
    } else if (type == "int64" || type == "uint64" || type == "double" || type == "real64") {
      return 8;
    }
    return 0;
  }
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SDO_MANAGER_HPP_
