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

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

#include <rclcpp/rclcpp.hpp>

#include <numeric>
#include <algorithm>
#include <array>
#include <ctime>
#include <filesystem>
#include <sstream>


namespace ethercat_generic_plugins
{

namespace
{

double raw_value_for_csv(const ethercat_interface::EcPdoChannelManager & channel)
{
  const auto & d = channel.data();
  const double logged_value = d.last_value;

  // RxPDO = from PC to Device
  if (channel.pdo_type == ethercat_interface::RPDO) {
    return logged_value; // value is in already in raw space (i.e. already scaled by factor and offset to convert to raw)
  }

  // TxPDO = from Device to PC
  if (d.factor != 0.0) {
    return (logged_value - d.offset) / d.factor; // value has been converted to physical space before being stored, so convert back to raw space
  }

  return logged_value;
}

std::string module_name_for_log(const std::unordered_map<std::string, std::string> & parameters)
{
  const auto name_it = parameters.find("name");
  if (name_it != parameters.end()) {
    return name_it->second;
  }

  return "<unknown>";
}

}  // namespace

EcCiA402Drive::EcCiA402Drive()
: GenericEcSlave() {}

EcCiA402Drive::~EcCiA402Drive()
{
  if (csv_dump_file_.is_open()) {
    csv_dump_file_.close();
  }
}

bool EcCiA402Drive::initialized() {return initialized_;}

void EcCiA402Drive::updateState()
{
  if (status_word_ != last_status_word_) {
    state_ = deviceState(status_word_);
    if (state_ != last_state_) {
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "STATE: %s with status word :%d",
        DEVICE_STATE_STR.at(state_).c_str(),
        status_word_
      );
    }
  }
  last_status_word_ = status_word_;
  last_state_ = state_;
  counter_++;
  initialized_ = is_operational_;

  if (initialized_ && !initialization_position_logged_ && !std::isnan(last_position_) &&
    !std::isnan(last_raw_position_))
  {
    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "EcCiA402Drive initialized: name=%s alias=%u position=%u raw_position=%f converted_position=%f joint_offset=%f",
      module_name_for_log(parameters_).c_str(),
      alias_,
      position_,
      last_raw_position_,
      last_position_,
      joint_offset_);
    initialization_position_logged_ = true;
  }
}

void EcCiA402Drive::setup_csv_dump()
{
  if (parameters_.find("csv_dump_enabled") != parameters_.end()) {
    const std::string & value = parameters_["csv_dump_enabled"];
    csv_dump_enabled_ = (value == "true" || value == "1" || value == "True");
  }

  if (!csv_dump_enabled_) {
    return;
  }

  if (parameters_.find("csv_dump_path") != parameters_.end()) {
    csv_dump_path_ = parameters_["csv_dump_path"];
  } else {
    std::array<char, 64> timestamp_buffer{};
    std::time_t now = std::time(nullptr);
    std::tm tm_now;
    if (nullptr != localtime_r(&now, &tm_now) &&
      0 < std::strftime(
        timestamp_buffer.data(),
        timestamp_buffer.size(),
        "log_%Y%m%d_%H%M%S",
        &tm_now))
    {
      std::stringstream path;
      path << "logs/" << timestamp_buffer.data()
           << "_cia402_a" << alias_ << "_p" << position_ << ".csv";
      csv_dump_path_ = path.str();
    } else {
      std::stringstream path;
      path << "logs/log_cia402_a" << alias_ << "_p" << position_ << ".csv";
      csv_dump_path_ = path.str();
    }
  }

  if (parameters_.find("csv_dump_flush_every_n") != parameters_.end()) {
    csv_flush_every_n_ = std::max(
      static_cast<std::size_t>(1),
      static_cast<std::size_t>(std::stoul(parameters_["csv_dump_flush_every_n"])));
  }

  csv_rpdo_domain_indices_.clear();
  csv_tpdo_domain_indices_.clear();
  for (std::size_t domain_idx = 0; domain_idx < domain_map_.size(); ++domain_idx) {
    const auto channel_idx = domain_map_[domain_idx];
    const auto * channel = pdo_channels_info_[channel_idx];
    if (channel->pdo_type == ethercat_interface::RPDO) {
      csv_rpdo_domain_indices_.push_back(domain_idx);
    } else if (channel->pdo_type == ethercat_interface::TPDO) {
      csv_tpdo_domain_indices_.push_back(domain_idx);
    }
  }

  const auto csv_parent_dir = std::filesystem::path(csv_dump_path_).parent_path();
  if (!csv_parent_dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(csv_parent_dir, ec);
  }

  csv_dump_file_.open(csv_dump_path_, std::ios::out | std::ios::trunc);
  if (!csv_dump_file_.is_open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "EcCiA402Drive: failed to open CSV dump file: %s",
      csv_dump_path_.c_str());
    csv_dump_enabled_ = false;
    return;
  }

  csv_dump_file_ << "timestamp_ns,cycle,phase";
  for (const auto domain_idx : csv_rpdo_domain_indices_) {
    const auto channel_idx = domain_map_[domain_idx];
    const auto * channel = pdo_channels_info_[channel_idx];
    csv_dump_file_ << ",rpdo_"
                   << channel->index_hex_str() << "_"
                   << channel->sub_index_hex_str() << "_"
                   << channel->interface_name();
  }
  for (const auto domain_idx : csv_tpdo_domain_indices_) {
    const auto channel_idx = domain_map_[domain_idx];
    const auto * channel = pdo_channels_info_[channel_idx];
    csv_dump_file_ << ",tpdo_"
                   << channel->index_hex_str() << "_"
                   << channel->sub_index_hex_str() << "_"
                   << channel->interface_name();
  }
  csv_dump_file_ << "\n";
  csv_header_written_ = true;
  csv_t0_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "EcCiA402Drive: CSV dump enabled, output: %s",
    csv_dump_path_.c_str());
}

void EcCiA402Drive::dump_cycle_csv_row()
{
  if (!csv_dump_enabled_ || !csv_header_written_ || !csv_dump_file_.is_open()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now - csv_t0_).count();

  csv_dump_file_ << timestamp_ns << "," << csv_cycle_counter_ << "," << process_phase();
  for (const auto domain_idx : csv_rpdo_domain_indices_) {
    const auto channel_idx = domain_map_[domain_idx];
    const auto * channel = pdo_channels_info_[channel_idx];
    csv_dump_file_ << "," << raw_value_for_csv(*channel);
  }
  for (const auto domain_idx : csv_tpdo_domain_indices_) {
    const auto channel_idx = domain_map_[domain_idx];
    const auto * channel = pdo_channels_info_[channel_idx];
    csv_dump_file_ << "," << raw_value_for_csv(*channel);
  }
  csv_dump_file_ << "\n";
  ++csv_cycle_counter_;

  if (csv_flush_every_n_ <= 1 || (csv_cycle_counter_ % csv_flush_every_n_) == 0U) {
    csv_dump_file_.flush();
  }
}

void EcCiA402Drive::processData(size_t entry_idx, uint8_t * domain_address)
{
  auto index = domain_map_[entry_idx];
  ethercat_interface::EcPdoSingleInterfaceChannelManager * channel_ptr =
    static_cast<
    ethercat_interface::EcPdoSingleInterfaceChannelManager *>(
    pdo_channels_info_[index]);
  ethercat_interface::EcPdoSingleInterfaceChannelManager & channel(*channel_ptr);
  // Special case: ControlWord
  if (channel.index == CiA402D_RPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
          !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      if (auto_state_transitions_) {
        channel.default_value = transition(
          state_,
          channel.ec_read(domain_address));
      }
    }
  }

  // setup current position as default position
  if (channel.index == CiA402D_RPDO_POSITION) {
    if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE) {
      channel.default_value =
        channel.factor * (last_position_ - joint_offset_) + channel.offset;
    }
    channel.override_command =
      (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;

    if (mode_of_operation_display_ == ModeOfOperation::MODE_CYCLIC_SYNC_POSITION &&
      command_interface_ptr_ != nullptr &&
      channel.has_command_interface_name() &&
      channel.is_command_interface_defined() &&
      channel.command_interface_index(0) < command_interface_ptr_->size())
    {
      const double command_position = command_interface_ptr_->at(channel.command_interface_index(0));
      channel.ec_read_to_interface(domain_address);
      channel.ec_write(domain_address, command_position - joint_offset_);
      return;
    }
  }

  // setup mode of operation
  if (channel.index == CiA402D_RPDO_MODE_OF_OPERATION) {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
      channel.default_value = mode_of_operation_;
    }
  }

  channel.ec_update(domain_address);

  // get mode_of_operation_display_
  if (channel.index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = channel.last_value;
  }

  if (channel.index == CiA402D_TPDO_POSITION) {
    last_raw_position_ = raw_value_for_csv(channel);
    last_position_ = channel.last_value + joint_offset_;
    if (state_interface_ptr_ != nullptr &&
      channel.has_state_interface_name() &&
      channel.is_state_interface_defined() &&
      channel.state_interface_index(0) < state_interface_ptr_->size())
    {
      state_interface_ptr_->at(channel.state_interface_index(0)) = last_position_;
    }
  }

  // Special case: StatusWord
  if (channel.index == CiA402D_TPDO_STATUSWORD) {
    status_word_ = channel.last_value;
  }


  // CHECK FOR STATE CHANGE
  if (entry_idx == domain_map_.size() - 1) {  // if last entry in domain
    updateState();
    dump_cycle_csv_row();
  }
}

bool EcCiA402Drive::setupSlave(
  std::unordered_map<std::string, std::string> slave_parameters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  parameters_ = slave_parameters;
  initialization_position_logged_ = false;
  last_raw_position_ = std::numeric_limits<double>::quiet_NaN();
  last_position_ = std::numeric_limits<double>::quiet_NaN();

  if (parameters_.find("slave_config") != parameters_.end()) {
    if (!setup_from_config_file(parameters_["slave_config"])) {
      return false;
    }
  } else {
      RCLCPP_ERROR(
          rclcpp::get_logger("EthercatDriver"),
          "EcCiA402Drive: failed to find 'slave_config' tag in URDF.");
    return false;
  }

  setup_interface_mapping();
  setup_syncs();

if (parameters_.find("mode_of_operation") != parameters_.end()) {
    const std::string & value = parameters_["mode_of_operation"];
    try {
        mode_of_operation_ = std::stod(value);
    } catch (const std::invalid_argument &) {
        RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: failed to parse parameter 'mode_of_operation' with value '%s'",
            value.c_str());
    } catch (const std::out_of_range &) {
        RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: parameter 'mode_of_operation' out of range with value '%s'",
            value.c_str());
    }
}

if (parameters_.find("joint_offset") != parameters_.end()) {
    const std::string & value = parameters_["joint_offset"];
    try {
        joint_offset_ = std::stod(value);
    } catch (const std::invalid_argument &) {
        RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: failed to parse parameter 'joint_offset' with value '%s'",
            value.c_str());
    } catch (const std::out_of_range &) {
        RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: parameter 'joint_offset' out of range with value '%s'",
            value.c_str());
    }
}

if (parameters_.find("command_interface/reset_fault") != parameters_.end()) {
    const std::string & value = parameters_["command_interface/reset_fault"];
    try {
        fault_reset_command_interface_index_ = std::stoi(value);
    } catch (const std::invalid_argument &) {
        RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: failed to parse parameter 'command_interface/reset_fault' with value '%s'",
            value.c_str());
    } catch (const std::out_of_range &) {
        RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: parameter 'command_interface/reset_fault' out of range with value '%s'",
            value.c_str());
    }
}

  setup_csv_dump();

  return true;
}

bool EcCiA402Drive::setup_from_config(YAML::Node drive_config)
{
  if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
  // additional configuration parameters for CiA402 Drives
  if (drive_config["auto_fault_reset"]) {
    auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }
  return true;
}

bool EcCiA402Drive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "EcCiA402Drive: failed to load drive configuration: %s",
      ex.what());
    return false;
  } catch (const YAML::BadFile & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "EcCiA402Drive: failed to load drive configuration: %s",
      ex.what());
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

/** returns device state based upon the status_word */
DeviceState EcCiA402Drive::deviceState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return STATE_NOT_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return STATE_SWITCH_ON_DISABLED;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return STATE_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return STATE_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return STATE_OPERATION_ENABLED;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return STATE_QUICK_STOP_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return STATE_FAULT_REACTION_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  }
  return STATE_UNDEFINED;
}

/** returns the control word that will take device from state to next desired state */
uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word)
{
  switch (state) {
    case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
      return (control_word & 0b01111110) | 0b00000110;
    case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
      return (control_word & 0b01110111) | 0b00000111;
    case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_OPERATION_ENABLED:         // -> GOOD
      return control_word;
    case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
