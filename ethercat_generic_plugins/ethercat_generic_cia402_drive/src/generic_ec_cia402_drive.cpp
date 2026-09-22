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

#include <numeric>
#include <algorithm>
#include <array>
#include <ctime>
#include <filesystem>
#include <sstream>

#define _USE_MATH_DEFINES  // enable M_PI constant in cmath
#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ethercat_generic_plugins
{

namespace
{

/// CiA-402 Disable Operation: Switch On, Enable Voltage and Quick Stop set, Enable Operation
/// cleared. Takes the drive from Operation Enabled to Switched On, dropping the power stage so the
/// axis is held by its brake. This is the wind-down's default stop, because it needs nothing of the
/// drive beyond the CiA-402 state machine itself.
constexpr uint16_t CONTROL_WORD_DISABLE_OPERATION = 0b00000111;

/// CiA-402 Quick Stop: Enable Voltage set, Quick Stop cleared. The drive decelerates on its quick
/// stop ramp (0x6085) and then follows its quick stop option code (0x605A), which either drops it
/// into Switch On Disabled or holds it in Quick Stop Active.
///
/// Only sent to drives whose slave config sets `quick_stop_supported`. A drive that does not
/// implement Quick Stop can respond to it destructively: one drive whose datasheet listed the
/// function as not supported answered a Quick Stop from Operation Enabled by clearing status word
/// bit 12, abandoning the commanded position and accelerating the axis to roughly twice its
/// commanded velocity under its own torque, until it dropped out of Operation Enabled by itself.
constexpr uint16_t CONTROL_WORD_QUICK_STOP = 0b00001011;

/// CiA-402 Disable Voltage: every command bit cleared, which forces Switch On Disabled from any
/// energised state.
constexpr uint16_t CONTROL_WORD_DISABLE_VOLTAGE = 0b00000000;

double raw_value_from_channel(const ethercat_interface::EcPdoChannelManager & channel)
{
  const auto & d = channel.data();
  const double logged_value = d.last_value;

  // RxPDO = from PC to Device
  if (channel.pdo_type == ethercat_interface::RPDO) {
    // value is in already in raw space (i.e. already scaled by factor and offset to convert to raw)
    return logged_value;
  }

  // TxPDO = from Device to PC
  if (d.factor != 0.0) {
    // value has been converted to physical space before being stored,
    // so convert back to raw space
    return (logged_value - d.offset) / d.factor;
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
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "STATE: %s with status word :%d [slave pos: %u]",
        DEVICE_STATE_STR.at(state_).c_str(),
        status_word_,
        position_
      );
    }
  }

  latch_fault_error_code();
  if (state_ == STATE_OPERATION_ENABLED) {
    // From here on a fault is this session's, so it latches and waits for a deliberate reset.
    operation_enabled_reached_ = true;
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
      "EcCiA402Drive initialized: name=%s alias=%u position=%u raw_position=%f "
      "converted_position=%f joint_offset=%f",
      module_name_for_log(parameters_).c_str(),
      alias_,
      position_,
      last_raw_position_,
      last_position_,
      joint_offset_);
    initialization_position_logged_ = true;
  }
}

void EcCiA402Drive::latch_fault_error_code()
{
  // Deliberately kept across the reset that follows. A drive clears 0x603F within a cycle or two of
  // being acknowledged, and with auto_fault_reset the acknowledgement goes out on the very next cycle,
  // so reading the live object afterwards says only that the drive is no longer complaining. Whoever
  // has to explain the trip needs what it was complaining about.
  const bool in_fault = state_ == STATE_FAULT || state_ == STATE_FAULT_REACTION_ACTIVE;
  if (!in_fault) {
    return;
  }

  const bool was_in_fault = last_state_ == STATE_FAULT || last_state_ == STATE_FAULT_REACTION_ACTIVE;
  if (!was_in_fault) {
    // A new fault replaces the previous record rather than being discarded behind it.
    last_fault_error_code_ = error_code_;
    last_fault_status_word_ = status_word_;
    fault_error_code_logged_ = false;
  } else if (last_fault_error_code_ == 0 && error_code_ != 0) {
    // Drives do not all publish the error code and the fault bit on the same cycle, so the first
    // non-zero code seen while the fault stands is taken rather than only the one at the edge.
    last_fault_error_code_ = error_code_;
  }

  if (!fault_error_code_logged_ && last_fault_error_code_ != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "EcCiA402Drive: drive faulted with error code 0x%04x, status word 0x%04x [slave pos: %u]. "
      "The code is latched on the last_error_code state interface; the drive's own 0x603F is cleared "
      "by the reset that follows.",
      last_fault_error_code_,
      last_fault_status_word_,
      position_);
    fault_error_code_logged_ = true;
  }
}

void EcCiA402Drive::publish_last_error_code()
{
  if (last_error_code_state_interface_index_ < 0 || state_interface_ptr_ == nullptr) {
    return;
  }
  if (static_cast<size_t>(last_error_code_state_interface_index_) >= state_interface_ptr_->size()) {
    return;
  }
  state_interface_ptr_->at(last_error_code_state_interface_index_) =
    static_cast<double>(last_fault_error_code_);
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

  csv_dump_file_ << "timestamp_ns,cycle,phase,is_operational";
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

  csv_dump_file_ << timestamp_ns << "," << csv_cycle_counter_ << "," << process_phase() << ","
                 << is_operational_;
  for (const auto domain_idx : csv_rpdo_domain_indices_) {
    const auto channel_idx = domain_map_[domain_idx];
    const auto * channel = pdo_channels_info_[channel_idx];
    csv_dump_file_ << "," << raw_value_from_channel(*channel);
  }
  for (const auto domain_idx : csv_tpdo_domain_indices_) {
    const auto channel_idx = domain_map_[domain_idx];
    const auto * channel = pdo_channels_info_[channel_idx];
    csv_dump_file_ << "," << raw_value_from_channel(*channel);
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
    if (is_operational_ && wind_down_requested_) {
      // The wind-down owns the control word: a fault reset or an automatic transition back up to
      // Operation Enabled would undo the very thing it is trying to achieve, and a control word
      // left behind by a controller that has already stopped must not be replayed either.
      channel.default_value = wind_down_transition(state_);
      channel.override_command = true;
    } else if (is_operational_) {
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
    // The wind-down holds the last read position, so a setpoint left behind by a controller that
    // has already stopped cannot be replayed into a drive that is being brought down.
    //
    // A drive that is not in Operation Enabled is held the same way, and for the same reason one step
    // earlier: while it is in Fault, or walking back up through Switch On Disabled after a reset, the
    // axis is free and moves. Whatever setpoint the command interface holds was written before that and
    // no longer describes where the axis is, so commanding it at the moment the power stage comes back
    // steps the axis to it. Holding the last read position throughout means the drive re-enables onto
    // the position it is actually at.
    const bool follow_position_command = !wind_down_requested_ &&
      state_ == STATE_OPERATION_ENABLED &&
      mode_of_operation_display_ == ModeOfOperation::MODE_CYCLIC_SYNC_POSITION;
    channel.override_command = !follow_position_command;

    if (follow_position_command) {
      channel.ec_read_to_interface(domain_address);

      if (joint_offset_startup_wrap_enabled_ && !joint_offset_startup_wrap_applied_) {
        // Fallback to default value (last read position)
        // while waiting for joint offset to be computed and applied
        channel.ec_write(domain_address, std::numeric_limits<double>::quiet_NaN());
        return;
      }

      if (command_interface_ptr_ != nullptr &&
        channel.has_command_interface_name() &&
        channel.is_command_interface_defined() &&
        channel.command_interface_index(0) < command_interface_ptr_->size())
      {
        // These lines mimic the behavior of channel.ec_update(),
        // but apply the joint offset to the command position before writing it to the PDO
        const double command_position =
          command_interface_ptr_->at(channel.command_interface_index(0));
        channel.ec_write(domain_address, command_position - joint_offset_);
        return;
      }
    }
  }

  // setup mode of operation
  if (channel.index == CiA402D_RPDO_MODE_OF_OPERATION) {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
      channel.default_value = mode_of_operation_;
    }
  }

  // Everything else the drive is commanded with falls back to its configured default while the
  // wind-down runs, and whenever the drive is not in Operation Enabled: zero velocity, zero torque, the
  // mode of operation the drive is already in.
  //
  // Assigned every cycle rather than only set, because override_command lives on the channel and outlives
  // the condition that raised it. Setting it on the way up, which every bring-up does before the drive
  // first reaches Operation Enabled, and never clearing it pins the channel to its default for the rest
  // of the session: the drive then ignores its velocity and torque commands for good.
  //
  // The control word is excluded because the state machine and the wind-down drive it themselves, just
  // above, and pinning it would strand the drive wherever it happens to be. The target position is
  // excluded because it assigns its own override from `follow_position_command`, which already carries
  // both conditions.
  if (channel.pdo_type == ethercat_interface::RPDO &&
    channel.index != CiA402D_RPDO_CONTROLWORD &&
    channel.index != CiA402D_RPDO_POSITION)
  {
    channel.override_command = wind_down_requested_ || state_ != STATE_OPERATION_ENABLED;
  }

  if (channel.index == CiA402D_TPDO_POSITION) {
    // For position feedback, we need to read the value from the device and apply the joint offset,
    // and not just update the interfaces directly with the read value
    channel.ec_read(domain_address);
  } else {
    channel.ec_update(domain_address);
  }

  // get mode_of_operation_display_
  if (channel.index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = channel.last_value;
  }

  if (channel.index == CiA402D_TPDO_POSITION) {
    last_raw_position_ = raw_value_from_channel(channel);
    bool update_position_state = true;
    if (joint_offset_startup_wrap_enabled_ && !joint_offset_startup_wrap_applied_) {
      // A non-zero status word indicates that data has been received from the slave.
      // TxPDO payload are already valid in SAFEOP, so this fires as early as possible
      // while still waiting for a valid current position value to be available.
      if (status_word_ != 0) {
        const double candidate_position = channel.last_value + joint_offset_;

        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Joint offset startup wrap enabled for pos=%u. "
          "Joint offset before wrapping = %f resulting in candidate position = %f",
          position_,
          joint_offset_,
          candidate_position);

        constexpr auto wrap_to_pi = [](const double angle) {
            constexpr double POSITION_WRAP_PERIOD_RAD = 2.0 * M_PI;
            return std::remainder(angle, POSITION_WRAP_PERIOD_RAD);
          };
        joint_offset_ += wrap_to_pi(candidate_position) - candidate_position;

        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Joint offset after wrapping = %f",
          joint_offset_);

        joint_offset_startup_wrap_applied_ = true;
      } else {
        update_position_state = false;
      }
    }
    if (update_position_state) {
      last_position_ = channel.last_value + joint_offset_;

      if (state_interface_ptr_ != nullptr &&
        channel.has_state_interface_name() &&
        channel.is_state_interface_defined() &&
        channel.state_interface_index(0) < state_interface_ptr_->size())
      {
        state_interface_ptr_->at(channel.state_interface_index(0)) = last_position_;
      }
    }
  }

  // Special case: StatusWord
  if (channel.index == CiA402D_TPDO_STATUSWORD) {
    status_word_ = channel.last_value;
  }

  // Special case: Error Code. Read every cycle so the latch below has the live value to take when the
  // drive raises the fault bit.
  if (channel.index == CiA402D_TPDO_ERROR_CODE) {
    error_code_ = static_cast<uint16_t>(channel.last_value);
  }


  // CHECK FOR STATE CHANGE
  if (entry_idx == domain_map_.size() - 1) {  // if last entry in domain
    updateState();
    publish_last_error_code();
    if (wind_down_requested_ && !wind_down_complete_) {
      ++wind_down_cycles_;
    }
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
  joint_offset_startup_wrap_enabled_ = false;
  joint_offset_startup_wrap_applied_ = false;

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

  if (parameters_.find("joint_offset_startup_wrap_enabled") != parameters_.end()) {
    const std::string & value = parameters_["joint_offset_startup_wrap_enabled"];
    joint_offset_startup_wrap_enabled_ = (value == "true" || value == "1" || value == "True");
  }

  if (parameters_.find("command_interface/reset_fault") != parameters_.end()) {
    const std::string & value = parameters_["command_interface/reset_fault"];
    try {
      fault_reset_command_interface_index_ = std::stoi(value);
    } catch (const std::invalid_argument &) {
      RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: failed to parse parameter 'command_interface/reset_fault' "
            "with value '%s'",
            value.c_str());
    } catch (const std::out_of_range &) {
      RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: parameter 'command_interface/reset_fault' out of range with value '%s'",
            value.c_str());
    }
  }

  if (parameters_.find("state_interface/last_error_code") != parameters_.end()) {
    const std::string & value = parameters_["state_interface/last_error_code"];
    try {
      last_error_code_state_interface_index_ = std::stoi(value);
    } catch (const std::invalid_argument &) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"),
        "EcCiA402Drive: failed to parse parameter 'state_interface/last_error_code' with value '%s'",
        value.c_str());
    } catch (const std::out_of_range &) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"),
        "EcCiA402Drive: parameter 'state_interface/last_error_code' out of range with value '%s'",
        value.c_str());
    }
  }

  // A drive that has not faulted yet reports zero rather than NaN, so a reader can tell "no fault on
  // record" from "this interface is not mapped", which stays NaN because nothing writes it.
  last_fault_error_code_ = 0;
  last_fault_status_word_ = 0;
  fault_error_code_logged_ = false;
  // Re-armed per activation, so a deactivate/activate cycle clears a fault the same way a fresh start
  // does rather than coming back up into one.
  operation_enabled_reached_ = false;
  startup_fault_reset_logged_ = false;
  publish_last_error_code();

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
  if (drive_config["reset_fault_on_startup"]) {
    reset_fault_on_startup_ = drive_config["reset_fault_on_startup"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }
  if (drive_config["quick_stop_supported"]) {
    quick_stop_supported_ = drive_config["quick_stop_supported"].as<bool>();
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
      {
        // A drive that comes up in Fault is cleared once regardless, because the fault belongs to a
        // session that has ended and nothing else can clear it before Operation Enabled is first
        // reached: with auto_fault_reset off the reset comes from a command interface, and no
        // controller is claiming one yet.
        const bool startup_reset = reset_fault_on_startup_ && !operation_enabled_reached_;
        if (startup_reset && !startup_fault_reset_logged_) {
          RCLCPP_WARN(
            rclcpp::get_logger("EthercatDriver"),
            "EcCiA402Drive: the drive came up in Fault with error code 0x%04x; clearing it once on the "
            "way to Operation Enabled [slave pos: %u]",
            last_fault_error_code_,
            position_);
          startup_fault_reset_logged_ = true;
        }
        if (auto_fault_reset_ || fault_reset_ || startup_reset) {
          fault_reset_ = false;
          return (control_word & 0b11111111) | 0b10000000;     // automatic reset
        }
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

void EcCiA402Drive::start_wind_down(double cycle_period_s, double timeout_s)
{
  // Taken before the first wind-down cycle forces them all true, because override_command lives
  // on the channel rather than on the wind-down: without this the drive would come back up with
  // every command channel still pinned to its default. Cast the way processData() does, since
  // that is the view through which the wind-down sets the flag in the first place.
  pre_wind_down_override_command_.clear();
  pre_wind_down_override_command_.reserve(pdo_channels_info_.size());
  for (auto * channel : pdo_channels_info_) {
    auto * single_channel =
      static_cast<ethercat_interface::EcPdoSingleInterfaceChannelManager *>(channel);
    pre_wind_down_override_command_.push_back(single_channel->override_command);
  }

  wind_down_requested_ = true;
  wind_down_cycles_ = 0;
  wind_down_complete_ = false;

  // Half of the budget is spent letting the drive decelerate on its quick stop ramp. Drives whose
  // quick stop option code takes them to Switch On Disabled finish well inside that and end the
  // wind-down early; the ones configured to hold position in Quick Stop Active never would, so the
  // remaining half is left for Disable Voltage to be commanded and take effect. Unused when the
  // drive does not support Quick Stop, since that path never enters Quick Stop Active.
  quick_stop_hold_cycles_ = (cycle_period_s > 0.0)
    ? static_cast<uint32_t>(0.5 * timeout_s / cycle_period_s)
    : 0;

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "EcCiA402Drive: winding down from %s using %s [slave pos: %u]",
    DEVICE_STATE_STR.at(state_).c_str(),
    quick_stop_supported_ ? "Quick Stop" : "Disable Operation",
    position_);
}

bool EcCiA402Drive::wind_down_complete()
{
  // A drive that is not operational is either already de-energised or no longer reachable over
  // process data: either way the wind-down has nothing left to do and must not hold up the caller.
  return wind_down_complete_ || !is_operational_;
}

void EcCiA402Drive::reset_wind_down()
{
  if (!wind_down_requested_) {
    return;
  }

  // Put every channel back the way the slave config had it. Anything the wind-down did not
  // snapshot is left alone, so a channel added since is not silently re-configured here.
  const size_t restored =
    std::min(pdo_channels_info_.size(), pre_wind_down_override_command_.size());
  for (size_t i = 0; i < restored; ++i) {
    auto * single_channel =
      static_cast<ethercat_interface::EcPdoSingleInterfaceChannelManager *>(pdo_channels_info_[i]);
    single_channel->override_command = pre_wind_down_override_command_[i];
  }
  pre_wind_down_override_command_.clear();

  wind_down_requested_ = false;
  wind_down_complete_ = true;
  wind_down_cycles_ = 0;
  quick_stop_hold_cycles_ = 0;

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "EcCiA402Drive: wind-down state cleared, the drive can be commanded again [slave pos: %u]",
    position_);
}

/** returns the control word that walks the device down towards Switch On Disabled */
uint16_t EcCiA402Drive::wind_down_transition(DeviceState state)
{
  switch (state) {
    case STATE_OPERATION_ENABLED:
      // Quick Stop decelerates on the drive's own ramp and is the better stop where the drive
      // implements it; disabling drops the power stage and leaves the axis to its brake.
      return quick_stop_supported_ ? CONTROL_WORD_QUICK_STOP : CONTROL_WORD_DISABLE_OPERATION;
    case STATE_QUICK_STOP_ACTIVE:
      // Still decelerating under power, so this is not somewhere to leave the drive.
      if (quick_stop_supported_ && wind_down_cycles_ < quick_stop_hold_cycles_) {
        return CONTROL_WORD_QUICK_STOP;
      }
      return CONTROL_WORD_DISABLE_VOLTAGE;
    case STATE_FAULT_REACTION_ACTIVE:
      // The drive is running its fault reaction: still decelerating under power, and it reaches
      // Fault on its own once that finishes. Completing here would release the master onto a
      // moving axis, which is the failure this whole loop exists to avoid, so keep the frames
      // going until the drive leaves the state. Unlike Quick Stop Active this needs no budget of
      // its own: the reaction is transient by specification, so a drive still here when the
      // caller's timeout expires has something wrong with it and the warning is earned.
      return CONTROL_WORD_DISABLE_VOLTAGE;
    default:
      // Every other state has the drive function disabled, which is the whole point of winding
      // down before the frames stop. Disable Voltage still goes out on this cycle, so the drive
      // carries that command down to Switch On Disabled by itself; waiting to observe it would
      // cost the caller its entire timeout on a drive that parks in Ready to Switch On while its
      // DC bus is live, which some drives do. A standing fault is deliberately not reset:
      // clearing it on the way out would hide it from the next start-up.
      wind_down_complete_ = true;
      return CONTROL_WORD_DISABLE_VOLTAGE;
  }
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
