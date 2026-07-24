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

#include "ethercat_driver/ethercat_driver.hpp"
#include "ethercat_driver/loader_backed_transmission_coupling.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
void cleanup_master(
  std::shared_ptr<ethercat_interface::EcMaster> & master,
  bool & activated)
{
  activated = false;

  if (master) {
    master->shutdown();
    master.reset();
  }
}

/// Human-readable name for an EtherCAT application-layer (AL) state value.
std::string al_state_to_string(uint8_t al_state)
{
  switch (al_state) {
    case 1: return "INIT";
    case 2: return "PREOP";
    case 3: return "BOOT";
    case 4: return "SAFEOP";
    case 8: return "OP";
    default: {
      char buffer[16];
      std::snprintf(buffer, sizeof(buffer), "0x%02X", al_state);
      return std::string(buffer);
    }
  }
}

/// Human-readable description for a subset of the standard EtherCAT AL status codes (ETG.1000).
const char * al_status_code_to_string(uint16_t code)
{
  switch (code) {
    case 0x0000: return "No error";
    case 0x0001: return "Unspecified error";
    case 0x0002: return "No memory";
    case 0x0011: return "Invalid requested state change";
    case 0x0012: return "Unknown requested state";
    case 0x0013: return "Bootstrap not supported";
    case 0x0014: return "No valid firmware";
    case 0x0016: return "Invalid mailbox configuration";
    case 0x0017: return "Invalid sync manager configuration";
    case 0x0018: return "No valid inputs available";
    case 0x001B: return "Sync manager watchdog";
    case 0x001D: return "Invalid output configuration";
    case 0x001E: return "Invalid input configuration";
    case 0x0024: return "Invalid FMMU configuration";
    case 0x0028: return "DC PLL sync error";
    case 0x0029: return "DC sync IO error";
    case 0x002A: return "DC sync timeout";
    case 0x002C: return "Fatal sync error";
    case 0x002D: return "No sync error";
    case 0x0030: return "Invalid DC sync configuration";
    case 0x0032: return "DC sync error";
    case 0x0035: return "DC sync out of range";
    default: return "Unknown";
  }
}
}  // namespace

namespace ethercat_driver
{

namespace
{

uint16_t module_position_from_parameters(const std::unordered_map<std::string, std::string> & module_parameters)
{
  return static_cast<uint16_t>(std::stoul(module_parameters.at("position")));
}

void validate_module_parameter_alignment(
  const std::vector<std::shared_ptr<ethercat_interface::EcSlave>> & modules,
  const std::vector<std::unordered_map<std::string, std::string>> & module_parameters)
{
  if (modules.size() != module_parameters.size()) {
    throw std::runtime_error(
            "EtherCAT module list and module parameter list have different sizes: modules=" +
            std::to_string(modules.size()) + ", parameters=" + std::to_string(module_parameters.size()));
  }

  for (auto i = 0ul; i < modules.size(); ++i) {
    const auto parameter_position = module_position_from_parameters(module_parameters[i]);
    if (modules[i]->position_ != parameter_position) {
      throw std::runtime_error(
              "EtherCAT module position mismatch for module '" + module_parameters[i].at("name") +
              "': module position=" + std::to_string(modules[i]->position_) +
              ", parameter position=" + std::to_string(parameter_position));
    }
  }
}

void log_module_mapping(
  const std::vector<std::shared_ptr<ethercat_interface::EcSlave>> & modules,
  const std::vector<std::unordered_map<std::string, std::string>> & module_parameters)
{
  validate_module_parameter_alignment(modules, module_parameters);

  for (auto i = 0ul; i < modules.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "EtherCAT module[%zu]: name=%s alias=%u position=%u vendor_id=0x%x product_id=0x%x",
      i,
      module_parameters[i].at("name").c_str(),
      modules[i]->alias_,
      modules[i]->position_,
      modules[i]->vendor_id_,
      modules[i]->product_id_);
  }
}

}  // namespace

unsigned int uint_from_string(const std::string & str)
{
  // Strip leading and trailing whitespaces
  std::string s = std::regex_replace(str, std::regex("^ +| +$|( ) +"), "$1");
  // Test if the number is in hexadecimal format
  if (s.find("0x") == 0) {
    return std::stoul(s, nullptr, 16);
  }
  return std::stoul(s);
}

void getTransferMemoryInfo(
  const YAML::Node & element,
  ethercat_interface::EcMemoryEntry & entry,
  const std::string & dir,
  const std::string & transfer_net_name)
{
  if (!element["ec_module"]) {
    std::string msg = "Transfer definition without ec_module entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["index"]) {
    std::string msg = "Transfer definition without index entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["subindex"]) {
    std::string msg = "Transfer definition without subindex entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }

  entry.module_name = element["ec_module"].as<std::string>();
  entry.index = uint_from_string(element["index"].as<std::string>());
  entry.subindex = uint_from_string(element["subindex"].as<std::string>());
}

void throwErrorIfModuleParametersNotFound(
  const ethercat_interface::EcTransferEntry & transfer,
  const std::string & module_name,
  const std::string & transfer_net_name,
  const std::string & direction)
{
  std::string msg = "In transfer net: " + transfer_net_name + ", for transfer " +
    transfer.to_simple_string() + ", the module name of the " + direction + "( " + module_name +
    ") among all the recorded modules.";
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "EthercatDriver"), msg.c_str());
  throw std::runtime_error(msg);
}

uint16_t EthercatDriver::getAliasOrDefaultAlias(
  const std::unordered_map<std::string,
  std::string> & slave_parameters)
{
  if (slave_parameters.find("alias") != slave_parameters.end()) {
    return std::stoul(slave_parameters.at("alias"));
  } else {
    return 0;
  }
}

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  // Set state vectors
  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  raw_joint_states_ = hw_joint_states_;
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  // Set command vectors
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  raw_joint_commands_ = hw_joint_commands_;
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  joint_uses_transmission_.assign(info_.joints.size(), false);
  configureTransmissions();

  // Setup slave modules defined per joints in the URDF
  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        auto * state_interfaces = joint_uses_transmission_[j] ?
          &raw_joint_states_[j] : &hw_joint_states_[j];
        auto * command_interfaces = joint_uses_transmission_[j] ?
          &raw_joint_commands_[j] : &hw_joint_commands_[j];
        if (!module->setupSlave(
            module_params[i], state_interfaces, command_interfaces))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Joint module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.joints[j].name.c_str(), ex.what());
      }
    }
  }

  // Setup slave modules defined per GPIOs in the URDF
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        if (!module->setupSlave(
            module_params[i], &hw_gpio_states_[g], &hw_gpio_commands_[g]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of GPIO module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.gpios[g].name.c_str(), ex.what());
      }
    }
  }

  // Setup slave modules defined per sensors in the URDF
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        if (!module->setupSlave(
            module_params[i], &hw_sensor_states_[s], &hw_sensor_commands_[s]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Sensor module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.sensors[s].name.c_str(), ex.what());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %li modules", ec_modules_.size());
  try {
    log_module_mapping(ec_modules_, ec_module_parameters_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "%s", e.what());
    return CallbackReturn::ERROR;
  }

  // Check if a transfer configuration is provided
  if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() ||
    info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end())
  {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer configuration detected, ...");

    YAML::Node config;
    // Load the transfer config file
    loadTransferConfigYamlFile(config);

    // Parse transfer modules from the transfer yaml file
    auto transfer_module_params = getEcTransferModuleParam(config);

    // Append the transfer modules parameters to the list of modules parameters
    size_t idx_1st = ec_module_parameters_.size();
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), transfer_module_params.begin(), transfer_module_params.end());
    for (size_t i = 0; i < transfer_module_params.size(); i++) {
      ec_transfer_slaves_.push_back(idx_1st + i);
    }

    // Parse transfer nets from the transfer yaml file
    ec_transfer_nets_ = getEcTransferNets(config);

    // Append the transfer modules to the list of modules and load them
    for (const auto & transfer_module_param : transfer_module_params) {
      try {
        auto ec_module = ec_loader_.createSharedInstance(transfer_module_param.at("plugin"));
        ec_module->setAliasAndPosition(
          getAliasOrDefaultAlias(transfer_module_param),
          std::stoul(transfer_module_param.at("position")));
        if (!ec_module->setupSlave(
            transfer_module_param, &empty_interface_, &empty_interface_))
        {
          const std::string & module_name = transfer_module_param.at("name");
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of transfer only module %s FAILED.", module_name.c_str() );
          return CallbackReturn::ERROR;
        }

        auto idx = ec_modules_.size();
        ec_modules_.push_back(ec_module);
        ec_transfer_slaves_.push_back(idx);
      } catch (const pluginlib::PluginlibException & ex) {
        const std::string & module_name = transfer_module_param.at("name");
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "EthercatDriver"),
          "The plugin failed to load for transfer module %s. Error: %s\n",
          module_name.c_str(), ex.what());
      }
    }

    // Find all masters from the nets
    {
      std::vector<std::string> master_names;
      for (const auto & net : ec_transfer_nets_) {
        master_names.push_back(net.master);
      }
      for (size_t i = 0; i < ec_module_parameters_.size(); i++) {
        if (std::find(
            master_names.begin(), master_names.end(),
            ec_module_parameters_[i].at("name")) !=
          master_names.end())
        {
          ec_transfer_masters_.push_back(i);
        }
      }
    }

    // Identify (alias,position) all the modules participating in transfers
    for (auto & net : ec_transfer_nets_) {
      for (auto & transfer : net.transfers) {
        // Update each EcMemoryEntry with the alias and position of the module
        size_t in_idx = ec_module_parameters_.size();
        for (in_idx = 0; in_idx < ec_module_parameters_.size(); ++in_idx) {
          if (ec_module_parameters_[in_idx].at("name") == transfer.input.module_name) {
            break;
          }
        }
        size_t out_idx = ec_module_parameters_.size();
        for (out_idx = 0; out_idx < ec_module_parameters_.size(); ++out_idx) {
          if (ec_module_parameters_[out_idx].at("name") == transfer.output.module_name) {
            break;
          }
        }
        if (in_idx == ec_module_parameters_.size()) {
          throwErrorIfModuleParametersNotFound(
            transfer, transfer.input.module_name, net.name, "input");
        }
        if (out_idx == ec_module_parameters_.size()) {
          throwErrorIfModuleParametersNotFound(
            transfer, transfer.output.module_name, net.name, "output");
        }

        const auto & input_module = ec_modules_[in_idx];
        const auto & output_module = ec_modules_[out_idx];

        transfer.input.alias = input_module->alias_;
        transfer.input.position = input_module->position_;
        transfer.output.alias = output_module->alias_;
        transfer.output.position = output_module->position_;
      }
    }

    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "Transfer configuration loaded successfully!");

    try {
      log_module_mapping(ec_modules_, ec_module_parameters_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "%s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name,
          info_.joints[j].state_interfaces[i].name,
          &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[s].name,
          info_.sensors[s].state_interfaces[i].name,
          &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.gpios[g].name,
          info_.gpios[g].state_interfaces[i].name,
          &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name,
          info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.sensors[s].name,
          info_.sensors[s].command_interfaces[i].name,
          &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios[g].name,
          info_.gpios[g].command_interfaces[i].name,
          &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::setupMaster()
{
  unsigned int master_id = 666;
  // Get master id
  if (info_.hardware_parameters.find("master_id") == info_.hardware_parameters.end()) {
    // Master id was not provided, default to 0
    master_id = 0;
  } else {
    try {
      master_id = std::stoul(info_.hardware_parameters["master_id"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid master id (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  if (master_) {
    master_->shutdown();
    master_.reset();
  }

  master_ = std::make_shared<ethercat_interface::EcMaster>(master_id);

  if (!master_ || !master_->isValid()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "Failed to reserve EtherCAT master %u. Is another process already using it?",
      master_id);
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::configNetwork()
{
  // Get control frequency
  if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
    // Control frequency was not provided, default to 100 Hz
    control_frequency_ = 100.0;
  } else {
    try {
      control_frequency_ = std::stod(info_.hardware_parameters["control_frequency"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid control frequency (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  int32_t dc_sync0_shift_ns = 0;
  if (info_.hardware_parameters.find("dc_sync0_shift_ns") != info_.hardware_parameters.end()) {
    try {
      const long parsed = std::stol(info_.hardware_parameters["dc_sync0_shift_ns"]);
      if (parsed < std::numeric_limits<int32_t>::min() ||
        parsed > std::numeric_limits<int32_t>::max())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Invalid dc_sync0_shift_ns: value exceeds int32_t range");
        return CallbackReturn::ERROR;
      }
      dc_sync0_shift_ns = static_cast<int32_t>(parsed);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid dc_sync0_shift_ns (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  // start EC and wait until state operative

  master_->setCtrlFrequency(control_frequency_);
  master_->setDcSync0Shift(dc_sync0_shift_ns);

  // Diagnostics collection must be enabled before activate() so the master can create the
  // per-slave ESC register requests during activation.
  parseDiagnosticsParameters();
  master_->setDiagnosticsEnabled(publish_diagnostics_);

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    master_->addSlave(ec_modules_[i].get());
  }

  try {
    validate_module_parameter_alignment(ec_modules_, ec_module_parameters_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "%s", e.what());
    return CallbackReturn::ERROR;
  }

  // configure SDO
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    for (auto & sdo : ec_modules_[i]->sdo_config) {
      uint32_t abort_code;
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "Downloading config SDO for module '%s' at alias %u position %u: index 0x%x subindex 0x%x",
        ec_module_parameters_[i].at("name").c_str(),
        ec_modules_[i]->alias_,
        ec_modules_[i]->position_,
        sdo.index,
        sdo.sub_index);
      int ret = master_->configSlaveSdo(
        ec_modules_[i]->position_,
        sdo,
        &abort_code);
      if (ret) {
        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to download config SDO for module '%s' at alias %u position %u with Error: %d",
          ec_module_parameters_[i].at("name").c_str(),
          ec_modules_[i]->alias_,
          ec_modules_[i]->position_,
          abort_code);
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ...please wait...");

  // setup master
  if (setupMaster() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // configure network
  if (configNetwork() != CallbackReturn::SUCCESS) {
    if (master_) {
      master_->shutdown();
      master_.reset();
    }
    return CallbackReturn::ERROR;
  }

  if (!master_ || !master_->isValid()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "EtherCAT master is not available.");
    return CallbackReturn::ERROR;
  }

  if (!master_->activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    if (master_) {
      master_->shutdown();
      master_.reset();
    }
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

  // Configure transfer network if transfer nets are defined
  if (!ec_transfer_nets_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Configuring transfer network...");
    master_->registerTransferInDomain(ec_transfer_nets_);
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer network configured!");
  }

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_->update();

    // check if operational
    bool isAllInit = true;
    for (auto & module : ec_modules_) {
      isAllInit = isAllInit && module->initialized();
    }
    if (isAllInit) {
      running = false;
    }
    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += master_->getInterval();
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  activated_ = true;

  // Start the (non-real-time) health-diagnostics publisher once the bus is operational.
  startDiagnostics();

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

  stopDiagnostics();
  cleanup_master(master_, activated_);

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    master_->readData();
    if (publish_diagnostics_) {
      updateTimingStatistics();
    }
    for (auto & transmission : transmissions_) {
      transmission->actuator_to_joint(raw_joint_states_, hw_joint_states_);
    }
  }
  if (!lock.owns_lock()) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("EthercatDriver"), *get_clock(), 1000,
      "Could not acquire lock to read data from EtherCAT master, skipping this cycle.");
  }
  return hardware_interface::return_type::OK;
}

CallbackReturn EthercatDriver::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard lock(ec_mutex_);

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Shutdown cleanup ...please wait...");

  stopDiagnostics();
  cleanup_master(master_, activated_);
  cleanupPluginsForShutdown();

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Shutdown cleanup complete.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard lock(ec_mutex_);

  RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Error cleanup ...please wait...");

  stopDiagnostics();
  cleanup_master(master_, activated_);

  RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Error cleanup complete.");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    for (auto & transmission : transmissions_) {
      transmission->joint_to_actuator(hw_joint_commands_, raw_joint_commands_);
    }
    master_->writeData();
  }
  if (!lock.owns_lock()) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("EthercatDriver"), *get_clock(), 1000,
      "Could not acquire lock to write data to EtherCAT master, skipping this cycle.");
  }
  return hardware_interface::return_type::OK;
}

void EthercatDriver::parseDiagnosticsParameters()
{
  publish_diagnostics_ = false;
  auto it = info_.hardware_parameters.find("publish_diagnostics");
  if (it != info_.hardware_parameters.end()) {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    publish_diagnostics_ = (value == "true" || value == "1");
  }

  diagnostics_period_s_ = 1.0;
  it = info_.hardware_parameters.find("diagnostics_period_s");
  if (it != info_.hardware_parameters.end()) {
    try {
      diagnostics_period_s_ = std::stod(it->second);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid diagnostics_period_s (%s); using %.2f s.", e.what(), diagnostics_period_s_);
    }
  }

  dc_time_diff_warn__ns_ = 10000; // 10 us
  it = info_.hardware_parameters.find("dc_time_diff_warn_ns");
  if (it != info_.hardware_parameters.end()) {
    try {
      dc_time_diff_warn__ns_ = static_cast<int32_t>(std::stol(it->second));
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid dc_time_diff_warn_ns (%s); using %d ns.", e.what(), dc_time_diff_warn__ns_);
    }
  }
}

void EthercatDriver::updateTimingStatistics()
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  if (timing_last_valid_) {
    const double dt =
      static_cast<double>(now.tv_sec - timing_last_ts_.tv_sec) +
      static_cast<double>(now.tv_nsec - timing_last_ts_.tv_nsec) * 1e-9;
    const double expected_period_s =
      (control_frequency_ > 0.0) ? (1.0 / control_frequency_) : 0.0;

    const std::lock_guard<std::mutex> lock(timing_mutex_);
    if (timing_sample_count_ == 0) {
      timing_period_min_s_ = dt;
      timing_period_max_s_ = dt;
    } else {
      timing_period_min_s_ = std::min(timing_period_min_s_, dt);
      timing_period_max_s_ = std::max(timing_period_max_s_, dt);
    }
    timing_period_sum_s_ += dt;
    ++timing_sample_count_;
    timing_period_mean_s_ = timing_period_sum_s_ / static_cast<double>(timing_sample_count_);
    if (expected_period_s > 0.0 && dt > 1.5 * expected_period_s) {
      ++timing_overrun_count_;
    }
    timing_valid_ = true;
  }
  timing_last_ts_ = now;
  timing_last_valid_ = true;
}

void EthercatDriver::startDiagnostics()
{
  if (!publish_diagnostics_) {
    return;
  }

  // Reset the timing statistics accumulated during any previous activation.
  {
    const std::lock_guard<std::mutex> lock(timing_mutex_);
    timing_valid_ = false;
    timing_last_valid_ = false;
    timing_period_min_s_ = 0.0;
    timing_period_max_s_ = 0.0;
    timing_period_mean_s_ = 0.0;
    timing_period_sum_s_ = 0.0;
    timing_sample_count_ = 0;
    timing_overrun_count_ = 0;
  }

  diagnostics_node_ = std::make_shared<rclcpp::Node>("ethercat_diagnostics");
  diagnostics_updater_ =
    std::make_unique<diagnostic_updater::Updater>(diagnostics_node_, diagnostics_period_s_);
  diagnostics_updater_->setHardwareID("ethercat_master");

  diagnostics_updater_->add("EtherCAT Master", this, &EthercatDriver::produceMasterDiagnostics);
  diagnostics_updater_->add("EtherCAT RT Timing", this, &EthercatDriver::produceTimingDiagnostics);

  for (size_t i = 0; i < ec_modules_.size(); ++i) {
    std::string name = "EtherCAT Slave " + std::to_string(i);
    if (i < ec_module_parameters_.size()) {
      const auto name_it = ec_module_parameters_[i].find("name");
      if (name_it != ec_module_parameters_[i].end()) {
        name = "EtherCAT Slave: " + name_it->second;
      }
    }
    diagnostics_updater_->add(
      name, [this, i](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        produceSlaveDiagnostics(stat, i);
      });
  }

  // The updater has no subscriptions, so rather than spin an executor we simply force an
  // update at the configured period. force_update() runs every task and publishes immediately.
  diagnostics_thread_running_ = true;
  diagnostics_thread_ = std::thread(
    [this]() {
      const auto tick = std::chrono::milliseconds(100);
      double elapsed_s = diagnostics_period_s_;  // publish on the first iteration
      while (rclcpp::ok() && diagnostics_thread_running_) {
        if (elapsed_s >= diagnostics_period_s_) {
          diagnostics_updater_->force_update();
          elapsed_s = 0.0;
        }
        std::this_thread::sleep_for(tick);
        elapsed_s += 0.1;
      }
    });

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "EtherCAT diagnostics publishing to /diagnostics every %.2f s.", diagnostics_period_s_);
}

void EthercatDriver::stopDiagnostics()
{
  diagnostics_thread_running_ = false;
  if (diagnostics_thread_.joinable()) {
    diagnostics_thread_.join();
  }
  diagnostics_updater_.reset();
  diagnostics_node_.reset();
}

void EthercatDriver::produceMasterDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!master_) {
    stat.summary(DiagnosticStatus::ERROR, "EtherCAT master not available");
    return;
  }
  const auto diag = master_->getDiagnostics();

  stat.add("slaves_responding", diag.slaves_responding);
  stat.add("link_up", diag.link_up ? "true" : "false");
  stat.addf("al_states", "0x%02X", diag.al_states);
  stat.add("domain_working_counter", diag.working_counter);
  const char * wc_state =
    (diag.wc_state == 2) ? "COMPLETE" : (diag.wc_state == 1) ? "INCOMPLETE" : "ZERO";
  stat.add("domain_wc_state", wc_state);
  stat.add("incomplete_cycles", diag.incomplete_cycle_count);
  stat.add("total_cycles", diag.update_count);

  if (!diag.link_up) {
    stat.summary(DiagnosticStatus::ERROR, "EtherCAT link down");
  } else if (diag.wc_state != 2) {
    stat.summary(DiagnosticStatus::WARN, "Domain working counter incomplete (frame loss)");
  } else {
    stat.summary(DiagnosticStatus::OK, "EtherCAT master operational");
  }
}

void EthercatDriver::produceSlaveDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat, size_t slave_index)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!master_) {
    stat.summary(DiagnosticStatus::ERROR, "EtherCAT master not available");
    return;
  }
  const auto diag = master_->getDiagnostics();
  if (slave_index >= diag.slaves.size()) {
    stat.summary(DiagnosticStatus::WARN, "Slave diagnostics not yet available");
    return;
  }
  const auto & s = diag.slaves[slave_index];

  stat.add("alias", s.alias);
  stat.add("position", s.position);
  stat.addf("vendor_id", "0x%08X", s.vendor_id);
  stat.addf("product_id", "0x%08X", s.product_id);
  stat.add("al_state", al_state_to_string(s.al_state));
  stat.add("online", s.online ? "true" : "false");
  stat.add("operational", s.operational ? "true" : "false");
  if (s.al_status_code_valid) {
    stat.addf(
      "al_status_code", "0x%04X (%s)",
      s.al_status_code, al_status_code_to_string(s.al_status_code));
  }
  if (s.dc_system_time_diff_valid) {
    stat.add("dc_system_time_diff_ns", s.dc_system_time_diff__ns);
  }
  if (s.dc_propagation_delay_valid) {
    stat.add("dc_propagation_delay_ns", s.dc_propagation_delay__ns);
  }
  if (s.has_cia402) {
    stat.add("cia402_state", s.cia402.device_state_label);
    stat.addf("status_word", "0x%04X", s.cia402.status_word);
  }

  if (!s.online) {
    stat.summary(DiagnosticStatus::ERROR, "Slave offline");
  } else if (!s.operational) {
    stat.summary(
      DiagnosticStatus::ERROR,
      "Slave not operational (AL state " + al_state_to_string(s.al_state) + ")");
  } else if (s.has_cia402 && s.cia402.in_fault) {
    stat.summary(DiagnosticStatus::ERROR, "Drive fault: " + s.cia402.device_state_label);
  } else if (s.dc_system_time_diff_valid &&
    std::abs(s.dc_system_time_diff__ns) > dc_time_diff_warn__ns_)
  {
    stat.summary(DiagnosticStatus::WARN, "DC clock drift high");
  } else {
    stat.summary(DiagnosticStatus::OK, "Operational");
  }
}

void EthercatDriver::produceTimingDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const std::lock_guard<std::mutex> lock(timing_mutex_);
  if (!timing_valid_) {
    stat.summary(DiagnosticStatus::OK, "No timing samples yet");
    return;
  }
  const double expected_period_s =
    (control_frequency_ > 0.0) ? (1.0 / control_frequency_) : 0.0;
  stat.add("expected_period_ms", expected_period_s * 1e3);
  stat.add("period_mean_ms", timing_period_mean_s_ * 1e3);
  stat.add("period_min_ms", timing_period_min_s_ * 1e3);
  stat.add("period_max_ms", timing_period_max_s_ * 1e3);
  stat.add("jitter_max_ms", (timing_period_max_s_ - expected_period_s) * 1e3);
  stat.add("overrun_count", timing_overrun_count_);
  stat.add("sample_count", timing_sample_count_);

  if (timing_overrun_count_ > 0) {
    stat.summary(DiagnosticStatus::WARN, "Cyclic loop deadline overruns detected");
  } else {
    stat.summary(DiagnosticStatus::OK, "Cyclic loop timing nominal");
  }
}

void EthercatDriver::configureTransmissions()
{
  transmissions_.clear();

  for (const auto & transmission : info_.transmissions) {
    auto coupling = std::make_unique<LoaderBackedTransmissionCoupling>();
    coupling->configure(transmission, info_.joints);
    for (const auto joint_index : coupling->joint_indices()) {
      joint_uses_transmission_[joint_index] = true;
    }
    for (const auto actuator_index : coupling->actuator_indices()) {
      joint_uses_transmission_[actuator_index] = true;
    }
    transmissions_.push_back(std::move(coupling));
  }
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcModuleParam(
  const std::string & urdf,
  const std::string & component_name,
  const std::string & component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  while (ros2_control_it) {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it) {
      if (!component_name.compare(ros2_control_child_it->Attribute("name"))) {
        const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");
        while (ec_module_it) {
          module_param.clear();
          module_param["name"] = ec_module_it->Attribute("name");
          const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
          if (NULL != plugin_it) {
            module_param["plugin"] = plugin_it->GetText();
          }
          const auto * param_it = ec_module_it->FirstChildElement("param");
          while (param_it) {
            module_param[param_it->Attribute("name")] = param_it->GetText();
            param_it = param_it->NextSiblingElement("param");
          }
          module_params.push_back(module_param);
          ec_module_it = ec_module_it->NextSiblingElement("ec_module");
        }
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }

  return module_params;
}

void EthercatDriver::loadTransferConfigYamlFile(YAML::Node & node, const std::string & path)
{
  std::string file_path;
  if (path.empty()) {
    // Get the fsoe_config or transfer_config parameter of the ethercat_driver hardware plugin
    if (info_.hardware_parameters.find("fsoe_config") == info_.hardware_parameters.end() &&
      info_.hardware_parameters.find("transfer_config") == info_.hardware_parameters.end() )
    {
      std::string msg("transfer_config or fsoe_config parameter is missing!");
      // Transfer (or fsoe) config file was not provided
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() &&
      info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end())
    {
      std::string msg(
        "Both transfer_config and fsoe_config parameters are provided! Please provide only one "
        "of them.");
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() ) {
      std::string msg("The fsoe_config parameter is deprecated. "
        "Please use transfer_config instead.");
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      file_path = info_.hardware_parameters.at("fsoe_config");
    }
    if (info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end()) {
      file_path = info_.hardware_parameters.at("transfer_config");
    }
  } else {
    file_path = path;
  }

  try {
    node = YAML::LoadFile(file_path);
  } catch (const YAML::ParserException & ex) {
    std::string msg =
      std::string(
      "EthercatDriver : failed to load transfer configuration "
      "(YAML file is incorrect): ") + std::string(ex.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (const YAML::BadFile & ex) {
    std::string msg =
      std::string(
      "EthercatDriver : failed to load transfer configuration "
      "(file path is incorrect or file is damaged): " + std::string(ex.what()));
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (std::exception & e) {
    std::string msg =
      std::string(
      "EthercatDriver : error while loading transfer configuration: ") + std::string(e.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  }
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcTransferModuleParam(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }
  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  // It is possible that modules are only involved in transfers and hence
  // not declared in the ros2_control xacro file.
  // This is a common situation with modules only involved in safety
  // operations. In this case, it is necessary to find the plugin to load,
  // the position and the alias for those slaves.
  if (config["transfer_modules"]) {
    for (const auto & module : config["transfer_modules"]) {
      module_param.clear();
      module_param["name"] = module["name"].as<std::string>();
      module_param["plugin"] = module["plugin"].as<std::string>();
      for (const auto & param : module["parameters"]) {
        module_param[param.first.as<std::string>()] = param.second.as<std::string>();
      }
      module_params.push_back(module_param);
    }
  }

  if (config["safety_modules"]) {
    for (const auto & module : config["safety_modules"]) {
      module_param.clear();
      module_param["name"] = module["name"].as<std::string>();
      module_param["plugin"] = module["plugin"].as<std::string>();
      for (const auto & param : module["parameters"]) {
        module_param[param.first.as<std::string>()] = param.second.as<std::string>();
      }
      module_params.push_back(module_param);
    }
  }

  return module_params;
}

std::vector<ethercat_interface::EcTransferNet> EthercatDriver::getEcTransferNets(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }

  std::vector<ethercat_interface::EcTransferNet> transfer_nets;
  ethercat_interface::EcTransferNet transfer_net;

  if (config["nets"]) {
    for (const auto & net : config["nets"]) {
      transfer_net.reset(net["name"].as<std::string>());
      if (net["safety_master"]) {
        transfer_net.master = net["safety_master"].as<std::string>();
      }
      if (net["transfer_master"]) {
        transfer_net.master = net["transfer_master"].as<std::string>();
      }
      for (const auto & transfer : net["transfers"]) {
        ethercat_interface::EcTransferEntry transfer_entry;
        if (!transfer["size"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «size» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["in"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «in» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["out"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «out» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        transfer_entry.size = transfer["size"].as<size_t>();
        getTransferMemoryInfo(
          transfer["in"], transfer_entry.input,
          "in", transfer_net.name);
        getTransferMemoryInfo(
          transfer["out"], transfer_entry.output,
          "out", transfer_net.name);
        transfer_net.transfers.push_back(transfer_entry);
      }
      transfer_nets.push_back(transfer_net);
    }
  }

  return transfer_nets;
}

void EthercatDriver::configTransferNetwork()
{
  // This method can be used for additional transfer network configuration if needed
  // Currently, transfer network configuration is handled in on_activate()
}

void EthercatDriver::cleanupPluginsForShutdown()
{
  ec_modules_.clear();
  ec_module_parameters_.clear();
  ec_transfer_nets_.clear();
  ec_transfer_masters_.clear();
  ec_transfer_slaves_.clear();
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
