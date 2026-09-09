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

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

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

/// Logs @p message as a warning and then throws it as a std::runtime_error.
///
/// The log line is written first because these failures are also reported from destructors: an
/// exception leaving a destructor while another exception unwinds out of `on_activate()` calls
/// std::terminate(), and the log line is then the only surviving record of the cause. The severity
/// is a warning rather than an error because `controller_manager` logs an error of its own for the
/// exception once it reaches the hardware component boundary.
[[noreturn]] void log_and_throw(const std::string & message)
{
  RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "%s", message.c_str());
  throw std::runtime_error(message);
}

/// RAII helper that temporarily elevates the calling thread to SCHED_FIFO real-time scheduling for
/// the duration of a scope, restoring the previous scheduling policy and priority on destruction.
///
/// The EtherCAT bring-up loop that disciplines the Distributed Clocks runs on the (non-real-time)
/// activation thread, not on the controller_manager real-time update thread. Sending the cyclic
/// sync frames with low scheduling jitter is required for DC slaves to converge (system-time
/// difference below the master threshold) within the master's DC sync-wait window; otherwise each
/// DC slave stalls for the full wait before the master proceeds. Memory locking is intentionally
/// not handled here: the controller_manager already locks process memory (its `lock_memory`
/// parameter) and mlockall() is process-wide.
class ScopedFifoPriority
{
public:
  /// @param priority SCHED_FIFO priority to apply; values <= 0 disable the FIFO elevation.
  explicit ScopedFifoPriority(int priority)
  : thread_(pthread_self()), elevated_priority_(priority)
  {
    if (const int error = pthread_getschedparam(thread_, &saved_policy_, &saved_param_)) {
      log_and_throw(
        "Failed to get scheduling policy and parameters for the activation thread: " +
        std::string(std::strerror(error)));
    }

    sched_param param{};
    param.sched_priority = priority;
    if (const int error = pthread_setschedparam(thread_, SCHED_FIFO, &param)) {
      log_and_throw(
        "Failed to set SCHED_FIFO priority " + std::to_string(priority) +
        " for the activation thread: " + std::string(std::strerror(error)));
    }

    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"),
      "Activation thread elevated to SCHED_FIFO priority %d.", priority);
  }

  /// Restores the saved scheduling policy and priority, then reads the scheduling state back to
  /// confirm the thread actually left the elevated priority. A restore that reports success without
  /// taking effect, or a saved state that was itself real-time, would otherwise leave the thread at
  /// real-time priority with nothing in the log to show it.
  ///
  /// Every failure is logged and then thrown, so that a thread left under real-time scheduling
  /// fails the activation instead of being carried silently into operation. Throwing here is a
  /// deliberate trade: an exception leaving this destructor while another exception unwinds out of
  /// `on_activate()` calls std::terminate(), which is why the log line is always written first.
  ~ScopedFifoPriority() noexcept(false)
  {
    if (const int error = pthread_setschedparam(thread_, saved_policy_, &saved_param_)) {
      log_and_throw(
        "Failed to restore the activation thread to scheduling policy " +
        std::to_string(saved_policy_) + " priority " +
        std::to_string(saved_param_.sched_priority) + ": " + std::string(std::strerror(error)) +
        ". The thread stays at SCHED_FIFO priority " + std::to_string(elevated_priority_) + ".");
    }

    int restored_policy = 0;
    sched_param restored_param{};
    if (const int error = pthread_getschedparam(thread_, &restored_policy, &restored_param)) {
      log_and_throw(
        "Restored the activation thread scheduling but could not read it back to confirm: " +
        std::string(std::strerror(error)));
    }

    if (restored_policy != saved_policy_ ||
      restored_param.sched_priority != saved_param_.sched_priority)
    {
      log_and_throw(
        "Activation thread scheduling restore did not take effect: expected policy " +
        std::to_string(saved_policy_) + " priority " +
        std::to_string(saved_param_.sched_priority) + ", read back policy " +
        std::to_string(restored_policy) + " priority " +
        std::to_string(restored_param.sched_priority) + ".");
    }

    if (restored_policy == SCHED_FIFO && restored_param.sched_priority == elevated_priority_) {
      log_and_throw(
        "Activation thread is still at SCHED_FIFO priority " +
        std::to_string(elevated_priority_) + " after restore: the scheduling state saved on entry "
        "was itself real-time.");
    }

    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"),
      "Activation thread scheduling restored to policy %d priority %d.",
      saved_policy_, saved_param_.sched_priority);
  }

  ScopedFifoPriority(const ScopedFifoPriority &) = delete;
  ScopedFifoPriority & operator=(const ScopedFifoPriority &) = delete;

private:
  pthread_t thread_;
  sched_param saved_param_{};
  int saved_policy_ = 0;
  int elevated_priority_ = 0;
};

/// RAII helper that temporarily pins the calling thread to a single CPU core for the duration of a
/// scope, restoring the previous CPU affinity mask on destruction.
///
/// Pinning the activation thread keeps the Distributed Clocks bring-up loop on one core, avoiding
/// the migration-induced jitter described for @ref ScopedFifoPriority.
class ScopedCpuAffinity
{
public:
  /// @param cpu_core CPU core to pin the thread to; values < 0 leave the affinity unchanged.
  explicit ScopedCpuAffinity(int cpu_core)
  : thread_(pthread_self()), pinned_core_(cpu_core)
  {
    CPU_ZERO(&saved_affinity_);
    if (const int error = pthread_getaffinity_np(thread_, sizeof(saved_affinity_),
        &saved_affinity_))
    {
      log_and_throw(
        "Failed to get CPU affinity for the activation thread: " +
        std::string(std::strerror(error)));
    }

    cpu_set_t requested;
    CPU_ZERO(&requested);
    CPU_SET(cpu_core, &requested);
    if (const int error = pthread_setaffinity_np(thread_, sizeof(requested), &requested)) {
      log_and_throw(
        "Failed to pin the activation thread to CPU core " + std::to_string(cpu_core) +
        ": " + std::string(std::strerror(error)));
    }

    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"),
      "Activation thread pinned to CPU core %d.", cpu_core);
  }

  /// Restores the saved CPU affinity mask, then reads the mask back to confirm the thread is no
  /// longer confined to the pinned core. See @ref ScopedFifoPriority::~ScopedFifoPriority for the
  /// trade-off that throwing from these destructors accepts.
  ~ScopedCpuAffinity() noexcept(false)
  {
    if (const int error = pthread_setaffinity_np(thread_, sizeof(saved_affinity_),
        &saved_affinity_))
    {
      log_and_throw(
        "Failed to restore the CPU affinity of the activation thread: " +
        std::string(std::strerror(error)) + ". The thread stays pinned to CPU core " +
        std::to_string(pinned_core_) + ".");
    }

    cpu_set_t restored;
    CPU_ZERO(&restored);
    if (const int error = pthread_getaffinity_np(thread_, sizeof(restored), &restored)) {
      log_and_throw(
        "Restored the activation thread CPU affinity but could not read it back to confirm: " +
        std::string(std::strerror(error)));
    }

    if (!CPU_EQUAL(&restored, &saved_affinity_)) {
      log_and_throw(
        "Activation thread CPU affinity restore did not take effect: expected " +
        std::to_string(CPU_COUNT(&saved_affinity_)) + " CPUs, read back " +
        std::to_string(CPU_COUNT(&restored)) + ".");
    }

    if (pinned_core_ >= 0 && CPU_COUNT(&restored) == 1 && CPU_ISSET(pinned_core_, &restored)) {
      log_and_throw(
        "Activation thread is still pinned to CPU core " + std::to_string(pinned_core_) +
        " after restore: the affinity saved on entry was that core alone.");
    }

    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"),
      "Activation thread CPU affinity restored to %d CPUs.", CPU_COUNT(&restored));
  }

  ScopedCpuAffinity(const ScopedCpuAffinity &) = delete;
  ScopedCpuAffinity & operator=(const ScopedCpuAffinity &) = delete;

private:
  pthread_t thread_;
  cpu_set_t saved_affinity_{};
  int pinned_core_ = -1;
};
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
    log_and_throw(
      "EtherCAT module list and module parameter list have different sizes: modules=" +
      std::to_string(modules.size()) + ", parameters=" +
      std::to_string(module_parameters.size()));
  }

  for (auto i = 0ul; i < modules.size(); ++i) {
    const auto parameter_position = module_position_from_parameters(module_parameters[i]);
    if (modules[i]->position_ != parameter_position) {
      log_and_throw(
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
    log_and_throw(msg);
  }
  if (!element["index"]) {
    std::string msg = "Transfer definition without index entry, net: " +
      transfer_net_name + " direction: " + dir;
    log_and_throw(msg);
  }
  if (!element["subindex"]) {
    std::string msg = "Transfer definition without subindex entry, net: " +
      transfer_net_name + " direction: " + dir;
    log_and_throw(msg);
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

  // Optional real-time scheduling for the activation/bring-up loop (see on_activate()).
  activation_thread_priority_ = 0;
  if (info_.hardware_parameters.find("activation_thread_priority") !=
    info_.hardware_parameters.end())
  {
    try {
      activation_thread_priority_ =
        std::stoi(info_.hardware_parameters["activation_thread_priority"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid activation_thread_priority (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  activation_cpu_core_ = -1;
  if (info_.hardware_parameters.find("activation_cpu_core") != info_.hardware_parameters.end()) {
    try {
      activation_cpu_core_ = std::stoi(info_.hardware_parameters["activation_cpu_core"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid activation_cpu_core (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  // start EC and wait until state operative

  master_->setCtrlFrequency(control_frequency_);
  master_->setDcSync0Shift(dc_sync0_shift_ns);

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

  // Elevate this thread to real-time scheduling for the blocking bring-up loop below. The loop
  // drives master_->update(), which sends the cyclic EtherCAT frames that discipline the
  // Distributed Clocks; sending them with low jitter lets DC slaves converge within the master's
  // DC sync-wait window instead of stalling for the full timeout. Scheduling is restored on exit.
  // Constructed priority-first so destruction restores the affinity before the scheduling policy.
  const std::unique_ptr<ScopedFifoPriority> activation_priority =
    activation_thread_priority_ > 0 ? std::make_unique<ScopedFifoPriority>(activation_thread_priority_) : nullptr;
  const std::unique_ptr<ScopedCpuAffinity> activation_affinity =
    activation_cpu_core_ >= 0 ? std::make_unique<ScopedCpuAffinity>(activation_cpu_core_) : nullptr;

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

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

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
    log_and_throw("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    log_and_throw("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    log_and_throw("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    log_and_throw("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    log_and_throw("no ros2_control tag");
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
