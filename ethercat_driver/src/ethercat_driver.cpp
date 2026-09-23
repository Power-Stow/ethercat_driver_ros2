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

#include <tinyxml2.h>

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ethercat_driver/loader_backed_transmission_coupling.hpp"
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

/// Seconds elapsed on CLOCK_MONOTONIC since @p since.
double monotonic_elapsed_s(const struct timespec & since)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<double>(now.tv_sec - since.tv_sec) +
         static_cast<double>(now.tv_nsec - since.tv_nsec) * 1e-9;
}

/// How long on_activate() waits after activating the master before its first update, in seconds.
constexpr double ACTIVATION_INITIAL_DELAY_S = 1.0;

/// How much of the time the master spends re-scanning the bus is kept out of activation_timeout_s,
/// in seconds.
/// The master configures no slave while it scans, so a slave on its way up waits out every scan,
/// and a bus whose slaves are still booting can re-scan several times, each scan taking far longer
/// than usual while a slave is slow to hand over its EEPROM.
/// Beyond this much scanning the time counts again, so a bus that never stops re-scanning still
/// gives up.
constexpr double ACTIVATION_SCAN_ALLOWANCE_S = 30.0;

/// The CLOCK_MONOTONIC instant @p seconds after @p base.
struct timespec monotonic_after(const struct timespec & base, double seconds)
{
  // Clamped so the conversion to nanoseconds cannot overflow: a budget beyond a billion seconds is
  // indistinguishable from none.
  const int64_t offset_ns = static_cast<int64_t>(std::clamp(seconds, 0.0, 1e9) * 1e9);
  struct timespec result = base;
  result.tv_sec += static_cast<time_t>(offset_ns / 1000000000);
  result.tv_nsec += static_cast<long>(offset_ns % 1000000000);  // NOLINT(runtime/int)
  if (result.tv_nsec >= 1000000000) {
    result.tv_nsec -= 1000000000;
    result.tv_sec++;
  }
  return result;
}

/// True when @p instant lies after @p deadline.
bool monotonic_later(const struct timespec & instant, const struct timespec & deadline)
{
  return instant.tv_sec > deadline.tv_sec ||
         (instant.tv_sec == deadline.tv_sec && instant.tv_nsec > deadline.tv_nsec);
}

/// Pulls @p wake_up back to @p deadline when it lies beyond it.
void cap_wake_up(struct timespec & wake_up, const struct timespec & deadline)
{
  if (monotonic_later(wake_up, deadline)) {
    wake_up = deadline;
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

/// RAII helper that runs a cleanup callback on scope exit only when the scope is left by an exception.
///
/// Normal returns are expected to perform their own cleanup (or none, on success),
/// so the callback fires only if more exceptions are in flight than when the guard was constructed.
class ScopedCleanupOnException
{
public:
  explicit ScopedCleanupOnException(std::function<void()> cleanup)
  : cleanup_(std::move(cleanup)), uncaught_exceptions_(std::uncaught_exceptions())
  {
  }

  ~ScopedCleanupOnException()
  {
    if (std::uncaught_exceptions() > uncaught_exceptions_) {
      cleanup_();
    }
  }

  ScopedCleanupOnException(const ScopedCleanupOnException &) = delete;
  ScopedCleanupOnException & operator=(const ScopedCleanupOnException &) = delete;

private:
  std::function<void()> cleanup_;
  int uncaught_exceptions_;
};

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
    if (priority <= 0) {
      return;
    }
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

    active_ = true;
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
    if (!active_) {
      return;
    }
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
  const pthread_t thread_;
  sched_param saved_param_{};
  int saved_policy_ = 0;
  const int elevated_priority_ = 0;
  bool active_ = false;
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
    if (cpu_core < 0) {
      return;
    }
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

    active_ = true;
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"),
      "Activation thread pinned to CPU core %d.", cpu_core);
  }

  /// Restores the saved CPU affinity mask, then reads the mask back to confirm the thread is no
  /// longer confined to the pinned core. See @ref ScopedFifoPriority::~ScopedFifoPriority for the
  /// trade-off that throwing from these destructors accepts.
  ~ScopedCpuAffinity() noexcept(false)
  {
    if (!active_) {
      return;
    }
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
  const pthread_t thread_;
  cpu_set_t saved_affinity_{};
  const int pinned_core_ = -1;
  bool active_ = false;
};

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

uint16_t module_position_from_parameters(
  const std::unordered_map<std::string,
  std::string> & module_parameters)
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

EthercatDriver::~EthercatDriver()
{
  // A still-joinable std::thread member would call std::terminate() on destruction.
  stopDiagnostics();
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
      const int64_t parsed = std::stoll(info_.hardware_parameters["dc_sync0_shift_ns"]);
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

  // Budget for the activation/bring-up loop (see on_activate()).
  activation_timeout_s_ = 10.0;
  if (info_.hardware_parameters.find("activation_timeout_s") != info_.hardware_parameters.end()) {
    try {
      activation_timeout_s_ = std::stod(info_.hardware_parameters["activation_timeout_s"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid activation_timeout_s (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
    // std::stod accepts "nan" and "inf", which slip past every <= 0 check and into the deadline
    // arithmetic.
    if (!std::isfinite(activation_timeout_s_)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "activation_timeout_s must be a finite number!");
      return CallbackReturn::ERROR;
    }
    // The initial delay makes no update, so a budget it uses up could never observe the bus coming
    // up: reject it here rather than fail every activation with a misleading timeout.
    if (activation_timeout_s_ > 0.0 && activation_timeout_s_ <= ACTIVATION_INITIAL_DELAY_S) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "activation_timeout_s (%.3f s) must exceed the %.1f s initial delay of the bring-up loop, "
        "or be <= 0 to wait indefinitely!",
        activation_timeout_s_, ACTIVATION_INITIAL_DELAY_S);
      return CallbackReturn::ERROR;
    }
  }

  // Budget for the shutdown wind-down loop (see windDownSlaves()).
  shutdown_wind_down_timeout_s_ = 1.0;
  if (info_.hardware_parameters.find("shutdown_wind_down_timeout_s") !=
    info_.hardware_parameters.end())
  {
    try {
      shutdown_wind_down_timeout_s_ =
        std::stod(info_.hardware_parameters["shutdown_wind_down_timeout_s"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid shutdown_wind_down_timeout_s (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
    if (!std::isfinite(shutdown_wind_down_timeout_s_)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "shutdown_wind_down_timeout_s must be a finite number!");
      return CallbackReturn::ERROR;
    }
  }

  // Whether a failed startup config SDO download is fatal (see the download loop below).
  require_startup_sdo_ = false;
  if (info_.hardware_parameters.find("require_startup_sdo") != info_.hardware_parameters.end()) {
    const std::string & value = info_.hardware_parameters["require_startup_sdo"];
    require_startup_sdo_ = (value == "true" || value == "1" || value == "True");
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
  size_t failed_sdo_count = 0;
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    for (auto & sdo : ec_modules_[i]->sdo_config) {
      // Only written when the drive's CoE layer aborts the transfer, so it has to start at zero for
      // a failure without a reported abort to be distinguishable.
      uint32_t abort_code = 0;
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
        ++failed_sdo_count;
        RCLCPP_ERROR(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to download config SDO index 0x%x subindex 0x%x for module '%s' at alias %u "
          "position %u: %s. CoE abort code 0x%08x%s",
          sdo.index,
          sdo.sub_index,
          ec_module_parameters_[i].at("name").c_str(),
          ec_modules_[i]->alias_,
          ec_modules_[i]->position_,
          std::strerror(ret < 0 ? -ret : ret),
          abort_code,
          abort_code == 0 ?
          " (zero: no CoE abort was reported, which usually means the transfer did not reach the "
          "drive's CoE layer - check 'ethercat slaves' for its state and identity)" : "");
      }
    }
  }

  if (failed_sdo_count > 0) {
    if (require_startup_sdo_) {
      // The startup SDOs carry values such as the drive's speed limit, torque limits and control
      // gains. Coming up without them silently runs the axis on whatever the drive happens to
      // hold, which is worth refusing an activation over on a machine that relies on them.
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "%zu startup config SDO download(s) failed; refusing to bring the bus up without the "
        "limits and gains they carry (require_startup_sdo is set).",
        failed_sdo_count);
      return CallbackReturn::ERROR;
    }

    // Default, and what this driver has always done: the bus comes up anyway. Said out loud,
    // because the drives then run on whatever they already hold.
    RCLCPP_WARN(
      rclcpp::get_logger("EthercatDriver"),
      "%zu startup config SDO download(s) failed; bringing the bus up anyway, so the drives keep "
      "whatever values they already hold. Set the hardware parameter require_startup_sdo to true "
      "to refuse the activation instead.",
      failed_sdo_count);
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

  // The modules are created once, in on_init(), so they survive a deactivate/activate cycle with
  // whatever state the last wind-down left on them. A wind-down still in force owns the control
  // word and pins every other command channel to its default, which would keep a drive out of
  // Operation Enabled for good.
  for (auto & module : ec_modules_) {
    module->reset_wind_down();
  }

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

  // Start the (non-real-time) health-diagnostics publisher *before* the blocking bring-up loop so
  // that a slave stuck reaching OP (e.g. DC not converging) is still observable on /diagnostics.
  // The bring-up loop below drives master_->update(), which populates the snapshot; the publisher
  // reads it via getDiagnostics() (a separate lock), so it keeps publishing even while this
  // activation thread is busy in the loop.
  // Started before the priority elevation below so the publisher thread does not inherit SCHED_FIFO.
  startDiagnostics();
  // Stop and join the publisher if any later activation step throws (e.g. ScopedFifoPriority),
  // so it does not outlive a failed activation or leave a joinable thread behind.
  // Declared before the priority/affinity guards so it runs after they have restored scheduling.
  const ScopedCleanupOnException diagnostics_cleanup([this]() {stopDiagnostics();});

  // Elevate this thread to real-time scheduling for the blocking bring-up loop below. The loop
  // drives master_->update(), which sends the cyclic EtherCAT frames that discipline the
  // Distributed Clocks; sending them with low jitter lets DC slaves converge within the master's
  // DC sync-wait window instead of stalling for the full timeout. Scheduling is restored on exit.
  // Constructed priority-first so destruction restores the affinity before the scheduling policy.
  // Held by value rather than through a unique_ptr: the guards' destructors throw by design, and
  // unique_ptr's destructor is noexcept, so a throw from one would call std::terminate(). A guard
  // given a disabled value (priority <= 0, core < 0) does nothing.
  const ScopedFifoPriority activation_priority(activation_thread_priority_);
  const ScopedCpuAffinity activation_affinity(activation_cpu_core_);

  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  // Timed from here rather than from the first cycle, so the initial delay counts against
  // activation_timeout_s instead of being added to it.
  const struct timespec activation_start = t;

  const uint32_t interval_ns = master_->getInterval();

  // Start after the initial delay, which on_init() guarantees is shorter than activation_timeout_s.
  // Slept one period at a time rather than in one go, so a shutdown request is honoured during the
  // delay as well. Every wake-up is capped at the deadline it serves, so a control period longer
  // than the remaining budget cannot overshoot it: the last step of the delay, and of the loop
  // below, is cut short.
  const double initial_delay_s = ACTIVATION_INITIAL_DELAY_S;
  const struct timespec initial_delay_end = monotonic_after(activation_start, initial_delay_s);
  while (rclcpp::ok() && monotonic_elapsed_s(activation_start) < initial_delay_s) {
    t.tv_nsec += interval_ns;
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
    cap_wake_up(t, initial_delay_end);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
  }

  // Time the master spends re-scanning the bus is kept out of the budget, up to
  // ACTIVATION_SCAN_ALLOWANCE_S of it. The master configures no slave while it scans, so counting
  // that wait would fail a bus that is only slow to come up, while giving it no time at all would
  // let a bus that never stops re-scanning wait forever.
  double scan_s = 0.0;
  double previous_elapsed_s = monotonic_elapsed_s(activation_start);
  bool was_scanning = false;
  const auto budget_s = [this, &scan_s]()
    {
      return activation_timeout_s_ + std::min(scan_s, ACTIVATION_SCAN_ALLOWANCE_S);
    };
  const auto timed_out = [this, &activation_start, &budget_s]()
    {
      return activation_timeout_s_ > 0.0 &&
             monotonic_elapsed_s(activation_start) >= budget_s();
    };

  bool operational = false;
  while (true) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    // The interval that just ended counts towards the scan allowance when the master is scanning.
    // Sampled before the exits, so the budget they check already includes it.
    const double elapsed_s = monotonic_elapsed_s(activation_start);
    const bool scanning = master_->scanBusy();
    if (scanning) {
      scan_s += elapsed_s - previous_elapsed_s;
      if (!was_scanning) {
        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "EtherCAT master is re-scanning the bus and configures no slave until the scan ends. The "
          "scan does not count against activation_timeout_s, up to %.0f s of scanning in total.",
          ACTIVATION_SCAN_ALLOWANCE_S);
      }
    }
    was_scanning = scanning;
    previous_elapsed_s = elapsed_s;

    // Both exits are checked before the update, not only after it: an update started once shutdown
    // is requested or the budget is spent can only delay giving up, and its result would not be
    // accepted anyway. The failure path below still winds down whatever reached OP.
    if (!rclcpp::ok()) {
      // This loop runs on the thread that delivered the robot description, so while it spins the
      // node answers no service and honours no signal: a bus that never reaches OP used to leave
      // ros2_control_node to be SIGKILLed. Give up as soon as shutdown is requested, and ahead of
      // a bus that has come up meanwhile: activating into a shutdown helps nobody.
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Shutdown requested while waiting for the EtherCAT bus to become operational.");
      break;
    }
    if (timed_out()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"),
        "EtherCAT bus did not become operational within %.1f s, not counting %.1f s the master "
        "spent re-scanning it. Still waiting on: %s. Check 'ethercat slaves': a slave stuck in "
        "INIT whose identity reads 0x00000000 has not released its EEPROM to the master, which "
        "'ethercat rescan' usually clears.",
        activation_timeout_s_,
        std::min(scan_s, ACTIVATION_SCAN_ALLOWANCE_S),
        pendingModuleDescription().c_str());
      break;
    }

    // update EtherCAT bus
    master_->update();

    // check if operational
    bool isAllInit = true;
    for (auto & module : ec_modules_) {
      isAllInit = isAllInit && module->initialized();
    }
    // Only accepted when the update also finished inside the budget: capping the requested wake-up
    // does not cap the actual one, and a late wake-up or a slow update() must not turn into a
    // success past it. A late result goes round once more and is reported as the timeout above.
    if (isAllInit && !timed_out()) {
      operational = true;
      break;
    }

    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += interval_ns;
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
    if (activation_timeout_s_ > 0.0) {
      cap_wake_up(t, monotonic_after(activation_start, budget_s()));
    }
  }

  if (!operational) {
    // On a bus that is only part of the way up, the drives that already reached Operation Enabled
    // would otherwise lose their cyclic data while energised. The ones still pending report the
    // wind-down complete at once, so this costs nothing when no drive got that far. The thread is
    // still under the activation's real-time guards, so the wind-down reuses them rather than
    // nesting its own: a nested guard saves the already-elevated state, and its restore checks
    // throw from destructors while the other guard's throw is unwinding, which calls
    // std::terminate().
    try {
      windDownSlaves(false);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "EtherCAT wind-down failed: %s. Releasing the master anyway.", e.what());
    }
    stopDiagnostics();
    if (master_) {
      master_->shutdown();
      master_.reset();
    }
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  activated_ = true;

  return CallbackReturn::SUCCESS;
}

std::string EthercatDriver::pendingModuleDescription() const
{
  std::string pending;
  for (size_t i = 0; i < ec_modules_.size(); ++i) {
    if (ec_modules_[i]->initialized()) {
      continue;
    }

    std::string name = "<unnamed>";
    if (i < ec_module_parameters_.size()) {
      const auto name_it = ec_module_parameters_[i].find("name");
      if (name_it != ec_module_parameters_[i].end()) {
        name = name_it->second;
      }
    }

    if (!pending.empty()) {
      pending += ", ";
    }
    pending += name + " (alias " + std::to_string(ec_modules_[i]->alias_) +
      " position " + std::to_string(ec_modules_[i]->position_) + ")";
  }

  return pending.empty() ? "none" : pending;
}

void EthercatDriver::windDownSlaves(bool elevate_scheduling)
{
  // Deliberately not gated on activated_: a failed bring-up winds down whatever reached OP too.
  // The master is released on every exit from ACTIVE, so a released bus still returns here.
  if (!master_ || !master_->isValid()) {
    return;
  }

  const uint32_t interval_ns = master_->getInterval();
  if (interval_ns == 0 || shutdown_wind_down_timeout_s_ <= 0.0) {
    return;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "Winding down %zu EtherCAT module(s), at most %.3f s ...",
    ec_modules_.size(), shutdown_wind_down_timeout_s_);

  for (auto & module : ec_modules_) {
    module->start_wind_down(shutdown_wind_down_timeout_s_);
  }

  // The bring-up loop's real-time treatment applies here for the same reason: a scheduling gap
  // stops the cyclic frames for longer than a DC slave's sync watchdog allows, which is exactly the
  // synchronization error this loop exists to avoid. Constructed priority-first so destruction
  // restores the affinity before the scheduling policy. Held by value so a throw from their
  // destructors reaches the caller's try/catch; see on_activate().
  const ScopedFifoPriority wind_down_priority(elevate_scheduling ? activation_thread_priority_ : 0);
  const ScopedCpuAffinity wind_down_affinity(elevate_scheduling ? activation_cpu_core_ : -1);

  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  // Bounded by elapsed time rather than a count of nominal cycles: after a scheduling stall or an
  // update() that overruns its period, the absolute deadlines below are already in the past, and a
  // cycle count would let the catch-up cycles hold deactivation well beyond the timeout.
  const struct timespec wind_down_start = t;
  // The first wind-down frame goes out straight away, and no update is started at or past the
  // deadline: an update begun there would take deactivation over the budget by however long it
  // runs. Stopping before the sleep rather than after it also keeps a control period longer than
  // the remaining budget from holding deactivation for a full period past it.
  const struct timespec wind_down_deadline =
    monotonic_after(wind_down_start, shutdown_wind_down_timeout_s_);

  bool complete = false;
  while (true) {
    master_->update();

    complete = true;
    for (auto & module : ec_modules_) {
      complete = complete && module->wind_down_complete();
    }
    if (complete) {
      break;
    }

    // calculate next shot. carry over nanoseconds into seconds.
    t.tv_nsec += interval_ns;
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (!monotonic_later(wind_down_deadline, t) || !monotonic_later(wind_down_deadline, now)) {
      break;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
  }

  if (complete) {
    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "Wind-down complete after %.3f s.",
      monotonic_elapsed_s(wind_down_start));
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("EthercatDriver"),
      "Wind-down did not complete within %.3f s. The master is released with at least one slave "
      "still energised, which can leave that slave reporting a synchronization error.",
      shutdown_wind_down_timeout_s_);
  }
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

  // A wind-down that throws must not cost us the master release: the slaves are worse off holding
  // an activated master than they are having skipped the wind-down.
  try {
    windDownSlaves();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("EthercatDriver"),
      "EtherCAT wind-down failed: %s. Releasing the master anyway.", e.what());
  }

  activated_ = false;

  stopDiagnostics();
  cleanup_master(master_, activated_);

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & stop_interfaces)
{
  // Starting interfaces are left alone: a controller that has just claimed one writes it before the
  // next cycle reaches the bus, and preempting that would overwrite its first command.
  for (const auto & interface_name : stop_interfaces) {
    release_joint_command(interface_name);
  }
  return hardware_interface::return_type::OK;
}

double EthercatDriver::joint_position_state(size_t joint_index) const
{
  const auto & state_interfaces = info_.joints[joint_index].state_interfaces;
  for (size_t k = 0; k < state_interfaces.size(); k++) {
    if (state_interfaces[k].name == hardware_interface::HW_IF_POSITION) {
      return hw_joint_states_[joint_index][k];
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void EthercatDriver::release_joint_command(const std::string & interface_name)
{
  for (size_t j = 0; j < info_.joints.size(); j++) {
    for (size_t i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      const std::string & name = info_.joints[j].command_interfaces[i].name;
      if (interface_name != info_.joints[j].name + "/" + name) {
        continue;
      }

      if (name == hardware_interface::HW_IF_POSITION) {
        // Held at the joint's last read position. A NaN is not "stay where you are" for every
        // module: the channel managers write their configured default in place of it, which only
        // the CiA-402 plugin keeps at the last read position, so a generic channel defaulting to
        // zero would be commanded to zero. Without a position reading to hold, the last command is
        // left in place.
        const double held_position = joint_position_state(j);
        if (std::isnan(held_position)) {
          RCLCPP_WARN(
            rclcpp::get_logger("EthercatDriver"),
            "Command interface '%s' was released without a position reading to hold; its last "
            "command is left in place.",
            interface_name.c_str());
          return;
        }
        hw_joint_commands_[j][i] = held_position;
      } else if (name == hardware_interface::HW_IF_VELOCITY || // NOLINT
        name == hardware_interface::HW_IF_EFFORT)
      {
        hw_joint_commands_[j][i] = 0.0;
      } else {
        // The control word, the mode of operation and the fault reset are not motion, and a drive
        // that is between controllers should keep the mode and the state machine it already had.
        return;
      }

      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "Released command interface '%s' to a value that commands no motion.",
        interface_name.c_str());
      return;
    }
  }
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

  // Only does anything when the component is finalized straight from ACTIVE; after on_deactivate()
  // the master is already released and this returns immediately.
  try {
    windDownSlaves();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("EthercatDriver"),
      "EtherCAT wind-down failed: %s. Releasing the master anyway.", e.what());
  }

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

  // ERROR can be entered straight from ACTIVE, after an exception in a cyclic read or write, with
  // drives still in Operation Enabled. Released without a wind-down they would lose their cyclic
  // data energised. When the master is already gone this returns immediately.
  try {
    windDownSlaves();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("EthercatDriver"),
      "EtherCAT wind-down failed: %s. Releasing the master anyway.", e.what());
  }

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
      const auto diagnostics_period_in = std::stod(it->second);
      if (diagnostics_period_in > 0) {
        diagnostics_period_s_ = diagnostics_period_in;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("EthercatDriver"),
          "Invalid diagnostics_period_s (%f); using %.2f s.", diagnostics_period_in, diagnostics_period_s_);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid diagnostics_period_s (%s); using %.2f s.", e.what(), diagnostics_period_s_);
    }
  }

  dc_time_diff_warn_ns_ = 10000; // 10 us
  it = info_.hardware_parameters.find("dc_time_diff_warn_ns");
  if (it != info_.hardware_parameters.end()) {
    try {
      dc_time_diff_warn_ns_ = static_cast<int32_t>(std::stol(it->second));
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid dc_time_diff_warn_ns (%s); using %d ns.", e.what(), dc_time_diff_warn_ns_);
    }
  }

  dt_tolerated_overrun_ = 0.5;  // 50 % overrun
  it = info_.hardware_parameters.find("dt_tolerated_overrun");
  if (it != info_.hardware_parameters.end()) {
    try {
      dt_tolerated_overrun_ = std::stod(it->second);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid dt_tolerated_overrun (%s); using %.2f.", e.what(), dt_tolerated_overrun_);
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
    if (expected_period_s > 0.0 && dt > (1.0 + dt_tolerated_overrun_) * expected_period_s) {
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

  // Derive the node name and hardware ID from the hardware component name so multiple driver instances
  // in one controller manager publish distinguishable statuses (the updater prefixes each status name
  // with the node name). Characters that are not valid in a ROS node name are replaced by '_'.
  std::string node_name = "ethercat_diagnostics_" + info_.name;
  std::replace_if(
    node_name.begin(), node_name.end(),
    [](unsigned char c) {return !std::isalnum(c) && c != '_';}, '_');
  diagnostics_node_ = std::make_shared<rclcpp::Node>(node_name);
  diagnostics_updater_ =
    std::make_unique<diagnostic_updater::Updater>(diagnostics_node_, diagnostics_period_s_);
  diagnostics_updater_->setHardwareID(info_.name);

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
      // Sleep in bounded slices so stopDiagnostics() is not blocked by a long period.
      const auto max_sleep = std::chrono::milliseconds(100);
      const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(diagnostics_period_s_));
      auto next_publish = std::chrono::steady_clock::now();  // publish on the first iteration
      while (rclcpp::ok() && diagnostics_thread_running_) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_publish) {
          diagnostics_updater_->force_update();
          next_publish += period;
          if (next_publish <= now) {
            // Publishing fell behind; resynchronize rather than publish in a burst.
            next_publish = now + period;
          }
        }
        std::this_thread::sleep_until(std::min(next_publish, now + max_sleep));
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
  if (!diag.valid) {
    // The bring-up loop has not run its first EtherCAT cycle yet, so the default-constructed
    // snapshot (e.g. link_up == false) must not be reported as a failure.
    stat.summary(DiagnosticStatus::OK, "Waiting for first EtherCAT cycle");
    return;
  }

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
  if (!diag.valid) {
    stat.summary(DiagnosticStatus::OK, "Waiting for first EtherCAT cycle");
    return;
  }
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
    stat.add("cia402_state", std::string(s.cia402.device_state_label));
    stat.addf("status_word", "0x%04X", s.cia402.status_word);
  }

  if (!s.online) {
    stat.summary(DiagnosticStatus::ERROR, "Slave offline");
  } else if (!s.operational) {
    stat.summary(
      DiagnosticStatus::ERROR,
      "Slave not operational (AL state " + al_state_to_string(s.al_state) + ")");
  } else if (s.has_cia402 && s.cia402.in_fault) {
    stat.summary(
      DiagnosticStatus::ERROR, std::string("Drive fault: ") + s.cia402.device_state_label);
  } else if (s.dc_system_time_diff_valid &&
    std::abs(s.dc_system_time_diff__ns) > dc_time_diff_warn_ns_)
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
  // Copy the scalars and release the lock before formatting, since stat.add() and stat.summary()
  // may allocate and the real-time read() path contends for the same mutex.
  bool valid = false;
  double period_min_s = 0.0;
  double period_max_s = 0.0;
  double period_mean_s = 0.0;
  uint64_t sample_count = 0;
  uint64_t overrun_count = 0;
  {
    const std::lock_guard<std::mutex> lock(timing_mutex_);
    valid = timing_valid_;
    period_min_s = timing_period_min_s_;
    period_max_s = timing_period_max_s_;
    period_mean_s = timing_period_mean_s_;
    sample_count = timing_sample_count_;
    overrun_count = timing_overrun_count_;
  }
  if (!valid) {
    stat.summary(DiagnosticStatus::OK, "No timing samples yet");
    return;
  }
  const double expected_period_s =
    (control_frequency_ > 0.0) ? (1.0 / control_frequency_) : 0.0;
  stat.add("expected_period_ms", expected_period_s * 1e3);
  stat.add("period_mean_ms", period_mean_s * 1e3);
  stat.add("period_min_ms", period_min_s * 1e3);
  stat.add("period_max_ms", period_max_s * 1e3);
  stat.add("jitter_max_ms", (period_max_s - expected_period_s) * 1e3);
  stat.add("overrun_count", overrun_count);
  stat.add("sample_count", sample_count);

  if (overrun_count > 0) {
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
