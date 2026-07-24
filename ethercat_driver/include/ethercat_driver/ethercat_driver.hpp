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

#ifndef ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ethercat_driver/visibility_control.h"
#include "ethercat_driver/transmission_coupling_base.hpp"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_master.hpp"
#include "yaml-cpp/yaml.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ethercat_driver
{

class EthercatDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  std::vector<std::unordered_map<std::string, std::string>> getEcModuleParam(
    const std::string & urdf,
    const std::string & component_name,
    const std::string & component_type);

  void configureTransmissions();

  uint16_t getAliasOrDefaultAlias(
    const std::unordered_map<std::string,
    std::string> & slave_parameters);

  virtual CallbackReturn setupMaster();

  CallbackReturn configNetwork();

  /** @brief Load transfer config YAML file
   * One use case is to load transfers for FailSafe Over EtherCAT Safety
   * @param[out] node YAML node containing the transfer configuration root
   * @param[in] path Path to the YAML file, if empty, the file is loaded from the *fsoe_config*
   * or *transfer_config* of the YAML document
   */
  void loadTransferConfigYamlFile(YAML::Node & node, const std::string & path = "");

  /** @brief Get transfer module parameters from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of maps containing transfer module parameters, each map corresponds to a module
   * involved in a transfer
   */
  std::vector<std::unordered_map<std::string, std::string>> getEcTransferModuleParam(
    const YAML::Node & config);

  /** @brief Get transfer nets from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of transfer nets
   */
  std::vector<ethercat_interface::EcTransferNet> getEcTransferNets(const YAML::Node & config);

  /** @brief Configure the transfer networks
   */
  void configTransferNetwork();

  void cleanupPluginsForShutdown();

  /** Parse diagnostics-related hardware parameters (publish_diagnostics, thresholds). */
  void parseDiagnosticsParameters();

  /** Start the non-real-time diagnostics publishing node/thread (after activation). */
  void startDiagnostics();

  /** Stop and join the diagnostics publishing thread (before releasing the master). */
  void stopDiagnostics();

  /** Update the real-time cycle-timing statistics from the current read() invocation. */
  void updateTimingStatistics();

  /** diagnostic_updater task: master- and bus-level health. */
  void produceMasterDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /** diagnostic_updater task: per-slave health for the slave at @p slave_index. */
  void produceSlaveDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & stat, size_t slave_index);
  /** diagnostic_updater task: real-time cyclic-loop timing health. */
  void produceTimingDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

protected:
  std::vector<std::shared_ptr<ethercat_interface::EcSlave>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> raw_joint_commands_;
  std::vector<std::vector<double>> hw_sensor_commands_;
  std::vector<std::vector<double>> hw_gpio_commands_;
  std::vector<std::vector<double>> hw_joint_states_;
  std::vector<std::vector<double>> raw_joint_states_;
  std::vector<std::vector<double>> hw_sensor_states_;
  std::vector<std::vector<double>> hw_gpio_states_;
  std::vector<bool> joint_uses_transmission_;
  std::vector<std::unique_ptr<TransmissionCouplingBase>> transmissions_;

  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{
    "ethercat_interface", "ethercat_interface::EcSlave"};

  double control_frequency_;

  /** SCHED_FIFO priority applied to the activation/bring-up loop; <= 0 disables the elevation. */
  int activation_thread_priority_ = 0;
  /** CPU core the activation/bring-up loop is pinned to; < 0 leaves the CPU affinity unchanged. */
  int activation_cpu_core_ = -1;

  std::shared_ptr<ethercat_interface::EcMaster> master_;
  std::mutex ec_mutex_;
  bool activated_;

  /** Transfer nets */
  std::vector<ethercat_interface::EcTransferNet> ec_transfer_nets_;

  /** Indexes of modules inside ec_modules_ vector that are transfer masters */
  std::vector<size_t> ec_transfer_masters_;
  /** Indexes of modules inside ec_modules_ vector that are transfer slaves only */
  std::vector<size_t> ec_transfer_slaves_;

  /** Empty interfaces */
  std::vector<double> empty_interface_;

  // --- Health diagnostics (opt-in via the "publish_diagnostics" hardware parameter) ---
  bool publish_diagnostics_ = false;
  double diagnostics_period_s_ = 1.0;
  int32_t dc_time_diff_warn__ns_ = 1000;

  rclcpp::Node::SharedPtr diagnostics_node_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_updater_;
  std::thread diagnostics_thread_;
  std::atomic<bool> diagnostics_thread_running_{false};

  /** guards the real-time cycle-timing statistics below */
  std::mutex timing_mutex_;
  bool timing_valid_ = false;
  double timing_period_min_s_ = 0.0;
  double timing_period_max_s_ = 0.0;
  double timing_period_mean_s_ = 0.0;
  double timing_period_sum_s_ = 0.0;
  uint64_t timing_sample_count_ = 0;
  uint64_t timing_overrun_count_ = 0;
  bool timing_last_valid_ = false;
  struct timespec timing_last_ts_ = {};
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
