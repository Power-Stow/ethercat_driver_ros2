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

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
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
  /// Neutralise the command interfaces a controller is giving up, so nothing stale is left behind.
  /**
   * A command interface keeps its last value when the controller writing it is deactivated: nothing in
   * ros2_control clears it, and this driver goes on writing it to the drive every cycle. That is a
   * setpoint from before the switch being commanded indefinitely afterwards, and it is how a drive comes
   * back from a fault reset and steps the axis to wherever it was told to go before the fault.
   *
   * Position interfaces are released to NaN, which the channel managers turn into the configured default,
   * and for a CiA-402 position channel that default is the last read position, so the drive holds where
   * it is. Velocity and effort are released to zero. Anything else is left alone, since only the
   * interfaces that move an axis are unsafe to leave stale.
   */
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

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

  /** @brief Command every module into its safe, de-energised state with process data still running.
   *
   * Runs the cyclic exchange from the calling thread, the way `on_activate()` does for bring-up, so
   * that a CiA-402 drive can be walked down to Switch On Disabled before the frames stop. Drives
   * that lose their cyclic data while still in Operation Enabled report a synchronization error and
   * can latch a communication fault that survives into the next start-up.
   *
   * Returns once every module reports the wind-down complete or `shutdown_wind_down_timeout_s_`
   * has elapsed, whichever comes first.
   * Also runs when `on_activate()` gives up on a bus that is only partly operational,
   * so a drive that already reached Operation Enabled is not left energised.
   *
   * @param elevate_scheduling Apply `activation_thread_priority` and `activation_cpu_core` for the
   * duration of the loop. False when the caller already holds them, as the failed bring-up does:
   * the scheduling guards are not nestable.
   */
  void windDownSlaves(bool elevate_scheduling = true);

  /** @brief Name the modules that have not reached their operational state yet.
   *
   * Used for the activation timeout message, so the log says which drive the bus is waiting on
   * rather than only that it timed out.
   *
   * @return Comma separated `name (alias N position N)` entries, or `none` if every module is up.
   */
  std::string pendingModuleDescription() const;

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
  /** Budget in seconds for the shutdown wind-down loop; <= 0 skips the wind-down entirely. */
  double shutdown_wind_down_timeout_s_ = 1.0;
  /** Budget in seconds for the activation/bring-up loop; <= 0 waits indefinitely.
    * Time the master spends re-scanning the bus is added on top, up to ACTIVATION_SCAN_ALLOWANCE_S.
    */
  double activation_timeout_s_ = 10.0;
  /** Whether a failed startup config SDO download refuses the activation. Off by default, so
   *  the bus still comes up on whatever the drives already hold, as it always has. */
  bool require_startup_sdo_ = false;

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

  /// Release one command interface, named "<joint>/<interface>", to a value that commands no
  /// motion.
  void release_joint_command(const std::string & interface_name);

  /// The joint's last read position state, or NaN when it has no position state interface.
  double joint_position_state(size_t joint_index) const;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
