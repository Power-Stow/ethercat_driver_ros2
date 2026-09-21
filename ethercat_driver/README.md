# `ethercat_driver`

## Overview

`ethercat_driver` provides a `ros2_control` system interface for EtherCAT-based joints, sensors, and GPIO modules.

**Keywords:** `ethercat`, `ros2_control`, `hardware_interface`

### Responsibility and Scope

- **Primary purpose:** Bridge EtherCAT modules into a `ros2_control` `SystemInterface` implementation.
- **Out of scope:** Controller logic and high-level robot behavior.
- **Package type:** library package

## Transmission Support

The driver supports mixed setups where some joints use ROS 2 transmissions while others are direct passthrough joints.

- Supported transmission types:
  - `transmission_interface/SimpleTransmission`
  - `transmission_interface/DifferentialTransmission`
- Transmission mappings are loaded via the ROS 2 transmission loaders (`SimpleTransmissionLoader` and `DifferentialTransmissionLoader`).
- Multi-DOF transmission roles such as `joint1`, `joint2`, `actuator1`, and `actuator2` define the
  transmission handle slot order and are honored independently of URDF declaration order.
- For transmitted actuator command interfaces, `position`, `velocity`, and `effort` are mapped through the
  transmission. Other command interfaces, such as `reset_fault`, pass through from exported hardware commands to the
  raw actuator command buffer used by EtherCAT plugins.
- Joints not included in any transmission continue to use direct state/command passthrough.
- Lifecycle error recovery keeps configured EtherCAT module/plugin instances so a recovered activation can reconfigure
  the master with the same slave set.

### EtherCAT-specific constraint for transmitted joints

For this driver, actuator names in transmission definitions must correspond to existing joint component names (the underlying EtherCAT module data channels in this driver are joint-indexed).

## Hardware parameters

Set on the `<hardware>` element of the `ros2_control` system.

`master_id` — EtherCAT master index to request (default `0`).
`control_frequency` — cyclic exchange rate in Hz; also the SYNC0 cycle time for DC slaves (default `100`).
`dc_sync0_shift_ns` — SYNC0 shift time in nanoseconds applied to DC slaves (default `0`).
`activation_thread_priority` — SCHED_FIFO priority applied to the activation/bring-up loop only; `<= 0` (default) keeps normal scheduling.
`activation_cpu_core` — CPU core the activation/bring-up loop is pinned to; `< 0` (default) leaves the CPU affinity unchanged.
`shutdown_wind_down_timeout_s` — budget in seconds for the shutdown wind-down loop (default `1.0`); `<= 0` skips the wind-down.
`activation_timeout_s` — budget in seconds for the activation/bring-up loop (default `10.0`); `<= 0` waits indefinitely.

### Real-time activation loop

The blocking bring-up loop in `on_activate()` runs `master_->update()`, which sends the cyclic
frames that discipline the Distributed Clocks. It runs on the (non-real-time) activation thread, not
the `controller_manager` real-time update thread, so under a loaded/shared CPU its jitter can exceed
the master's DC synchronization threshold — then every DC slave stalls for the full per-slave DC
sync-wait before the master proceeds, dominating startup time.

`activation_thread_priority` and `activation_cpu_core` let the loop run under `SCHED_FIFO` (and,
optionally, pinned to a dedicated/isolated core) for its duration; the previous scheduling policy,
priority and affinity are restored when activation completes. Both are opt-in and default to
no-ops, preserving the original behavior. Choose a priority above the threaded-IRQ priority so the
loop is not preempted by IRQ threads. Both require `CAP_SYS_NICE` or the real-time rlimits
(`rtprio`); if the elevation fails it is logged as a warning and activation fails. Process memory
locking is not handled here — enable the `controller_manager` `lock_memory` parameter (mlockall is
process-wide).

### Bounded, interruptible bring-up

The bring-up loop in `on_activate()` waits for every module to report itself operational. It runs on
whichever thread delivered the robot description, which for a `ros2_control_node` driven by the
`/robot_description` topic is the executor thread: while the loop spins, the node answers no service
and honours no signal. Unbounded, a bus that never reaches OP therefore cost a `SIGKILL`, a
controller spawner timing out against `/controller_manager/list_controllers`, and about twenty
seconds of teardown.

The loop now gives up in two cases, and returns `ERROR` from `on_activate()` after releasing the
master:

- `rclcpp::ok()` goes false, so Ctrl-C is honoured while the bus is still coming up.
- `activation_timeout_s` elapses. The message names the modules still waited on, by configured name
  and alias/position.

For reference, a healthy bring-up of a single DC drive takes about six seconds including the
loop's initial one second delay, and that delay counts against the budget. A bus carrying more
DC slaves needs a larger one.

A slave whose identity reads `0x00000000:0x00000000` in `ethercat slaves` while stuck in `INIT` has
not released its EEPROM to the master — its own CPU still owns register `0x0500` — so the master
cannot match it against the configured vendor and product code and never configures it.
`ethercat rescan` usually clears that.

### Startup SDO failures

The per-module startup SDOs carry the drive's speed limit, torque limits and control gains.
A failed download is reported at `ERROR` with the `errno` from the transfer and the CoE abort code,
and `configNetwork()` then refuses to continue rather than bringing the bus up on whatever the
drives happen to hold.

A zero abort code means the transfer never reached the drive's CoE layer — the slave is unreachable
or its mailbox is not up — as opposed to the drive rejecting the object. The message says so,
because the previous wording reported the abort code alone and so read as `Error: 0` for exactly
the case where the drive was never spoken to.

### Shutdown wind-down

`on_deactivate()` releases the EtherCAT master, which stops the cyclic frames. A slave still in
EtherCAT OP at that moment sees its process data disappear: a DC drive reports AL status `0x001A`
("Synchronization error") and can latch a communication fault of its own. On some drives that latch
survives the next master activation, so the drive walks its CiA-402 state machine all the way up to
Operation Enabled on the following start-up while its cyclic motion task stays inhibited — it looks
healthy, reports a clean status word, and silently ignores every setpoint.

Before releasing the master, `on_deactivate()` therefore runs a blocking cyclic loop of its own, the
mirror image of the bring-up loop in `on_activate()`. Each module is asked to wind down
(`EcSlave::start_wind_down()`) and the loop keeps the process data flowing until every module
reports `EcSlave::wind_down_complete()`, or until `shutdown_wind_down_timeout_s` expires. Modules
with nothing to wind down report completion immediately, so the loop costs a single cycle for a bus
that carries none. `ethercat_generic_cia402_drive` uses it to disable and then de-energise
the drive, or to Quick Stop it where its slave config declares `quick_stop_supported`; see that
package's README for the sequence and for why Quick Stop is opt-in.

`activation_thread_priority` and `activation_cpu_core` apply to this loop too, for the same reason
they apply to bring-up: a scheduling gap here stops the frames for longer than a DC slave's sync
watchdog allows, which is the failure the wind-down exists to avoid.

`on_shutdown()` runs the same wind-down, which only does anything when the component is finalized
straight from ACTIVE; after `on_deactivate()` the master is already released and it returns
immediately.

## Package Organization

```text
ethercat_driver/
├── include/ethercat_driver/         # Public headers
├── src/                             # Driver and transmission coupling implementations
├── test/                            # Unit tests
├── examples/                        # Example ros2_control/xacro configurations
├── CMakeLists.txt
└── package.xml
```

## Dependencies

### Runtime Dependencies

- `hardware_interface`: base `SystemInterface` API.
- `transmission_interface`: transmission loaders and runtime mapping primitives.
- `ethercat_interface`: EtherCAT master/slave abstraction layer.
- `pluginlib`, `rclcpp`, `rclcpp_lifecycle`: plugin and lifecycle integration.

### Build/Test Dependencies

- `ament_cmake_gtest`: C++ unit testing.

## Testing

### Unit Tests

- `test_ethercat_safety_driver`: transfer/safety configuration parsing coverage.
- `test_loader_backed_transmission_coupling`: loader-backed simple/differential mapping behavior and unsupported type rejection.
