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
`activation_timeout_s` — budget in seconds for the activation/bring-up loop (default `10.0`); `<= 0` waits indefinitely, and a positive value must exceed the loop's one second initial delay. Time the master spends re-scanning the bus does not count, up to 30 s of it.
`require_startup_sdo` — refuse the activation when a startup config SDO download fails (default `false`, which brings the bus up anyway).
`publish_diagnostics` — enable EtherCAT health diagnostics on `/diagnostics` (default `false`).
`diagnostics_period_s` — diagnostics publish period in seconds (default `1.0`, valid range `[0.001, 3600]`; out-of-range values fall back to the default).
`dc_time_diff_warn_ns` — per-slave DC system-time-difference magnitude above which a `WARN` is raised (default `10000`, valid range `[0, 2147483647]`; out-of-range values fall back to the default).
`dt_tolerated_overrun` — fraction of the expected cycle period a cycle may exceed before it counts as an overrun, i.e. the threshold is `(1 + dt_tolerated_overrun) / control_frequency` (default `0.5`; must be finite and `>= 0`, otherwise the default is used).

### Health diagnostics

When `publish_diagnostics` is `true`, the driver publishes `diagnostic_msgs/DiagnosticArray` on
`/diagnostics` from a dedicated non-real-time node (`ethercat_diagnostics_<hardware_name>`), leaving the cyclic
`SCHED_FIFO` loop untouched apart from cheap state snapshotting. It integrates with
`rqt_runtime_monitor` and `diagnostic_aggregator`.
The node name and hardware ID are derived from the `ros2_control` hardware component name,
with characters invalid in a node name replaced by `_` plus a stable hash suffix to keep sanitized names unique,
so multiple driver instances in one controller manager publish distinguishable statuses,
since `diagnostic_updater` prefixes each status name with the node name.

Published `DiagnosticStatus` entries:

- **EtherCAT Master** — `slaves_responding`, `link_up`, master `al_states`, domain working counter and `wc_state` (ZERO/INCOMPLETE/COMPLETE), and a cumulative incomplete-cycle count used as a lost-frame proxy.
- **EtherCAT Slave: `<name>`** (one per slave) — AL state (INIT/PREOP/SAFEOP/OP), `online`/`operational`, AL status code (ESC register `0x0134`), DC system-time difference (`0x092C`) and DC propagation delay (`0x0928`) for DC-enabled slaves, and CiA 402 device state for drive slaves.
  If several modules share a configured name, the module index is appended to keep task names unique.
- **EtherCAT RT Timing** — cyclic-loop period min/mean/max, max jitter (largest deviation from the expected period, early or late), cumulative deadline-overrun count, and overruns since the previous report.

Levels: link down, a slave offline/not-operational/not configured by the master, or a drive fault → `ERROR`;
incomplete working counter, high DC clock drift, or loop overruns since the previous report → `WARN`.
A failure to set up the diagnostics node is logged as an error and activation continues without diagnostics.

The IgH realtime API does not expose Tx-error / lost-frame counters directly, so the master status
reports the working-counter-derived incomplete-cycle count as a lost-frame proxy.
Counting starts only once the domain working counter has first reached COMPLETE,
so the incomplete cycles expected while slaves transition towards OP are not counted as losses.
Until the first EtherCAT cycle has produced a snapshot, the master and slave statuses report `OK` with
"Waiting for first EtherCAT cycle" rather than a spurious link-down error.
If a refresh of a register-derived value (AL status code, DC system-time difference) fails,
that value is omitted until the next successful read instead of reporting the stale sample.
The DC propagation delay is read once and read again after the slave has been offline.

The publisher starts as soon as the master is activated, i.e. **before** the blocking bring-up loop
that waits for all slaves to reach OP. This means a slave stuck during initialization (for example
DC clocks not converging) stays observable on `/diagnostics` — the per-slave status shows the AL
state it is stuck in and the AL status code explaining why — instead of the feed only appearing once
bring-up has already succeeded.

When bring-up fails, on a timeout or a shutdown request, `on_activate()` stops the diagnostics
before it releases the master.
If any activation step after the publisher has started throws instead,
the publisher is stopped and joined before the exception propagates.

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
  and alias/position, and how long the master spent re-scanning the bus.

The master re-scans the bus on its own whenever the number of responding slaves changes,
for example while power-cycled slaves come back one by one,
and it configures no slave while it scans.
That time is therefore added to the budget rather than counted against it,
so a bus that is only slow to come up is not failed for it.
The allowance is capped at 30 s, so a bus that never stops re-scanning still gives up.
The loop logs each scan it observes.

For reference, a healthy bring-up of a single DC drive takes about six seconds including the
loop's initial one second delay, and that delay counts against the budget. A bus carrying more
DC slaves needs a larger one.
The delay makes no update, so `on_init()` rejects a positive budget that does not exceed it,
which could never observe the bus coming up.
Both exits above are checked during the delay as well as during the loop itself.
Every wake-up is capped at the deadline it serves, so a control period longer than the remaining
budget does not overshoot it.
A shutdown request and the timeout are both checked before each update, so no update is started
once either applies, and a bus that has come up is only accepted when its update finished inside the
budget, so a late wake-up or a slow update cannot turn into a success past it.

If you ever experience a slave who won't initialize (i.e. stuck in `INIT`) and whose identity reads `0x00000000:0x00000000` in `ethercat slaves`, it is because the device has not released its EEPROM to the master — its own CPU still owns register `0x0500` — so the master cannot match it against the configured vendor and product code and never configures it.
`ethercat rescan` usually clears that.

### Startup SDO failures

A failed download is reported at `ERROR` with the `errno` from the transfer and the CoE abort code.

A zero abort code means no CoE abort was reported. That usually means the transfer did not reach the
drive's CoE layer — the slave is unreachable or its mailbox is not up — rather than the drive
rejecting the object, but a zero is not proof of it. The message says so,
because the previous wording reported the abort code alone and so read as `Error: 0` for exactly
the case where the drive was never spoken to.

What happens next is governed by `require_startup_sdo`:

| `require_startup_sdo` | Behaviour |
| --------------------- | --------- |
| `false` (default) | The bus comes up anyway and a `WARN` summary says how many downloads failed. This is what the driver has always done, so an existing configuration is unaffected. |
| `true` | `configNetwork()` returns `ERROR` and the activation is refused. |

Enable it where the startup SDOs carry values the machine depends on — a speed limit, torque
limits, control gains. A drive that did not receive them runs on whatever it already holds, which
may be the defaults of whoever configured it last, and nothing downstream can tell the difference.

### Released command interfaces

When a controller stops, `perform_command_mode_switch()` releases each command interface it held
to a value that commands no motion.
Velocity and effort are set to zero.
Position is held at the joint's last read position, rather than set to NaN,
because a NaN makes a channel write its configured default,
and only the CiA-402 plugin keeps that default at the last read position.
A joint with no position reading keeps its last command.
The control word, the mode of operation and the fault reset are left as they are.

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
reports `EcSlave::wind_down_complete()`, or until `shutdown_wind_down_timeout_s` expires.
The timeout is measured on the monotonic clock rather than counted in cycles,
so a scheduling stall or an overrunning update cannot stretch deactivation past it.
The first wind-down frame goes out straight away, and no update is started once the next one would
begin at or past the deadline, so a control period longer than the remaining budget cannot either. Modules
with nothing to wind down report completion immediately, so the loop costs a single cycle for a bus
that carries none. `ethercat_generic_cia402_drive` uses it to disable and then de-energise
the drive, or to Quick Stop it where its slave config declares `quick_stop_supported`.

`activation_thread_priority` and `activation_cpu_core` apply to this loop too, for the same reason
they apply to bring-up: a scheduling gap here stops the frames for longer than a DC slave's sync
watchdog allows, which is the failure the wind-down exists to avoid.
When `on_activate()` winds down a failed bring-up, the thread is still under the bring-up's own
scheduling, so the wind-down keeps that rather than applying it a second time.

`on_shutdown()` runs the same wind-down, which only does anything when the component is finalized
straight from ACTIVE; after `on_deactivate()` the master is already released and it returns
immediately.
`on_error()` runs it too, because ERROR can be entered straight from ACTIVE after an exception in
a cyclic read or write, with drives still in Operation Enabled.

`on_activate()` runs it as well when it gives up on the bus, on `activation_timeout_s` or a
shutdown request. Some drives may already be in Operation Enabled while another module is still
pending, and releasing the master under them is the same failure. Modules that never became
operational report the wind-down complete after a single cycle.

`on_activate()` calls `EcSlave::reset_wind_down()` on every module before it brings the bus up.
The modules are created once, in `on_init()`, and outlive a deactivate/activate cycle, so a
wind-down left in force would go on commanding the slaves down instead of letting them come back
up.

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
- `diagnostic_updater`, `diagnostic_msgs`: EtherCAT health diagnostics publishing.

### Build/Test Dependencies

- `ament_cmake_gtest`: C++ unit testing.

## Testing

### Unit Tests

- `test_ethercat_safety_driver`: transfer/safety configuration parsing coverage.
- `test_loader_backed_transmission_coupling`: loader-backed simple/differential mapping behavior and unsupported type rejection.
