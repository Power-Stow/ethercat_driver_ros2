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
`publish_diagnostics` — enable EtherCAT health diagnostics on `/diagnostics` (default `false`).
`diagnostics_period_s` — diagnostics publish period in seconds (default `1.0`).
`dc_time_diff_warn_ns` — per-slave DC system-time-difference magnitude above which a `WARN` is raised (default `1000`).

### Health diagnostics

When `publish_diagnostics` is `true`, the driver publishes `diagnostic_msgs/DiagnosticArray` on
`/diagnostics` from a dedicated non-real-time node (`ethercat_diagnostics`), leaving the cyclic
`SCHED_FIFO` loop untouched apart from cheap state snapshotting. It integrates with
`rqt_runtime_monitor` and `diagnostic_aggregator`.

Published `DiagnosticStatus` entries:

- **EtherCAT Master** — `slaves_responding`, `link_up`, master `al_states`, domain working counter and `wc_state` (ZERO/INCOMPLETE/COMPLETE), and a cumulative incomplete-cycle count used as a lost-frame proxy.
- **EtherCAT Slave: `<name>`** (one per slave) — AL state (INIT/PREOP/SAFEOP/OP), `online`/`operational`, AL status code (ESC register `0x0134`), DC system-time difference (`0x092C`), DC propagation delay (`0x0928`), and CiA 402 device state for drive slaves.
- **EtherCAT RT Timing** — cyclic-loop period min/mean/max, max jitter, and deadline-overrun count.

Levels: link down or a slave offline/not-operational or a drive fault → `ERROR`; incomplete working
counter, high DC clock drift, or loop overruns → `WARN`.

The IgH realtime API does not expose Tx-error / lost-frame counters directly, so the master status
reports the working-counter-derived incomplete-cycle count as a lost-frame proxy.

The publisher starts as soon as the master is activated, i.e. **before** the blocking bring-up loop
that waits for all slaves to reach OP. This means a slave stuck during initialization (for example
DC clocks not converging) stays observable on `/diagnostics` — the per-slave status shows the AL
state it is stuck in and the AL status code explaining why — instead of the feed only appearing once
bring-up has already succeeded.

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
loop is not preempted by IRQ threads, and requires `CAP_SYS_NICE`/real-time rlimits (failures are
logged and fall back to normal scheduling). Process memory locking is not handled here — enable the
`controller_manager` `lock_memory` parameter (mlockall is process-wide).

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
