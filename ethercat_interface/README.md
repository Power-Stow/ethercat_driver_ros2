# `ethercat_interface`

## Overview

`ethercat_interface` is the low-level EtherCAT master/slave abstraction layer used by
`ethercat_driver`. It wraps the IgH EtherLab realtime master (`ecrt_*`) into C++ classes and
defines the `EcSlave` plugin base that concrete slave plugins (for example the CiA 402 drive)
implement.

**Keywords:** `ethercat`, `igh`, `etherlab`, `realtime`

### Responsibility and Scope

- **Primary purpose:** Own the IgH master lifecycle, domain/PDO registration, the cyclic
  read/write exchange, distributed-clock synchronization, and per-slave state tracking.
- **Out of scope:** `ros2_control` integration (that lives in `ethercat_driver`) and slave-specific
  PDO semantics (those live in slave plugins).
- **Package type:** library package

## Package Organization

```text
ethercat_interface/
├── include/ethercat_interface/     # Public headers (master, slave base, PDO/SDO/sync managers)
│   ├── ec_master.hpp               # IgH master wrapper + cyclic loop
│   ├── ec_slave.hpp                # EcSlave plugin base class
│   └── ec_diagnostics.hpp          # POD health-snapshot structs (no ROS dependency)
├── src/                            # Implementations
├── test/
├── CMakeLists.txt
└── package.xml
```

## Dependencies

### Runtime Dependencies

- `ethercat` (IgH EtherLab): the realtime EtherCAT master (`ecrt.h`, `libethercat`).
- `rclcpp`: logging only (state-change messages).

### Build/Test Dependencies

- `ament_cmake_gtest`: C++ unit testing.

## Libraries

- `ec_master` / `EcMaster`: requests the IgH master, registers domains and PDOs, and runs the
  cyclic exchange (`update()`, `readData()`, `writeData()`). It also tracks master, domain, and
  per-slave state.
- `EcSlave`: abstract base for slave plugins (`processData`, `syncs`, `channels`, `setupSlave`).

## Logging & Diagnostics

State transitions (master AL state, link, domain working counter, per-slave AL state) are logged via
`RCLCPP_*` on change.

For machine-readable health monitoring, `EcMaster` maintains an optional diagnostics snapshot
(`ec_diagnostics.hpp`): master/domain state, a lost-frame proxy derived from the domain working
counter, and per-slave AL status code (ESC register `0x0134`), DC system-time difference (`0x092C`),
DC propagation delay (`0x0928`), and CiA 402 device state. Collection is opt-in via
`setDiagnosticsEnabled(true)` (called before `activate()`, so per-slave register requests can be
created) and read with the thread-safe `getDiagnostics()`. `ethercat_driver` publishes this snapshot
to `/diagnostics`; see that package's README.

Slave plugins expose their device-specific health by overriding `EcSlave::cia402Diagnostics()`.

## Testing

```bash
colcon test --packages-select ethercat_interface
colcon test-result --verbose
```

## Known Limitations

- Tx-error / lost-frame counters are not available through the IgH realtime API; the diagnostics
  snapshot reports a working-counter-derived incomplete-cycle count as a lost-frame proxy.
