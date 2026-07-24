# EtherCAT Diagnostics & Health Monitoring

## Context

The stack needs runtime health visibility into the EtherCAT bus. Today EtherCAT state
(master state, slave AL states, domain working counter) is only surfaced as `RCLCPP_*`
log lines emitted on change inside `EcMaster` — nothing is published, so no node, RViz
panel, or aggregator can react to bus health. We want a standard, machine-consumable
EtherCAT health feed.

The master is **IgH EtherLab** (`ecrt_*` API), and the driver
(`third_party/ethercat_driver_ros2`, a Power-Stow fork submodule) is a **ros2_control
`SystemInterface` plugin** running inside `controller_manager` — not a standalone node.
The DC/ESC registers (0x092C, 0x0928, 0x0134), master state, and domain working counter
are only reachable through the IgH master handle, which lives exclusively inside
`EcMaster`. Therefore acquisition **must** live in the fork. The chosen output is the
standard `diagnostic_msgs/DiagnosticArray` on `/diagnostics` via `diagnostic_updater`, so
it integrates for free with `rqt_runtime_monitor` / `diagnostic_aggregator`.

Intended outcome: an opt-in (`publish_diagnostics` parameter, default off) diagnostics
feed published at ~1 Hz from a **non-RT** thread, leaving the `SCHED_FIFO` cyclic loop
untouched except for cheap snapshotting.

## Diagnostics published (v1)

Per-slave `DiagnosticStatus` (keyed `ethercat: <name> (alias:pos)`):
- AL state (INIT/PREOP/SAFEOP/OP), online, operational flags
- **AL status code** (ESC reg 0x0134) — reason a slave left OP
- **System time difference** (DC reg 0x092C) — clock drift vs reference
- **Propagation delay** (DC reg 0x0928) — read once after OP (effectively static)
- CiA402 device state (fault / quick-stop / operation-enabled) for drive slaves

Master/bus `DiagnosticStatus`:
- Master state: `slaves_responding`, `al_states`, `link_up`
- Domain working counter (WKC), expected WKC, `wc_state` (ZERO/INCOMPLETE/COMPLETE)
- Lost-frame counter derived from `wc_state != COMPLETE` cycles (proxy; `ecrt` RT API
  does not expose tx-errors/lost-frames directly — noted below)

RT-timing `DiagnosticStatus`:
- Cyclic loop period, jitter (min/max/stddev), deadline-overrun count

## Approach

### 1. Diagnostics snapshot struct (`ethercat_interface`)
New header `ethercat_interface/include/ethercat_interface/ec_diagnostics.hpp`: a plain-POD
`MasterDiagnostics` + `SlaveDiagnostics` struct (no ROS deps). `EcMaster` owns one
instance guarded by a `std::mutex`; the RT loop writes it, the diagnostics thread copies
it out under lock.

### 2. Acquisition in `EcMaster` (`ec_master.hpp/.cpp`)
- Extend `checkMasterState` / `checkSlaveStates` / `checkDomainState` (they already run
  every `check_state_frequency_` cycles) to also **store** their values into the snapshot,
  not just log on change.
- Add ESC register reads via IgH **register requests**
  (`ecrt_slave_config_create_reg_request`, `ecrt_reg_request_read/state/data`): create
  per-slave requests for 0x0134, 0x092C, 0x0928 during `activate()`; service them in
  `readData()` at a low cadence (e.g. every ~1000 cycles), 0x0928 once after OP.
- Add public getter `MasterDiagnostics getDiagnostics() const` (locks, returns a copy).

### 3. CiA402 device-state hook
Add a virtual `std::optional<SlaveDiagnostics::Cia402Info> EcSlave::diagnostics()` to
`ec_slave.hpp` (default `std::nullopt`). Override in
`ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp` to return the already-decoded
`state_` (`DeviceState`) — reuses existing `deviceState()` logic; no new PDO wiring.

### 4. RT-timing measurement (`ethercat_driver.cpp`)
In the cyclic path (`read()`/`write()` and the `on_activate` bring-up loop), sample
`clock_gettime(CLOCK_MONOTONIC)` deltas, track period min/max/mean and count overruns vs
`interval_`. Store into the snapshot.

### 5. Publishing (`ethercat_driver.hpp/.cpp`)
- New parameter `publish_diagnostics` (default `false`) + threshold params
  (`diagnostics_period_s`, `dc_time_diff_warn_ns`).
- On `on_configure` (when enabled): create a dedicated `rclcpp::Node`
  (`ethercat_diagnostics`), a `diagnostic_updater::Updater` bound to it, register one
  task per slave + master/bus + RT-timing, and start a `std::thread` running a
  single-threaded executor/timer that calls `updater.force_update()` at
  `diagnostics_period_s`. Each task callback pulls `master_->getDiagnostics()` and fills
  `DiagnosticStatus` levels (OK/WARN/ERROR per thresholds: link-down→ERROR,
  slave-not-OP→ERROR, wc_state≠COMPLETE→WARN, time-diff>threshold→WARN,
  CiA402 fault→ERROR).
- Join the thread and reset the node in `on_deactivate`/`on_shutdown`.
- This keeps publishing entirely off the RT thread.

### 6. Build + docs
- `ethercat_driver/package.xml` + `CMakeLists.txt`: add `diagnostic_updater`,
  `diagnostic_msgs` deps.
- Update `ethercat_driver/README.md` and `ethercat_generic_cia402_drive/README.md`
  (new parameters, published `/diagnostics` contents) per `docs/package_readme_template.md`.

## Critical files

- `third_party/ethercat_driver_ros2/ethercat_interface/include/ethercat_interface/ec_diagnostics.hpp` (new)
- `.../ethercat_interface/include/ethercat_interface/ec_master.hpp` + `src/ec_master.cpp`
- `.../ethercat_interface/include/ethercat_interface/ec_slave.hpp`
- `.../ethercat_generic_plugins/ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp`
- `.../ethercat_driver/include/ethercat_driver/ethercat_driver.hpp` + `src/ethercat_driver.cpp`
- `.../ethercat_driver/{package.xml,CMakeLists.txt,README.md}`

## Notes / caveats

- **Submodule boundary:** all changes are inside the `ethercat_driver_ros2` fork
  submodule and must be committed to the `Power-Stow/ethercat_driver_ros2` repo, then the
  submodule pointer bumped in `auto_loader`. Design is opt-in/parameter-gated so it is
  upstreamable to ICube-Robotics.
- **tx-errors / lost-frames:** not in the `ecrt` realtime API. v1 derives a lost-frame
  proxy from `wc_state`. Exact IgH master counters (tx_errors, real lost-frame count)
  would require reading the IgH chardev/`ethercat` CLI stats — deferred; can be added to
  the master `DiagnosticStatus` later in the same non-RT thread if needed.
- **Per CLAUDE.md**, on acceptance this doc is copied into the changed package as
  `third_party/ethercat_driver_ros2/ethercat_driver/2026-07-24_ethercat-diagnostics.md`.

## Verification

- Build with `ps-build` / `colcon build` (user runs manually).
- Launch bringup with `publish_diagnostics:=true`; confirm `ros2 topic echo /diagnostics`
  shows one status per slave + master + RT-timing, refreshing at the configured period.
- `ros2 run rqt_runtime_monitor rqt_runtime_monitor` shows OK/WARN/ERROR roll-up.
- Fault injection: unplug a slave / force a drive fault and confirm the corresponding
  status flips to ERROR and AL status code / device state reflect the cause.
- (All launch/build/run steps are for the user to execute manually.)
