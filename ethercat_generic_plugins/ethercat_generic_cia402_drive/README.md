# `ethercat_generic_cia402_drive`

## Overview

`ethercat_generic_cia402_drive` provides a generic CiA402 EtherCAT slave plugin for `ethercat_driver_ros2`.
It maps configured RPDO/TPDO channels to ros2_control interfaces and handles CiA402 state transitions.

## Shutdown Wind-Down

When `ethercat_driver` deactivates, it asks every module to wind down while the cyclic exchange is
still running (`EcSlave::start_wind_down()`). This plugin uses that window to take the drive out of
Operation Enabled before the frames stop, because a drive that loses its process data while still
enabled reports AL status `0x001A` ("Synchronization error") and may latch a communication fault
that survives into the next start-up.

The control word is driven from the observed CiA-402 state, and the stop it uses depends on whether
the drive declares `quick_stop_supported` in its slave config:

| State | `quick_stop_supported: false` (default) | `quick_stop_supported: true` |
| ----- | --------------------------------------- | ---------------------------- |
| Operation Enabled | Disable Operation (`0x0007`) | Quick Stop (`0x000B`) |
| Quick Stop Active | Disable Voltage (`0x0000`) | Quick Stop, then Disable Voltage once the ramp budget is spent |
| Fault Reaction Active | Disable Voltage (`0x0000`), still winding down | Disable Voltage (`0x0000`), still winding down |
| anything else | Disable Voltage (`0x0000`), wind-down complete | Disable Voltage (`0x0000`), wind-down complete |

The wind-down reports itself complete as soon as the drive function is disabled, which is every
state except Operation Enabled, Quick Stop Active and Fault Reaction Active. Disable Voltage still
goes out on that cycle, so the drive carries the command down to Switch On Disabled on its own once
the frames stop.

Fault Reaction Active is not a stopped state: the drive is running its fault reaction, decelerating
under power, and reaches Fault by itself once that finishes. The wind-down keeps cycling until it
gets there, so the master is not released onto a moving axis. It needs no budget of its own, unlike
the quick stop ramp, because the reaction is transient by specification — a drive still in it when
`shutdown_wind_down_timeout_s` expires has something wrong with it, and the timeout warning is then
the right outcome. Fault itself is a completed state: the power stage is off, and the fault is left
standing rather than reset.

Waiting to *observe* Switch On Disabled would be stricter but wrong in practice: a drive can leave
Operation Enabled within a few cycles and then park in Ready to Switch On while its DC bus is live,
so the caller would spend its whole `shutdown_wind_down_timeout_s` waiting for a transition the
drive never makes, and warn about a slave that is already de-energised.

Half of the driver's `shutdown_wind_down_timeout_s` budget is given to the quick stop ramp,
timed on the monotonic clock from the start of the wind-down; the
remainder is left for Disable Voltage to be commanded and take effect. Drives whose quick stop
option code (`0x605A`) takes them to Switch On Disabled leave Quick Stop Active on their own and
finish early; the ones configured to hold position there are disabled once the budget is spent.

### `quick_stop_supported`

Quick Stop is the better shutdown where the drive implements it: it decelerates on the quick stop
ramp (`0x6085`) instead of dropping the power stage and leaving the axis to coast or to its brake.
It is opt-in per drive, and off by default, because a drive that does not implement it can respond
destructively.

This is not theoretical. A drive whose datasheet listed Quick Stop as not supported, and which had
neither `0x605A` nor `0x6085` configured, answered a Quick Stop from Operation Enabled by clearing
status word bit 12 (target position ignored), abandoning the commanded position and
**accelerating** the axis to roughly twice its commanded velocity under its own torque. It held it
there for a few hundred milliseconds, building a large following error, until it dropped out of
Operation Enabled by itself. The axis only stopped once the wind-down commanded Disable Voltage
and the brake engaged.

Before setting it to `true` on a drive, confirm the datasheet supports the function, that `0x605A`
and `0x6085` are configured, and verify the behaviour under motion on a test rig.

### While the wind-down runs

- the wind-down owns the control word, whatever `auto_state_transitions` is set to, so neither the
  automatic state transitions nor a fault reset can take the drive back up to Operation Enabled,
- the motion setpoints fall back to their configured defaults — zero velocity, zero torque and
  the last read position — so a setpoint left behind by a controller that has already stopped is
  not replayed into a drive that is being brought down; the mode of operation and any other
  non-motion channel are left as commanded, so the drive is not switched mode mid-stop,
- a standing fault is deliberately not reset: clearing it on the way out would hide it from the
  next start-up.

The wind-down is complete at the end of a cycle in which Disable Voltage went out,
and the status word read on that same cycle shows a state with the drive function disabled:
Not Ready to Switch On, Switch On Disabled, Ready to Switch On, Switched On or Fault.
The control word is chosen from the previous cycle's state, so a drive that faulted in between is
judged on its Fault Reaction Active rather than released while it still decelerates.
Quick Stop Active, Fault Reaction Active and a status word that decodes to no CiA-402 state keep the
frames going until a known disabled state is read, or the caller's timeout expires.

The cached EtherCAT operational flag plays no part in this, because the master refreshes it only
every few cycles and a drive can reach Operation Enabled in between.
The wind-down control word goes out whether or not the flag is set, and a slave outside OP ignores it.
A drive that never became operational reads its zeroed status word as Not Ready to Switch On,
so it completes after a single cycle.

### Reactivation

`reset_wind_down()` releases the control word and puts every command channel's `override_command`
back the way it was when the wind-down started. The driver calls it on activation, because the plugin
instance outlives a deactivate/activate cycle: without the reset the drive would be commanded
down on every cycle of the next run, and no automatic transition could take it back up to
Operation Enabled.
It also re-arms `reset_fault_on_startup` on every activation, with or without a prior wind-down,
so a drive that comes back up in Fault is cleared the same way it is on a fresh start.
The startup reset only covers a fault the drive comes up in: once the drive has been seen in a
non-fault state past Not Ready to Switch On, a fault is this session's,
and it is cleared only by `auto_fault_reset` or the fault reset command interface.
A fault reset requested through that interface in the previous session, and never consumed because
the drive was not in Fault, is discarded rather than carried into the next one.
It forgets the drive state as well, so the first status word of each activation is decoded afresh.
A drive deactivated in Fault that comes back up in Fault, perhaps for a different reason,
then counts as a new fault: its error code replaces the one latched on `last_error_code` and is logged.

## Joint Offset Startup Wrap

Some absolute encoders only report their power-up angle within a principal interval such as `[-pi, pi]`, even though
the joint should remain continuous and multi-turn after initialization. For those joints, the plugin supports an opt-in
one-time startup wrap adjustment.

When enabled, TPDO position export is held back until the slave has sent data, indicated by a non-zero status word.
Since TxPDO payloads are already valid in SAFEOP, this happens before the drive is operational. The first such TPDO
position sample is converted using the configured `joint_offset`, wrapped back into `[-pi, pi]` with `std::remainder`,
and the resulting branch correction is folded into the runtime `joint_offset`. All later samples remain unwrapped and
continuous.

This helps avoid false ros2_control joint-limit violations at startup when the drive boots on the opposite branch of the
configured `joint_offset`.

## Health diagnostics

The plugin implements `EcSlave::cia402Diagnostics()`, exposing the decoded CiA 402 device state
(state enum, a pointer to its static human-readable label, raw status word, and a fault flag).
The snapshot is filled from the cyclic loop without heap allocation,
and the label is turned into a string only by the diagnostics publisher thread.
When `ethercat_driver` is launched with `publish_diagnostics:=true`, this appears per drive on `/diagnostics`, and a drive in
`Fault` / `Fault Reaction Active` raises an `ERROR`. No configuration is required.

## CSV PDO Dump (debug)

The plugin supports optional CSV dumping of all assigned PDO channels for each EtherCAT cycle.

## Initialization Position Logging

When a CiA402 drive reaches its initialized / operational state, the plugin now emits a one-time ROS log with:

- configured module name
- EtherCAT alias and position
- raw TPDO position value
- converted exported joint position value
- configured `joint_offset`

This is intended to help debug mismatches between EtherCAT feedback and ROS joint limits, for example when a drive
boots with a position outside the URDF-configured joint bounds.

- One CSV row is written per cycle (at the end of `processData()` for the plugin instance).
- Both RPDO and TPDO channels assigned in the slave config are included.
- Each row contains a monotonic timestamp (nanoseconds since dump start), cycle counter, phase, `is_operational`, then channel values.

### Parameters

Add these optional `<param>` entries in the corresponding `<ec_module>` block:

| Name                                | Type                    | Default                                                    | Description                                                                                                                                                             |
| ----------------------------------- | ----------------------- | ---------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `joint_offset_startup_wrap_enabled` | `bool` (`true`/`false`) | `false`                                                    | Once the slave sends data, wrap the first offset-compensated TPDO position sample into `[-pi, pi]` and fold the branch correction into the runtime `joint_offset`.      |
| `csv_dump_enabled`                  | `bool` (`true`/`false`) | `false`                                                    | Enables CSV dump when set to `true`.                                                                                                                                    |
| `csv_dump_path`                     | `string`                | `logs/log_YYYYMMDD_HHMMSS_cia402_a<alias>_p<position>.csv` | Output CSV file path.                                                                                                                                                   |
| `csv_dump_flush_every_n`            | `uint`                  | `1`                                                        | Flush the file every N rows (minimum 1).                                                                                                                                |

### CSV columns

The header begins with:

- `timestamp_ns`
- `cycle`
- `phase`
- `is_operational`

Then all mapped RPDO channels (in domain order), followed by all mapped TPDO channels, each named as:

- `rpdo_<index_hex>_<sub_index_hex>_<interface_name>`
- `tpdo_<index_hex>_<sub_index_hex>_<interface_name>`

Example column names:

- `rpdo_0x6040_0x0_control_word`
- `tpdo_0x6041_0x0_status_word`

### Notes

- This feature is intended for debugging and telemetry capture.
- Keep `joint_offset_startup_wrap_enabled` disabled unless the drive's power-up encoder position is known to wrap into
  a principal interval and needs one-time branch selection.
- CSV value semantics:
  - `phase` identifies whether the row was captured during the EtherCAT `read`, `write`, or startup `update` pass.
  - `is_operational` is the plugin's current EtherCAT operational-state flag for that cycle (`0` or `1`).
  - RPDO columns log the raw process-data values written to the drive after applying channel factor/offset scaling.
  - TPDO columns log process-data values reconstructed from the scaled state value (inverse of channel factor/offset).
- Keep `csv_dump_enabled` disabled in normal operation to avoid disk I/O overhead.
- If `csv_dump_path` is not provided, the plugin auto-generates a timestamped filename under `logs/`.
