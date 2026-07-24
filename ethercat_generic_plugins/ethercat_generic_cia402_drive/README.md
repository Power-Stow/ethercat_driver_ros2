# `ethercat_generic_cia402_drive`

## Overview

`ethercat_generic_cia402_drive` provides a generic CiA402 EtherCAT slave plugin for `ethercat_driver_ros2`.
It maps configured RPDO/TPDO channels to ros2_control interfaces and handles CiA402 state transitions.

## Health diagnostics

The plugin implements `EcSlave::cia402Diagnostics()`, exposing the decoded CiA 402 device state
(state enum, human-readable label, raw status word, and a fault flag). When `ethercat_driver` is
launched with `publish_diagnostics:=true`, this appears per drive on `/diagnostics`, and a drive in
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
- Each row contains a monotonic timestamp (nanoseconds since dump start), cycle counter, phase, then channel values.

### Parameters

Add these optional `<param>` entries in the corresponding `<ec_module>` block:

| Name                     | Type                    | Default                                                    | Description                              |
| ------------------------ | ----------------------- | ---------------------------------------------------------- | ---------------------------------------- |
| `csv_dump_enabled`       | `bool` (`true`/`false`) | `false`                                                    | Enables CSV dump when set to `true`.     |
| `csv_dump_path`          | `string`                | `logs/log_YYYYMMDD_HHMMSS_cia402_a<alias>_p<position>.csv` | Output CSV file path.                    |
| `csv_dump_flush_every_n` | `uint`                  | `1`                                                        | Flush the file every N rows (minimum 1). |

### CSV columns

The header begins with:

- `timestamp_ns`
- `cycle`
- `phase`

Then all mapped RPDO channels (in domain order), followed by all mapped TPDO channels, each named as:

- `rpdo_<index_hex>_<sub_index_hex>_<interface_name>`
- `tpdo_<index_hex>_<sub_index_hex>_<interface_name>`

Example column names:

- `rpdo_0x6040_0x0_control_word`
- `tpdo_0x6041_0x0_status_word`

## Notes

- This feature is intended for debugging and telemetry capture.
- CSV value semantics:
  - `phase` identifies whether the row was captured during the EtherCAT `read`, `write`, or startup `update` pass.
  - RPDO columns log the raw process-data values written to the drive after applying channel factor/offset scaling.
  - TPDO columns log process-data values reconstructed from the scaled state value (inverse of channel factor/offset).
- Keep `csv_dump_enabled` disabled in normal operation to avoid disk I/O overhead.
- If `csv_dump_path` is not provided, the plugin auto-generates a timestamped filename under `logs/`.
