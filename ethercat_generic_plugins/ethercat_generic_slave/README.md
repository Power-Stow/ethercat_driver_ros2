# `ethercat_generic_slave`

## Overview

`ethercat_generic_slave` provides `GenericEcSlave`, an `ethercat_interface::EcSlave` plugin for any EtherCAT slave whose process data can be described in a YAML slave config.
It maps the configured RPDO and TPDO channels to the ros2_control command and state interfaces of the joint, sensor or `<gpio>` it is declared under.

**Keywords:** `ethercat`, `pdo`, `ros2_control`, `plugin`

### Responsibility and Scope

- **Primary purpose:** Exchange configured PDO channels with ros2_control interfaces, and declare the configured startup SDOs and sync managers.
- **Out of scope:** Device-specific state machines, such as CiA-402, which live in `ethercat_generic_cia402_drive`.
- **Package type:** library package (pluginlib plugin)

## Package Organization

```text
ethercat_generic_slave/
├── include/ethercat_generic_plugins/   # GenericEcSlave header
├── src/                                # GenericEcSlave implementation
├── test/
├── ethercat_plugins.xml                # pluginlib export: ethercat_generic_plugins/GenericEcSlave
├── CMakeLists.txt
└── package.xml
```

## Dependencies

### Runtime Dependencies

- `ethercat_interface`: `EcSlave` base class and the PDO, SDO and sync manager configuration.
- `pluginlib`: plugin export.
- `yaml_cpp_vendor`: slave config parsing.

### Build/Test Dependencies

- `ament_cmake_gmock`: C++ unit testing.

## Usage

Declare the module inside an `<ec_module>` with `<plugin>ethercat_generic_plugins/GenericEcSlave</plugin>`, and point `slave_config` at its YAML file:

```xml
<ec_module name="ek1914">
  <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
  <param name="alias">0</param>
  <param name="position">0</param>
  <param name="slave_config">beckhoff_ek1914.yaml</param>
</ec_module>
```

## Configuration

The slave config holds `vendor_id`, `product_id` and optionally `assign_activate`, `sdo`, `sm`, `rpdo` and `tpdo`.
See `ethercat_driver/examples/configurations/` for complete files.
Each PDO channel maps one entry to a `command_interface` (RPDO) or a `state_interface` (TPDO):

| Key | Description |
| --- | ----------- |
| `index`, `sub_index`, `type` | Object dictionary entry and its data type. |
| `command_interface` / `state_interface` | ros2_control interface the entry is exchanged with. |
| `factor`, `offset` | Scaling, applied as `factor * value + offset` on both write and read. |
| `factor_from_sdo` | Reads the factor from the slave at activation instead. |
| `default` | RPDO value written while the command is NaN, or when no command interface is mapped. |

### Position Hold

While the command on a `position` RPDO channel is NaN, the plugin writes the module's latest `position` state reading instead of the configured `default`, so the axis is held where it is.
This takes a single-interface TPDO channel mapped to the `position` state interface of the same module.
The `default` is still written before the first reading exists, and on modules without a `position` state interface.

## Testing

```bash
colcon test --packages-select ethercat_generic_slave
colcon test-result --verbose
```

## Known Limitations

- The position hold covers only single-interface channels named `position`; channels with `data_mapping` always write their `default` for a NaN command.
