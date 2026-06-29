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
- Joints not included in any transmission continue to use direct state/command passthrough.

### EtherCAT-specific constraint for transmitted joints

For this driver, actuator names in transmission definitions must correspond to existing joint component names (the underlying EtherCAT module data channels in this driver are joint-indexed).

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
