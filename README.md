![Slamming Works brand lockup](assets/slw-lockup.svg)

# elrs_joy

Slamming Works' ROS 2 driver for ExpressLRS (ELRS) receivers using the CRSF serial protocol.

## Demo

There is no receiver demo video or GIF in this repository yet.

## Hardware

The node connects to an ExpressLRS receiver over a CRSF serial port. This repository contains software, not a receiver BOM or schematic.

## Build and run

```bash
cd $COLCON_WS
git clone git@github.com:AlessioMorale/ros_elrs_joy.git src/ros_elrs_joy
rosdep install --ignore-src --from-paths src -y -r
colcon build --packages-up-to elrs_joy_crsf_node
source install/setup.bash
```

Run the node with the serial device for your receiver:

```bash
ros2 run elrs_joy_crsf_node crsf_node --ros-args -p serial.port:=/dev/ttyUSB0 -p joy_topic:=/joy
```

Run the package tests:

```bash
colcon test --packages-select elrs_joy_crsf_protocol elrs_joy_crsf_node
colcon test-result --verbose
```

| Package | What it does |
|---|---|
| [elrs_joy_crsf_node](elrs_joy_crsf_node/README.md) | Reads CRSF frames from serial, publishes `Joy`, handles failsafe and sends battery telemetry. |
| [elrs_joy_crsf_protocol](elrs_joy_crsf_protocol/README.md) | Parses and builds CRSF frames as a standalone C++20 library. |

## Status

**Partly works.** RC channel input, `Joy` publication, failsafe handling, diagnostics and outbound battery telemetry are implemented. The declared `serial.timeout_ms` parameter is not used yet.

The existing ROS package names and C++ namespaces are retained for compatibility.

## License

MIT. See [LICENSE](LICENSE).

## References

- [TBS CRSF specification](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md)
- [CRSF working group wiki](https://github.com/crsf-wg/crsf/wiki/)
