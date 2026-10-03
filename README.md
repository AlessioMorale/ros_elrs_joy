# elrs_joy

![License](https://img.shields.io/badge/License-MIT-blue.svg)

ROS 2 driver for ExpressLRS (ELRS) / CRSF receivers. It reads RC channels from a serial port, publishes them as `sensor_msgs/msg/Joy` for teleoperation, and sends battery telemetry back to the handset.

| Package | Description |
|---|---|
| [elrs_joy_crsf_node](elrs_joy_crsf_node/README.md) | `crsf_node`: serial CRSF → `Joy`, failsafe, battery telemetry |
| [elrs_joy_crsf_protocol](elrs_joy_crsf_protocol/README.md) | CRSF frame parser/serializer (plain C++20 library, no ROS deps) |

## Build

```bash
cd $COLCON_WS
git clone git@github.com:AlessioMorale/ros_elrs_joy.git src/ros_elrs_joy
rosdep install --ignore-src --from-paths src -y -r
colcon build --packages-up-to elrs_joy_crsf_node
source install/setup.bash
```

## Run

```bash
ros2 run elrs_joy_crsf_node crsf_node --ros-args -p serial.port:=/dev/ttyUSB0 -p joy_topic:=/joy
```

See [elrs_joy_crsf_node](elrs_joy_crsf_node/README.md) for topics, parameters and channel mapping.

## Test

```bash
colcon test --packages-select elrs_joy_crsf_protocol elrs_joy_crsf_node
colcon test-result --verbose
```

## References

- [TBS CRSF spec](https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md)
- [CRSF working group wiki](https://github.com/crsf-wg/crsf/wiki/)
