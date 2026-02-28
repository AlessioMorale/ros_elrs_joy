# elrs_joy_crsf_node

ROS 2 node that:
- receives CRSF `0x16 RC Channels Packed` from serial,
- publishes mapped `sensor_msgs/msg/Joy`,
- applies timeout-based failsafe,
- subscribes to `sensor_msgs/msg/BatteryState` and transmits CRSF `0x08 Battery Sensor` over serial.

## Parameters

### Core
- `joy_topic` (string, default: `crsf_joy`)
- `joy_publish_period_ms` (int, default: `50`)
- `monitor_period_ms` (int, default: `100`)
- `failsafe_timeout_ms` (int, default: `500`)
- `send_failsafe_continuously` (bool, default: `true`)

### Serial
- `serial.port` (string, default: `""`)
- `serial.baudrate` (int, default: `416666`)
- `serial.timeout_ms` (int, default: `100`)

### Joy Mapping
- `axis_mappings.channels` (int[], default: `[0,1,2,3]`)
- `axis_mappings.scale` (double[], default: `[1,1,1,1]`)
- `axis_mappings.offset` (double[], default: `[0,0,0,0]`)
- `axis_mappings.invert` (bool[], default: `[false,false,false,false]`)
- `axis_mappings.deadzone` (double[], default: `[0,0,0,0]`)
- `axis_mappings.min` (double[], default: `[-1,-1,-1,-1]`)
- `axis_mappings.max` (double[], default: `[1,1,1,1]`)
- `button_mappings.channels` (int[], default: `[4,5,6,7]`)
- `button_mappings.threshold` (double[], default: `[0.5,0.5,0.5,0.5]`)
- `button_mappings.invert` (bool[], default: `[false,false,false,false]`)

### Failsafe Output
- `failsafe_axes` (double[], default: zeros with same size as axis mappings)
- `failsafe_buttons` (int[], default: zeros with same size as button mappings)

### Telemetry TX
- `telemetry_battery_enabled` (bool, default: `true`)
- `battery_topic` (string, default: `battery_state`)

## Example Run

```bash
ros2 run elrs_joy_crsf_node crsf_node --ros-args \
  -p serial.port:=/dev/ttyUSB0 \
  -p serial.baudrate:=416666 \
  -p joy_topic:=/joy \
  -p failsafe_timeout_ms:=400 \
  -p send_failsafe_continuously:=true
```

## Telemetry Direction

Telemetry is node-originated and transmitted over serial CRSF.
In this phase, telemetry RX-to-ROS bridging is not implemented.
