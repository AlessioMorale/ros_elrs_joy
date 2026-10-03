# elrs_joy_crsf_node

Executable `crsf_node` (node name `crsf_joy_node`). It reads CRSF frames from a serial port, maps the 16 RC channels to a `sensor_msgs/msg/Joy`, and sends `BatteryState` back to the receiver as CRSF telemetry.

```bash
ros2 run elrs_joy_crsf_node crsf_node --ros-args \
  -p serial.port:=/dev/ttyUSB0 -p joy_topic:=/joy
```

## Interfaces

| Direction | Name (default) | Type | Notes |
|---|---|---|---|
| Pub | `crsf_joy` | `sensor_msgs/msg/Joy` | Published every `joy_publish_period_ms` |
| Pub | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | See [Diagnostics](#diagnostics) |
| Sub | `battery_state` | `sensor_msgs/msg/BatteryState` | Only when `telemetry_battery_enabled` is true |
| Serial RX | `serial.port` | CRSF `0x16` RC Channels Packed | Other frame types are ignored |
| Serial TX | `serial.port` | CRSF `0x08` Battery Sensor | One frame per received `BatteryState` |

## Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `serial.port` | string | `""` | Serial device. If empty, the node runs but receives nothing and stays in failsafe. |
| `serial.baudrate` | int | `416666` | Must match the receiver's UART baud rate. |
| `serial.timeout_ms` | int | `100` | Declared, but not used yet. |
| `joy_topic` | string | `crsf_joy` | Output topic. |
| `joy_publish_period_ms` | int | `50` | `Joy` publish period (50 ms = 20 Hz). |
| `monitor_period_ms` | int | `100` | How often the node checks for the failsafe timeout. |
| `failsafe_timeout_ms` | int | `500` | Enters failsafe after this long with no RC frames. |
| `send_failsafe_continuously` | bool | `true` | `true`: keep publishing the failsafe `Joy`. `false`: publish it once when failsafe starts. |
| `failsafe_axes` | double[] | zeros | Axis values published in failsafe. Must have one entry per axis. |
| `failsafe_buttons` | int[] | zeros | Button values published in failsafe (any non-zero becomes 1). Must have one entry per button. |
| `telemetry_battery_enabled` | bool | `true` | Subscribe to `battery_topic` and send its data as telemetry. |
| `battery_topic` | string | `battery_state` | Battery input topic. |
| `diagnostics.lq_warn` | double | `70` | RC link diagnostic is WARN below this uplink link quality (%). |
| `diagnostics.lq_error` | double | `30` | RC link diagnostic is ERROR below this uplink link quality (%). |
| `diagnostic_updater.period` | double | `1.0` | `/diagnostics` publish period in s (set by `diagnostic_updater`). |

All parameters are read once at startup; changing them at runtime has no effect.

## Channel mapping

`Joy.axes[i]` and `Joy.buttons[i]` are built from the *i*-th entry of each mapping array. Channel indices go from `0` to `15` (CH1 = 0).

**Axes**: `axis_mappings.{channels, scale, offset, invert, deadzone, min, max}`. Default: channels `[0,1,2,3]`, identity transform, range `[-1, 1]`.

```
v = clamp((us - 1500) / 500, -1, 1)     # 1000–2000 µs → -1..1
v = -v                  if invert
v = 0                   if |v| < deadzone
axis = clamp(v * scale + offset, min, max)
```

**Buttons**: `button_mappings.{channels, threshold, invert}`. Default: channels `[4,5,6,7]`, threshold `0.5`.

```
v = clamp((us - 1000) / 1000, 0, 1)     # 1000–2000 µs → 0..1
button = (v >= threshold) XOR invert
```

Validation:
- If the arrays in a group have different lengths, that group falls back to its 4-entry defaults (the node logs a warning).
- If any channel is outside `0..15`, any value is non-finite, or a threshold is outside `[0, 1]`, **all** mappings are cleared and `Joy` is published with empty `axes` and `buttons`. Check the log at startup.

Example params file:

```yaml
crsf_joy_node:
  ros__parameters:
    serial:
      port: /dev/ttyAMA0
      baudrate: 420000
    joy_topic: /joy
    axis_mappings:
      channels: [0, 1, 2, 3]
      scale:    [1.0, 1.0, 1.0, 1.0]
      offset:   [0.0, 0.0, 0.0, 0.0]
      invert:   [false, true, false, false]
      deadzone: [0.05, 0.05, 0.0, 0.05]
      min:      [-1.0, -1.0, -1.0, -1.0]
      max:      [1.0, 1.0, 1.0, 1.0]
    button_mappings:
      channels:  [4, 5]
      threshold: [0.5, 0.5]
      invert:    [false, false]
    failsafe_axes: [0.0, 0.0, 0.0, 0.0]
    failsafe_buttons: [0, 0]
```

```bash
ros2 run elrs_joy_crsf_node crsf_node --ros-args --params-file crsf.yaml
```

## Failsafe

- The node starts in failsafe and leaves it when the first valid RC frame arrives.
- It goes back into failsafe if no RC frame arrives within `failsafe_timeout_ms`.
- While in failsafe it publishes `failsafe_axes` / `failsafe_buttons`. These messages have **no header stamp**.

## Diagnostics

Two tasks on `/diagnostics` (hardware ID `crsf_receiver:<serial.port>`):

**RC link**

| Level | When |
|---|---|
| ERROR | Serial port not open; or failsafe active (no RC frames for `failsafe_timeout_ms`, or none yet since start); or uplink LQ below `diagnostics.lq_error` |
| WARN | Uplink LQ below `diagnostics.lq_warn`; or CRC errors increased since the last update |
| OK | Otherwise |

Values: `failsafe`, `rc_frames`, `ms_since_rc_frame`, serial parser counters (`parser_bytes`, `parser_frames_decoded`, `parser_crc_errors`, `parser_sync_errors`, `parser_length_errors`), and from the receiver's link statistics frame: `uplink_rssi_ant1_dbm`, `uplink_rssi_ant2_dbm` (if diversity), `uplink_link_quality_pct`, `uplink_snr_db`, `active_antenna`, `rf_mode`, `tx_power_mw`. Link statistics older than 2 s are not reported and don't affect the level.

**Battery telemetry**

| Level | When |
|---|---|
| WARN | No `battery_topic` message received yet, or none for 5 s |
| OK | Messages arriving, or `telemetry_battery_enabled` is false |

Values: `messages_received`, `frames_sent`, `ms_since_battery_message`, `last_voltage_v`, `last_current_a`. If the radio shows no battery telemetry, `frames_sent` rising confirms the node is transmitting.

## Battery telemetry

`BatteryState` → CRSF Battery Sensor:

| CRSF field | Source |
|---|---|
| voltage | `voltage` (NaN → 0) |
| current | `current` (NaN → 0) |
| used capacity (mAh) | `(capacity - charge) * 1000` if both are finite and `capacity > 0`, else 0 |
| remaining % | `percentage * 100`, clamped to 0–100 |

Telemetry coming *from* the receiver (link statistics etc.) is not published to ROS.
