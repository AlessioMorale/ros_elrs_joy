# elrs_joy_crsf_protocol

C++20 library for parsing and building CRSF frames. It has no ROS dependencies; it only uses ament for building and installing.

## Supported messages

Defined in `crsf/messages.hpp`. Each one has `from_frame()` and `to_frame()`.

| Class | Type | Payload |
|---|---|---|
| `RCChannelsMessage` | `0x16` | 16 channels, in µs (converted from 11-bit ticks) |
| `BatterySensorMessage` | `0x08` | voltage V, current A, used mAh, % |
| `LinkStatisticsMessage` | `0x14` | RSSI, LQ, SNR, RF mode, … |
| `AttitudeMessage` | `0x1E` | pitch, roll, yaw (raw int16) |
| `FlightModeMessage` | `0x21` | string |
| `HeartbeatMessage` | `0x0B` | – |

## Usage

```cpp
#include "elrs_joy_crsf_protocol/crsf/packets.hpp"
#include "elrs_joy_crsf_protocol/crsf/messages.hpp"
using namespace elrs_joy_crsf_protocol::crsf;

// Parse: feed raw bytes; the callback runs for each frame with a valid CRC
Packets parser([](const Message::Frame & frame) {
  if (auto rc = RCChannelsMessage::from_frame(frame)) {
    uint16_t ch1_us = rc->payload.channels[0];
  }
});
for (uint8_t b : bytes) parser.process_byte(b);

// Build: the default sync/address byte is the flight controller (0xC8)
BatterySensorMessage msg;
msg.payload = {.voltage = 12.4F, .current = 3.1F, .usedCapacity = 450, .batteryPercent = 80};
std::vector<uint8_t> out = msg.to_frame().data;
```

The parser accepts the sync bytes `0xC8`, `0x00` and `0xEE`, and counts sync, length and CRC errors. Read the counters with `get_statistics()` and clear them with `reset_statistics()`.

## CMake

```cmake
find_package(elrs_joy_crsf_protocol REQUIRED)
target_link_libraries(my_target elrs_joy_crsf_protocol::elrs_joy_crsf_protocol)
```
