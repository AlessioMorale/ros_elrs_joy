// Copyright 2025 Alessio Morale <alessiomorale-at-gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// SPDX-FileCopyrightText: 2025 Alessio Morale <alessiomorale-at-gmail.com>
// SPDX-License-Identifier: mit
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <span>  // NOLINT
#include <vector>

namespace crsf
{

enum class ValidationStatus : uint8_t
{
  OK = 0,
  INVALID_NOT_ENOUGH_DATA,
  INVALID_CRC,
  UNSUPPORTED_SYNC
};

enum class Address : uint8_t
{
  CRSF_ADDRESS_BROADCAST = 0,             // Broadcast (all devices process packet)
  CRSF_ADDRESS_USB = 0x10,                // USB
  CRSF_ADDRESS_BLUETOOTH = 0x12,          // Bluetooth module
  CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,   // TBS CORE PNP PRO
  CRSF_ADDRESS_RESERVED1 = 0x8A,          // Reserved, for one
  CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,     // External current sensor
  CRSF_ADDRESS_GPS = 0xC2,                // External GPS
  CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,       // External Blackbox logging device
  CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,  // Flight Controller (Betaflight / iNav)
  CRSF_ADDRESS_RESERVED2 = 0xCA,          // Reserved, for two
  CRSF_ADDRESS_RACE_TAG = 0xCC,           // Race tag
  CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,  // Handset (EdgeTX), not transmitter
  CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,      // Receiver hardware (TBS Nano RX / RadioMaster RP1)
  CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,   // Transmitter module, not handset
  CRSF_ADDRESS_ELRS_LUA = 0xEF            // Non-Standard Source address used by ExpressLRS Lua
};

enum class FrameType : uint8_t
{
  INVALID = 0x00,
  GPS = 0x02,
  VARIO = 0x07,
  BATTERY_SENSOR = 0x08,
  BARO_ALTITUDE = 0x09,
  HEARTBEAT = 0x0B,
  VIDEO_TRANSMITTER = 0x0F,
  OPENTX_SYNC = 0x10,
  LINK_STATISTICS = 0x14,
  RC_CHANNELS_PACKED = 0x16,
  ATTITUDE = 0x1E,
  FLIGHT_MODE = 0x21,
  DEVICE_PING = 0x28,
  DEVICE_INFO = 0x29,
  PARAMETER_SETTINGS_ENTRY = 0x2B,
  PARAMETER_READ = 0x2C,
  PARAMETER_WRITE = 0x2D,
  COMMAND = 0x32,
  RADIO_ID = 0x3A
};

class Frame
{
public:
  struct ParseResult
  {
    ValidationStatus validation_status;
    FrameType type;
    std::optional<std::vector<uint8_t>> payload;
  };

  // Common sync bytes values
  static constexpr uint8_t SYNC_BYTE =
    static_cast<uint8_t>(Address::CRSF_ADDRESS_FLIGHT_CONTROLLER);
  static constexpr uint8_t SYNC_BYTE_BROADCAST =
    static_cast<uint8_t>(Address::CRSF_ADDRESS_BROADCAST);
  static constexpr uint8_t SYNC_BYTE_EDGETX =
    static_cast<uint8_t>(Address::CRSF_ADDRESS_CRSF_TRANSMITTER);
  static constexpr size_t MIN_FRAME_LEN = 2;
  static constexpr size_t MAX_FRAME_LEN = 62;

  static inline ValidationStatus validate(const std::vector<uint8_t> & data);

  static std::vector<uint8_t> serialize(
    FrameType type, const std::vector<uint8_t> & payload,
    Address sync_byte = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER);
  static ParseResult parse_frame(const std::vector<uint8_t> & data);
  static uint8_t calculateCRC8(const std::span<const uint8_t> data);

private:
  static constexpr uint8_t CRC8_POLY = 0xD5;
  static bool isValidSync(uint8_t sync);
};
}  // namespace crsf
