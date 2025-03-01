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
#include <functional>
#include <optional>
#include <span>  // NOLINT
#include <vector>

#include "elrs_joy_crsf_protocol/encoding.hpp"

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

class Frame : public Encoder, public Decoder
{
public:
  const size_t FRAME_SYNC_POSITION = 0;
  const size_t FRAME_LENGTH_POSITION = 1;
  const size_t FRAME_TYPE_POSITION = 2;
  const size_t FRAME_PAYLOAD_START_POSITION = 3;

  explicit Frame(std::vector<uint8_t> & frame_data) : crsf::Encoder(frame_data), Decoder(frame_data)
  {
  }

  uint8_t get_sync_byte() const
  {
    auto pos = FRAME_SYNC_POSITION;
    return readU8(pos);
  }

  Address get_sync() const { return static_cast<Address>(get_sync_byte()); }

  size_t get_length() const
  {
    auto pos = FRAME_LENGTH_POSITION;
    return static_cast<size_t>(readU8(pos));
  }

  FrameType get_type() const
  {
    auto pos = FRAME_TYPE_POSITION;
    return static_cast<FrameType>(readU8(pos));
  }

  void set_sync(Address sync_byte)
  {
    set_write_position(FRAME_SYNC_POSITION);
    writeU8(static_cast<uint8_t>(sync_byte));
  }

  void set_size(size_t size)
  {
    set_write_position(FRAME_LENGTH_POSITION);
    writeU8(static_cast<uint8_t>(size));
    data.get().resize(size + 2);  // [Payload size + type + crc] + sync + length
  }

  void set_type(FrameType type)
  {
    set_write_position(FRAME_TYPE_POSITION);
    writeU8(static_cast<uint8_t>(type));
  }

  std::span<uint8_t> get_payload() const
  {
    return {
      data.get().begin() + static_cast<int16_t>(FRAME_PAYLOAD_START_POSITION),
      data.get().end() - 1};
  };

  void set_payload(const std::span<const uint8_t> new_payload)
  {
    set_write_position(FRAME_PAYLOAD_START_POSITION);
    for (auto byte : new_payload) {
      writeU8(byte);
    }
  }

  uint8_t get_crc() const
  {
    auto pos = get_storage_size() - 1;  // [Payload size + type + crc] + sync + length
    return readU8(pos);
  }

  void update_crc()
  {
    set_write_position(get_length() + 2 - 1);  // [Payload size + type + crc] + sync + length
    uint8_t crc = calculate_crc();
    writeU8(crc);
  }

  uint8_t calculate_crc() const
  {
    return calculateCRC8({data.get().cbegin() + 2, data.get().cend() - 1});
  }

  bool check_crc() const { return calculate_crc() == get_crc(); }

  bool check_size() const { return get_storage_size() == get_length() + 2u; }

  size_t get_storage_size() const { return data.get().size(); }

  std::span<const uint8_t> get_data() const { return {data.get()}; }

private:
  static constexpr uint8_t CRC8_POLY = 0xD5;
  uint8_t calculateCRC8(const std::span<const uint8_t> framedata) const
  {
    uint8_t crc = 0;
    for (uint8_t value : framedata) {
      crc ^= value;
      for (int j = 0; j < 8; ++j) {
        if (crc & 0x80) {
          crc = static_cast<uint8_t>((crc << 1) ^ CRC8_POLY);
        } else {
          crc = static_cast<uint8_t>(crc << 1);
        }
      }
    }
    return crc;
  }
};

class FrameSerializer
{
public:
  struct SerializationResult
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

  static inline ValidationStatus validate(Frame & frame);

  static std::vector<uint8_t> serialize(
    FrameType type, const std::vector<uint8_t> & payload,
    Address sync_byte = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER);
  static SerializationResult deserialize(std::vector<uint8_t> & data);

private:
  static bool is_valid_sync(uint8_t sync);
};
}  // namespace crsf
