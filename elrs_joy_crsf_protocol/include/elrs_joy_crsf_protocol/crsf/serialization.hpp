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

#ifndef ELRS_JOY_CRSF_PROTOCOL__CRSF__SERIALIZATION_HPP_
#define ELRS_JOY_CRSF_PROTOCOL__CRSF__SERIALIZATION_HPP_
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "elrs_joy_crsf_protocol/crsf/message.hpp"
#include "elrs_joy_crsf_protocol/crsf/payload.hpp"

namespace elrs_joy_crsf_protocol::crsf
{

class PayloadSerialization
{
public:
  static std::vector<uint8_t> serialize(const BatterySensorPayload & payload);
  static std::vector<uint8_t> serialize(const HeartbeatPayload & payload);
  static std::vector<uint8_t> serialize(const AttitudePayload & payload);
  static std::vector<uint8_t> serialize(const LinkStatisticsPayload & payload);
  static std::vector<uint8_t> serialize(const RCChannelsPayload & payload);
  static std::vector<uint8_t> serialize(const FlightModePayload & payload);
  static std::vector<uint8_t> serialize(const DevicePingPayload & payload);
  static std::vector<uint8_t> serialize(const DeviceInfoPayload & payload);
  static std::vector<uint8_t> serialize(const ParameterEntryPayload & payload);
  static std::vector<uint8_t> serialize(const ParameterReadPayload & payload);
  static std::vector<uint8_t> serialize(const ParameterWritePayload & payload);
  static std::vector<uint8_t> serialize(const CommandPayload & payload);
  static std::vector<uint8_t> serialize(const OpenTxSyncPayload & payload);

  static uint8_t calculateCommandCRC8(
    MessageType type, Address destination, Address origin, uint8_t command_id,
    const std::vector<uint8_t> & payload);

  // Generic templated deserialization - specializations defined for each payload type
  template <typename PayloadType>
  static std::optional<PayloadType> deserializeMessage(const std::vector<uint8_t> & data);

protected:
  friend class Message;
  template <typename PayloadType>
  struct type
  {
  };

  template <typename PayloadType>
  static std::optional<PayloadType> deserialize_impl(
    const std::vector<uint8_t> & data, type<PayloadType>);

  static inline void packU8(uint8_t value, std::vector<uint8_t> & data) { data.push_back(value); }

  static inline void packU16(uint16_t value, std::vector<uint8_t> & data)
  {
    packU8(static_cast<uint8_t>((value >> 8) & 0xFF), data);
    packU8(static_cast<uint8_t>(value & 0xFF), data);
  }

  static inline uint16_t unpackU16(const uint8_t * data)
  {
    return static_cast<uint16_t>((data[0] << 8) | data[1]);
  }

  static inline void packU32(uint32_t value, std::vector<uint8_t> & data)
  {
    packU8(static_cast<uint8_t>((value >> 24) & 0xFF), data);
    packU8(static_cast<uint8_t>((value >> 16) & 0xFF), data);
    packU8(static_cast<uint8_t>((value >> 8) & 0xFF), data);
    packU8(static_cast<uint8_t>(value & 0xFF), data);
  }

  static inline uint32_t unpackU32(const uint8_t * data)
  {
    return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
  }

  static inline int8_t unpackI8(const uint8_t * data) { return static_cast<int8_t>(*data); }

  static inline void packI8(int8_t value, std::vector<uint8_t> & data)
  {
    packU8(static_cast<uint8_t>(value), data);
  }

  static inline int16_t unpackI16(const uint8_t * data)
  {
    return static_cast<int16_t>(unpackU16(data));
  }

  static inline void packI16(int16_t value, std::vector<uint8_t> & data)
  {
    packU16(static_cast<uint16_t>(value), data);
  }

  static inline void packExtHeader(const ExtendedHeader & header, std::vector<uint8_t> & data)
  {
    packU8(static_cast<uint8_t>(header.ext_dest_addr), data);
    packU8(static_cast<uint8_t>(header.ext_src_addr), data);
  }

  static ExtendedHeader unpackExtHeader(const uint8_t * data)
  {
    return {
      .ext_src_addr = static_cast<Address>(data[1]),
      .ext_dest_addr = static_cast<Address>(data[0])};
  }

  static inline void packNullTerminatedString(
    const std::string & value, std::vector<uint8_t> & data)
  {
    data.insert(data.end(), value.begin(), value.end());
    packU8(0, data);
  }

  static std::optional<std::pair<std::string, size_t>> unpackNullTerminatedString(
    const std::vector<uint8_t> & data, size_t offset);
  static inline void packBS24(int32_t value, std::vector<uint8_t> & data)
  {
    std::vector<uint8_t> result(3);
    bool sign = value < 0;
    value = std::abs(value);

    if (value >= (1 << 23)) {
      value = (1 << 23) - 1;
    }

    uint32_t packed = (sign ? 1U << 23 : 0) | (value & 0x7FFFFF);
    packU8((packed >> 16) & 0xFF, data);
    packU8((packed >> 8) & 0xFF, data);
    packU8(packed & 0xFF, data);
  }

  static inline int32_t unpackBS24(const uint8_t * data)
  {
    uint32_t value = static_cast<uint32_t>((data[0] << 16) | (data[1] << 8) | data[2]);
    bool sign = (value >> 23) & 1;
    int32_t magnitude = value & 0x7FFFFF;
    return sign ? -magnitude : magnitude;
  }
};
}  // namespace elrs_joy_crsf_protocol::crsf
#endif  // ELRS_JOY_CRSF_PROTOCOL__CRSF__SERIALIZATION_HPP_
