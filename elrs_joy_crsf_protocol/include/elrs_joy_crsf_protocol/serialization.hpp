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

#ifndef ELRS_JOY_CRSF_PROTOCOL__SERIALIZATION_HPP_
#define ELRS_JOY_CRSF_PROTOCOL__SERIALIZATION_HPP_
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <vector>

#include "elrs_joy_crsf_protocol/encoding.hpp"
#include "elrs_joy_crsf_protocol/payload.hpp"
namespace crsf
{

class Serialization
{
public:
  static std::vector<uint8_t> serialize(const BatterySensorPayload & payload);
  static std::vector<uint8_t> serialize(const HeartbeatPayload & payload);
  static std::vector<uint8_t> serialize(const LinkStatisticsPayload & payload);
  static std::vector<uint8_t> serialize(const RCChannelsPayload & payload);
  static std::vector<uint8_t> serialize(const CommandPayload & payload);

  static std::optional<BatterySensorPayload> deserializeBatterySensor(
    const std::vector<uint8_t> & data);
  static std::optional<HeartbeatPayload> deserializeHeartbeat(const std::vector<uint8_t> & data);
  static std::optional<LinkStatisticsPayload> deserializeLinkStatistics(
    const std::vector<uint8_t> & data);
  static std::optional<RCChannelsPayload> deserializeRCChannels(const std::vector<uint8_t> & data);
  static std::optional<CommandPayload> deserializeCommand(const std::vector<uint8_t> & data);

protected:
  friend class FrameSerializer;

  static inline void packExtHeader(const ExtendedHeader & header, Encoder & encoder)
  {
    encoder.writeU8(static_cast<uint8_t>(header.ext_dest_addr));
    encoder.writeU8(static_cast<uint8_t>(header.ext_src_addr));
  }

  static ExtendedHeader unpackExtHeader(Decoder & decoder)
  {
    return {
      .ext_src_addr = static_cast<Address>(decoder.readU8()),
      .ext_dest_addr = static_cast<Address>(decoder.readU8())};
  }
};
}  // namespace crsf
#endif  // ELRS_JOY_CRSF_PROTOCOL__SERIALIZATION_HPP_
