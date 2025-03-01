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

#include "elrs_joy_crsf_protocol/serialization.hpp"

#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "elrs_joy_crsf_protocol/encoding.hpp"

namespace crsf
{

std::vector<uint8_t> Serialization::serialize(const BatterySensorPayload & payload)
{
  std::vector<uint8_t> data;
  Encoder encoder{data};
  data.reserve(BatterySensorPayload::SIZE);

  int16_t voltage = static_cast<int16_t>(payload.voltage * 100);
  int16_t current = static_cast<int16_t>(payload.current * 100);
  encoder.writeI16(voltage);
  encoder.writeI16(current);
  encoder.writeBS24(payload.usedCapacity);
  encoder.writeU8(payload.batteryPercent);
  return data;
}

std::vector<uint8_t> Serialization::serialize(const HeartbeatPayload & payload)
{
  std::vector<uint8_t> data;
  Encoder encoder{data};
  encoder.writeU16(static_cast<uint16_t>(payload.originDeviceAddress));
  return data;
}

std::vector<uint8_t> Serialization::serialize(const LinkStatisticsPayload & payload)
{
  std::vector<uint8_t> data;
  Encoder encoder{data};
  encoder.writeU8(payload.uplinkRssiAnt1);
  encoder.writeU8(payload.uplinkRssiAnt2);
  encoder.writeU8(payload.uplinkLinkQuality);
  encoder.writeI8(payload.uplinkSnr);
  encoder.writeU8(payload.activeAntenna);
  encoder.writeU8(payload.rfMode);
  encoder.writeU8(static_cast<uint8_t>(payload.uplinkTxPower));
  encoder.writeU8(payload.downlinkRssi);
  encoder.writeU8(payload.downlinkLinkQuality);
  encoder.writeI8(payload.downlinkSnr);
  return data;
}

std::vector<uint8_t> Serialization::serialize(const RCChannelsPayload & payload)
{
  std::vector<uint8_t> data(RCChannelsPayload::SIZE);
  uint32_t bitBuffer = 0;
  int bitCount = 0;
  size_t byteIndex = 0;
  auto channels = std::array(payload.channels);
  for (size_t i = 0; i < channels.size(); ++i) {
    channels[i] = RCChannelsPayload::us_to_ticks(channels[i]);
  }

  for (uint16_t channel : channels) {
    channel &= 0x7FF;  // Ensure 11-bit value
    bitBuffer |= static_cast<uint32_t>(channel) << bitCount;
    bitCount += 11;

    while (bitCount >= 8 && byteIndex < data.size()) {
      data[byteIndex++] = bitBuffer & 0xFF;
      bitBuffer >>= 8;
      bitCount -= 8;
    }
  }

  if (bitCount > 0 && byteIndex < data.size()) {
    data[byteIndex] = bitBuffer & 0xFF;
  }

  return data;
}

std::vector<uint8_t> Serialization::serialize(const CommandPayload & payload)
{
  std::vector<uint8_t> data;
  Encoder encoder{data};
  data.reserve(2 + payload.data.size());  // Realm + Command + Data
  packExtHeader(payload.ext_header, encoder);
  encoder.writeU8(static_cast<uint8_t>(payload.realm));
  encoder.writeU8(static_cast<uint8_t>(payload.command));
  data.insert(data.end(), payload.data.begin(), payload.data.end());
  return data;
}

std::optional<BatterySensorPayload> Serialization::deserializeBatterySensor(
  const std::vector<uint8_t> & data)
{
  if (data.size() < BatterySensorPayload::SIZE) {
    return std::nullopt;
  }
  Decoder decoder{data};
  BatterySensorPayload payload;
  int16_t voltage = decoder.readI16();
  int16_t current = decoder.readI16();

  payload.voltage = static_cast<float>(voltage) / 100.0f;
  payload.current = static_cast<float>(current) / 100.0f;
  payload.usedCapacity = decoder.readBS24();
  payload.batteryPercent = data[7];

  return payload;
}

std::optional<HeartbeatPayload> Serialization::deserializeHeartbeat(
  const std::vector<uint8_t> & data)
{
  if (data.size() < HeartbeatPayload::SIZE) {
    return std::nullopt;
  }
  Decoder decoder{data};
  HeartbeatPayload payload;
  auto tmp = decoder.readU16();
  payload.originDeviceAddress = static_cast<Address>(tmp);
  return payload;
}

std::optional<LinkStatisticsPayload> Serialization::deserializeLinkStatistics(
  const std::vector<uint8_t> & data)
{
  if (data.size() < LinkStatisticsPayload::SIZE) {
    return std::nullopt;
  }
  Decoder decoder{data};
  LinkStatisticsPayload payload;
  payload.uplinkRssiAnt1 = decoder.readU8();
  payload.uplinkRssiAnt2 = decoder.readU8();
  payload.uplinkLinkQuality = decoder.readU8();
  payload.uplinkSnr = decoder.readI8();
  payload.activeAntenna = decoder.readU8();
  payload.rfMode = decoder.readU8();
  payload.uplinkTxPower = static_cast<RFPower>(decoder.readU8());
  payload.downlinkRssi = decoder.readU8();
  payload.downlinkLinkQuality = decoder.readU8();
  payload.downlinkSnr = decoder.readI8();
  return payload;
}

std::optional<RCChannelsPayload> Serialization::deserializeRCChannels(
  const std::vector<uint8_t> & data)
{
  if (data.size() < RCChannelsPayload::SIZE) {
    return std::nullopt;
  }

  RCChannelsPayload payload;
  uint32_t bitBuffer = 0;
  int bitCount = 0;
  size_t byteIndex = 0;

  for (size_t i = 0; i < payload.channels.size(); ++i) {
    while (bitCount < 11 && byteIndex < data.size()) {
      bitBuffer |= static_cast<uint32_t>(data[byteIndex++]) << bitCount;
      bitCount += 8;
    }

    payload.channels[i] = bitBuffer & 0x7FF;
    bitBuffer >>= 11;
    bitCount -= 11;
  }
  for (size_t i = 0; i < payload.channels.size(); ++i) {
    payload.channels[i] = RCChannelsPayload::ticks_to_us(payload.channels[i]);
  }
  return payload;
}

std::optional<CommandPayload> Serialization::deserializeCommand(const std::vector<uint8_t> & data)
{
  if (data.size() < CommandPayload::BASE_SIZE) {
    return std::nullopt;
  }
  Decoder decoder{data};
  CommandPayload payload;
  payload.ext_header = Serialization::unpackExtHeader(decoder);
  payload.realm = static_cast<CommandRealm>(decoder.readU8());
  payload.command = static_cast<Command>(decoder.readU8());
  payload.data.assign(data.begin() + 4, data.end());
  return payload;
}

}  // namespace crsf
