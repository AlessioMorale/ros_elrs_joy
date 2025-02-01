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

namespace crsf
{

std::vector<uint8_t> Serialization::serialize(const BatterySensorPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(BatterySensorPayload::SIZE);

  int16_t voltage = static_cast<int16_t>(payload.voltage * 100);
  int16_t current = static_cast<int16_t>(payload.current * 100);

  packI16(voltage, data);
  packI16(current, data);

  packBS24(payload.usedCapacity, data);

  packU8(payload.batteryPercent, data);
  return data;
}

std::vector<uint8_t> Serialization::serialize(const HeartbeatPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(HeartbeatPayload::SIZE);
  packU16(static_cast<uint16_t>(payload.originDeviceAddress), data);
  return data;
}

std::vector<uint8_t> Serialization::serialize(const LinkStatisticsPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(LinkStatisticsPayload::SIZE);
  packU8(payload.uplinkRssiAnt1, data);
  packU8(payload.uplinkRssiAnt2, data);
  packU8(payload.uplinkLinkQuality, data);
  packI8(payload.uplinkSnr, data);
  packU8(payload.activeAntenna, data);
  packU8(payload.rfMode, data);
  packU8(static_cast<uint8_t>(payload.uplinkTxPower), data);
  packU8(payload.downlinkRssi, data);
  packU8(payload.downlinkLinkQuality, data);
  packI8(payload.downlinkSnr, data);
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
  data.reserve(2 + payload.data.size());  // Realm + Command + Data
  packExtHeader(payload.ext_header, data);
  packU8(static_cast<uint8_t>(payload.realm), data);
  packU8(static_cast<uint8_t>(payload.command), data);
  data.insert(data.end(), payload.data.begin(), payload.data.end());
  return data;
}

std::optional<BatterySensorPayload> Serialization::deserializeBatterySensor(
  const std::vector<uint8_t> & data)
{
  if (data.size() < BatterySensorPayload::SIZE) {
    return std::nullopt;
  }

  BatterySensorPayload payload;
  int16_t voltage = unpackI16(&data[0]);
  int16_t current = unpackI16(&data[2]);

  payload.voltage = static_cast<float>(voltage) / 100.0f;
  payload.current = static_cast<float>(current) / 100.0f;
  payload.usedCapacity = unpackBS24(&data[4]);
  payload.batteryPercent = data[7];

  return payload;
}

std::optional<HeartbeatPayload> Serialization::deserializeHeartbeat(
  const std::vector<uint8_t> & data)
{
  if (data.size() < HeartbeatPayload::SIZE) {
    return std::nullopt;
  }

  HeartbeatPayload payload;
  auto tmp = unpackU16(&data[0]);
  payload.originDeviceAddress = static_cast<Address>(tmp);
  return payload;
}

std::optional<LinkStatisticsPayload> Serialization::deserializeLinkStatistics(
  const std::vector<uint8_t> & data)
{
  if (data.size() < LinkStatisticsPayload::SIZE) {
    return std::nullopt;
  }

  LinkStatisticsPayload payload;
  payload.uplinkRssiAnt1 = data[0];
  payload.uplinkRssiAnt2 = data[1];
  payload.uplinkLinkQuality = data[2];
  payload.uplinkSnr = unpackI8(&data[3]);
  payload.activeAntenna = data[4];
  payload.rfMode = data[5];
  payload.uplinkTxPower = static_cast<RFPower>(data[6]);
  payload.downlinkRssi = data[7];
  payload.downlinkLinkQuality = data[8];
  payload.downlinkSnr = unpackI8(&data[9]);
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

  CommandPayload payload;
  payload.ext_header = Serialization::unpackExtHeader(&data[0]);
  payload.realm = static_cast<CommandRealm>(data[2]);
  payload.command = static_cast<Command>(data[3]);
  payload.data.assign(data.begin() + 4, data.end());
  return payload;
}

}  // namespace crsf
