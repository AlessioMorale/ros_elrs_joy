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

#include "elrs_joy_crsf_protocol/crsf/serialization.hpp"

#include <linux/limits.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace elrs_joy_crsf_protocol::crsf
{

std::vector<uint8_t> PayloadSerialization::serialize(const BatterySensorPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(BatterySensorPayload::SIZE);

  // 0.1 V / 0.1 A units, as sent by Betaflight/iNav and decoded by EdgeTX
  // (the TBS spec text says 10 uV / 10 uA, which no implementation uses)
  const auto voltage = static_cast<uint16_t>(
    std::clamp<long>(std::lround(payload.voltage * 10.0F), 0L, UINT16_MAX));
  const auto current = static_cast<int16_t>(
    std::clamp<long>(std::lround(payload.current * 10.0F), INT16_MIN, INT16_MAX));

  packU16(voltage, data);
  packI16(current, data);

  packBS24(payload.usedCapacity, data);

  packU8(payload.batteryPercent, data);
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const HeartbeatPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(HeartbeatPayload::SIZE);
  packU16(static_cast<uint16_t>(payload.originDeviceAddress), data);
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const AttitudePayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(AttitudePayload::SIZE);
  packI16(payload.pitch, data);
  packI16(payload.roll, data);
  packI16(payload.yaw, data);
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const FlightModePayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(payload.mode.size() + 1);
  packNullTerminatedString(payload.mode, data);
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const DevicePingPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(DevicePingPayload::SIZE);
  packExtHeader(payload.ext_header, data);
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const DeviceInfoPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(payload.device_name.size() + DeviceInfoPayload::BASE_SIZE + 1);
  packExtHeader(payload.ext_header, data);
  packNullTerminatedString(payload.device_name, data);
  packU32(payload.serial_number, data);
  packU32(payload.hardware_id, data);
  packU32(payload.firmware_id, data);
  packU8(payload.parameters_total, data);
  packU8(payload.parameter_version, data);
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const ParameterEntryPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(ParameterEntryPayload::BASE_SIZE + payload.data.size());
  packExtHeader(payload.ext_header, data);
  packU8(payload.parameter_number, data);
  packU8(payload.chunks_remaining, data);
  data.insert(data.end(), payload.data.begin(), payload.data.end());
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const ParameterReadPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(ParameterReadPayload::SIZE);
  packExtHeader(payload.ext_header, data);
  packU8(payload.parameter_number, data);
  packU8(payload.chunk_number, data);
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const ParameterWritePayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(ParameterWritePayload::BASE_SIZE + payload.data.size());
  packExtHeader(payload.ext_header, data);
  packU8(payload.parameter_number, data);
  data.insert(data.end(), payload.data.begin(), payload.data.end());
  return data;
}

std::vector<uint8_t> PayloadSerialization::serialize(const LinkStatisticsPayload & payload)
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

std::vector<uint8_t> PayloadSerialization::serialize(const RCChannelsPayload & payload)
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

std::vector<uint8_t> PayloadSerialization::serialize(const CommandPayload & payload)
{
  std::vector<uint8_t> data;
  data.reserve(CommandPayload::BASE_SIZE + payload.data.size());
  packExtHeader(payload.ext_header, data);
  packU8(static_cast<uint8_t>(payload.realm), data);
  packU8(static_cast<uint8_t>(payload.command), data);
  data.insert(data.end(), payload.data.begin(), payload.data.end());
  uint8_t command_crc = calculateCommandCRC8(
    MessageType::COMMAND, payload.ext_header.ext_dest_addr, payload.ext_header.ext_src_addr,
    static_cast<uint8_t>(payload.realm),
    std::vector<uint8_t>(data.begin() + ExtendedHeader::SIZE + 1, data.end()));
  packU8(command_crc, data);
  return data;
}

template <>
std::optional<BatterySensorPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<BatterySensorPayload>)
{
  if (data.size() < BatterySensorPayload::SIZE) {
    return std::nullopt;
  }

  BatterySensorPayload payload;
  uint16_t voltage = PayloadSerialization::unpackU16(&data[0]);
  int16_t current = PayloadSerialization::unpackI16(&data[2]);

  payload.voltage = static_cast<float>(voltage) / 10.0f;
  payload.current = static_cast<float>(current) / 10.0f;
  payload.usedCapacity = PayloadSerialization::unpackBS24(&data[4]);
  payload.batteryPercent = data[7];

  return payload;
}

template <>
std::optional<HeartbeatPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<HeartbeatPayload>)
{
  if (data.size() < HeartbeatPayload::SIZE) {
    return std::nullopt;
  }

  HeartbeatPayload payload;
  auto tmp = PayloadSerialization::unpackU16(&data[0]);
  payload.originDeviceAddress = static_cast<Address>(tmp);
  return payload;
}

template <>
std::optional<AttitudePayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<AttitudePayload>)
{
  if (data.size() < AttitudePayload::SIZE) {
    return std::nullopt;
  }

  AttitudePayload payload;
  payload.pitch = PayloadSerialization::unpackI16(&data[0]);
  payload.roll = PayloadSerialization::unpackI16(&data[2]);
  payload.yaw = PayloadSerialization::unpackI16(&data[4]);
  return payload;
}

template <>
std::optional<LinkStatisticsPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<LinkStatisticsPayload>)
{
  if (data.size() < LinkStatisticsPayload::SIZE) {
    return std::nullopt;
  }

  LinkStatisticsPayload payload;
  payload.uplinkRssiAnt1 = data[0];
  payload.uplinkRssiAnt2 = data[1];
  payload.uplinkLinkQuality = data[2];
  payload.uplinkSnr = PayloadSerialization::unpackI8(&data[3]);
  payload.activeAntenna = data[4];
  payload.rfMode = data[5];
  payload.uplinkTxPower = static_cast<RFPower>(data[6]);
  payload.downlinkRssi = data[7];
  payload.downlinkLinkQuality = data[8];
  payload.downlinkSnr = PayloadSerialization::unpackI8(&data[9]);
  return payload;
}

template <>
std::optional<RCChannelsPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<RCChannelsPayload>)
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

template <>
std::optional<FlightModePayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<FlightModePayload>)
{
  auto parsed = PayloadSerialization::unpackNullTerminatedString(data, 0);
  if (!parsed) {
    return std::nullopt;
  }

  FlightModePayload payload;
  payload.mode = parsed->first;
  return payload;
}

template <>
std::optional<DevicePingPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<DevicePingPayload>)
{
  if (data.size() < DevicePingPayload::SIZE) {
    return std::nullopt;
  }

  DevicePingPayload payload;
  payload.ext_header = PayloadSerialization::unpackExtHeader(&data[0]);
  return payload;
}

template <>
std::optional<DeviceInfoPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<DeviceInfoPayload>)
{
  if (data.size() < DeviceInfoPayload::BASE_SIZE + 1) {
    return std::nullopt;
  }

  DeviceInfoPayload payload;
  payload.ext_header = PayloadSerialization::unpackExtHeader(&data[0]);
  size_t offset = ExtendedHeader::SIZE;
  auto parsed = PayloadSerialization::unpackNullTerminatedString(data, offset);
  if (!parsed) {
    return std::nullopt;
  }

  payload.device_name = parsed->first;
  offset = parsed->second;
  if (data.size() < offset + 4 + 4 + 4 + 1 + 1) {
    return std::nullopt;
  }

  payload.serial_number = PayloadSerialization::unpackU32(&data[offset]);
  offset += 4;
  payload.hardware_id = PayloadSerialization::unpackU32(&data[offset]);
  offset += 4;
  payload.firmware_id = PayloadSerialization::unpackU32(&data[offset]);
  offset += 4;
  payload.parameters_total = data[offset++];
  payload.parameter_version = data[offset++];
  return payload;
}

template <>
std::optional<ParameterEntryPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<ParameterEntryPayload>)
{
  if (data.size() < ParameterEntryPayload::BASE_SIZE) {
    return std::nullopt;
  }

  ParameterEntryPayload payload;
  payload.ext_header = PayloadSerialization::unpackExtHeader(&data[0]);
  payload.parameter_number = data[ExtendedHeader::SIZE];
  payload.chunks_remaining = data[ExtendedHeader::SIZE + 1];
  payload.data.assign(data.begin() + ExtendedHeader::SIZE + 2, data.end());
  return payload;
}

template <>
std::optional<ParameterReadPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<ParameterReadPayload>)
{
  if (data.size() < ParameterReadPayload::SIZE) {
    return std::nullopt;
  }

  ParameterReadPayload payload;
  payload.ext_header = PayloadSerialization::unpackExtHeader(&data[0]);
  payload.parameter_number = data[ExtendedHeader::SIZE];
  payload.chunk_number = data[ExtendedHeader::SIZE + 1];
  return payload;
}

template <>
std::optional<ParameterWritePayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<ParameterWritePayload>)
{
  if (data.size() < ParameterWritePayload::BASE_SIZE) {
    return std::nullopt;
  }

  ParameterWritePayload payload;
  payload.ext_header = PayloadSerialization::unpackExtHeader(&data[0]);
  payload.parameter_number = data[ExtendedHeader::SIZE];
  payload.data.assign(data.begin() + ExtendedHeader::SIZE + 1, data.end());
  return payload;
}

template <>
std::optional<CommandPayload> PayloadSerialization::deserialize_impl(
  const std::vector<uint8_t> & data, type<CommandPayload>)
{
  if (data.size() < CommandPayload::BASE_SIZE) {
    return std::nullopt;
  }

  CommandPayload payload;
  payload.ext_header = PayloadSerialization::unpackExtHeader(&data[0]);
  payload.realm = static_cast<CommandRealm>(data[2]);
  payload.command = static_cast<Command>(data[3]);
  payload.data.assign(data.begin() + 4, data.end() - 1);
  payload.command_crc = data.back();

  std::vector<uint8_t> crc_payload(data.begin() + ExtendedHeader::SIZE + 1, data.end() - 1);
  auto expected_crc = PayloadSerialization::calculateCommandCRC8(
    MessageType::COMMAND, payload.ext_header.ext_dest_addr, payload.ext_header.ext_src_addr,
    static_cast<uint8_t>(payload.realm), crc_payload);
  if (expected_crc != payload.command_crc) {
    return std::nullopt;
  }

  return payload;
}

// Template specializations for deserializeMessage
template <>
std::optional<BatterySensorPayload> PayloadSerialization::deserializeMessage<BatterySensorPayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<BatterySensorPayload>{});
}

template <>
std::optional<HeartbeatPayload> PayloadSerialization::deserializeMessage<HeartbeatPayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<HeartbeatPayload>{});
}

template <>
std::optional<AttitudePayload> PayloadSerialization::deserializeMessage<AttitudePayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<AttitudePayload>{});
}

template <>
std::optional<LinkStatisticsPayload>
PayloadSerialization::deserializeMessage<LinkStatisticsPayload>(const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<LinkStatisticsPayload>{});
}

template <>
std::optional<RCChannelsPayload> PayloadSerialization::deserializeMessage<RCChannelsPayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<RCChannelsPayload>{});
}

template <>
std::optional<FlightModePayload> PayloadSerialization::deserializeMessage<FlightModePayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<FlightModePayload>{});
}

template <>
std::optional<DevicePingPayload> PayloadSerialization::deserializeMessage<DevicePingPayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<DevicePingPayload>{});
}

template <>
std::optional<DeviceInfoPayload> PayloadSerialization::deserializeMessage<DeviceInfoPayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<DeviceInfoPayload>{});
}

template <>
std::optional<ParameterEntryPayload>
PayloadSerialization::deserializeMessage<ParameterEntryPayload>(const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<ParameterEntryPayload>{});
}

template <>
std::optional<ParameterReadPayload> PayloadSerialization::deserializeMessage<ParameterReadPayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<ParameterReadPayload>{});
}

template <>
std::optional<ParameterWritePayload>
PayloadSerialization::deserializeMessage<ParameterWritePayload>(const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<ParameterWritePayload>{});
}

template <>
std::optional<CommandPayload> PayloadSerialization::deserializeMessage<CommandPayload>(
  const std::vector<uint8_t> & data)
{
  return deserialize_impl(data, type<CommandPayload>{});
}

uint8_t PayloadSerialization::calculateCommandCRC8(
  MessageType type, Address destination, Address origin, uint8_t command_id,
  const std::vector<uint8_t> & payload)
{
  static constexpr uint8_t CRC8_POLY = 0xBA;
  std::vector<uint8_t> data;
  data.reserve(4 + payload.size());
  data.push_back(static_cast<uint8_t>(type));
  data.push_back(static_cast<uint8_t>(destination));
  data.push_back(static_cast<uint8_t>(origin));
  data.push_back(command_id);
  data.insert(data.end(), payload.begin(), payload.end());

  uint8_t crc = 0;
  for (uint8_t value : data) {
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

std::optional<std::pair<std::string, size_t>> PayloadSerialization::unpackNullTerminatedString(
  const std::vector<uint8_t> & data, size_t offset)
{
  if (offset >= data.size()) {
    return std::nullopt;
  }

  auto end_it = std::find(data.begin() + static_cast<std::ptrdiff_t>(offset), data.end(), 0);
  if (end_it == data.end()) {
    return std::nullopt;
  }

  std::string value(data.begin() + static_cast<std::ptrdiff_t>(offset), end_it);
  size_t next_offset = static_cast<size_t>(std::distance(data.begin(), end_it)) + 1;
  return std::make_pair(value, next_offset);
}

}  // namespace elrs_joy_crsf_protocol::crsf
