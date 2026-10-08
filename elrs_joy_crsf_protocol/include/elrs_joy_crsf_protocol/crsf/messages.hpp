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

#include <optional>

#include "elrs_joy_crsf_protocol/crsf/message.hpp"
#include "elrs_joy_crsf_protocol/crsf/payload.hpp"
#include "elrs_joy_crsf_protocol/crsf/serialization.hpp"

namespace elrs_joy_crsf_protocol::crsf
{

class BatterySensorMessage
{
public:
  BatterySensorPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::BATTERY_SENSOR, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<BatterySensorMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::BATTERY_SENSOR) {
      return std::nullopt;
    }

    auto parsed =
      PayloadSerialization::deserializeMessage<BatterySensorPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return BatterySensorMessage{.payload = *parsed};
  }
};

class HeartbeatMessage
{
public:
  HeartbeatPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data =
        Message::serialize(MessageType::HEARTBEAT, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<HeartbeatMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::HEARTBEAT) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<HeartbeatPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return HeartbeatMessage{.payload = *parsed};
  }
};

class AttitudeMessage
{
public:
  AttitudePayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data =
        Message::serialize(MessageType::ATTITUDE, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<AttitudeMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::ATTITUDE) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<AttitudePayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return AttitudeMessage{.payload = *parsed};
  }
};

class LinkStatisticsMessage
{
public:
  LinkStatisticsPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::LINK_STATISTICS, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<LinkStatisticsMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::LINK_STATISTICS) {
      return std::nullopt;
    }

    auto parsed =
      PayloadSerialization::deserializeMessage<LinkStatisticsPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return LinkStatisticsMessage{.payload = *parsed};
  }
};

class RCChannelsMessage
{
public:
  RCChannelsPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::RC_CHANNELS_PACKED, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<RCChannelsMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::RC_CHANNELS_PACKED) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<RCChannelsPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return RCChannelsMessage{.payload = *parsed};
  }
};

class FlightModeMessage
{
public:
  FlightModePayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::FLIGHT_MODE, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<FlightModeMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::FLIGHT_MODE) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<FlightModePayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return FlightModeMessage{.payload = *parsed};
  }
};

class DevicePingMessage
{
public:
  DevicePingPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::DEVICE_PING, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<DevicePingMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::DEVICE_PING) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<DevicePingPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return DevicePingMessage{.payload = *parsed};
  }
};

class DeviceInfoMessage
{
public:
  DeviceInfoPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::DEVICE_INFO, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<DeviceInfoMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::DEVICE_INFO) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<DeviceInfoPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return DeviceInfoMessage{.payload = *parsed};
  }
};

class ParameterEntryMessage
{
public:
  ParameterEntryPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::PARAMETER_SETTINGS_ENTRY, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<ParameterEntryMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::PARAMETER_SETTINGS_ENTRY) {
      return std::nullopt;
    }

    auto parsed =
      PayloadSerialization::deserializeMessage<ParameterEntryPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return ParameterEntryMessage{.payload = *parsed};
  }
};

class ParameterReadMessage
{
public:
  ParameterReadPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::PARAMETER_READ, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<ParameterReadMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::PARAMETER_READ) {
      return std::nullopt;
    }

    auto parsed =
      PayloadSerialization::deserializeMessage<ParameterReadPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return ParameterReadMessage{.payload = *parsed};
  }
};

class ParameterWriteMessage
{
public:
  ParameterWritePayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data = Message::serialize(
        MessageType::PARAMETER_WRITE, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<ParameterWriteMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::PARAMETER_WRITE) {
      return std::nullopt;
    }

    auto parsed =
      PayloadSerialization::deserializeMessage<ParameterWritePayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return ParameterWriteMessage{.payload = *parsed};
  }
};

class CommandMessage
{
public:
  CommandPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_FLIGHT_CONTROLLER) const
  {
    return {
      .data =
        Message::serialize(MessageType::COMMAND, PayloadSerialization::serialize(payload), sync)};
  }

  static std::optional<CommandMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::COMMAND) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<CommandPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return CommandMessage{.payload = *parsed};
  }
};

class OpenTxSyncMessage
{
public:
  OpenTxSyncPayload payload;

  Message::Frame to_frame(Address sync = Address::CRSF_ADDRESS_RADIO_TRANSMITTER) const
  {
    return {
      .data =
        Message::serialize(MessageType::RADIO_ID, PayloadSerialization::serialize(payload), sync)};
  }

  // Only RADIO_ID frames carrying the timing-correction sub-type are accepted
  static std::optional<OpenTxSyncMessage> from_frame(const Message::Frame & frame)
  {
    if (frame.get_type() != MessageType::RADIO_ID) {
      return std::nullopt;
    }

    auto parsed = PayloadSerialization::deserializeMessage<OpenTxSyncPayload>(frame.get_payload());
    if (!parsed) {
      return std::nullopt;
    }

    return OpenTxSyncMessage{.payload = *parsed};
  }
};

}  // namespace elrs_joy_crsf_protocol::crsf
