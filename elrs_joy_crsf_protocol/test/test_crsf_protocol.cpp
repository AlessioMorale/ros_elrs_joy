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

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "elrs_joy_crsf_protocol/crsf/messages.hpp"
#include "elrs_joy_crsf_protocol/crsf/payload.hpp"
#include "elrs_joy_crsf_protocol/crsf/serialization.hpp"
namespace elrs_joy_crsf_protocol
{
TEST(SerializerTest, SerializeAndDeserializeBatterySensor)
{
  crsf::BatterySensorPayload payload = {12.3f, 4.6f, 789, 90};
  auto data = crsf::PayloadSerialization::serialize(payload);
  auto deserializedPayload =
    crsf::PayloadSerialization::deserializeMessage<crsf::BatterySensorPayload>(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_FLOAT_EQ(deserializedPayload->voltage, payload.voltage);
  EXPECT_FLOAT_EQ(deserializedPayload->current, payload.current);
  EXPECT_EQ(deserializedPayload->usedCapacity, payload.usedCapacity);
  EXPECT_EQ(deserializedPayload->batteryPercent, payload.batteryPercent);
}

TEST(SerializerTest, BatterySensorWireUnits)
{
  // Voltage and current go out big-endian in 0.1 V / 0.1 A, as EdgeTX expects
  crsf::BatterySensorPayload payload = {15.42f, 0.48f, 0, 0};
  auto data = crsf::PayloadSerialization::serialize(payload);
  ASSERT_EQ(data.size(), crsf::BatterySensorPayload::SIZE);
  EXPECT_EQ(data[0], 0x00);
  EXPECT_EQ(data[1], 154);
  EXPECT_EQ(data[2], 0x00);
  EXPECT_EQ(data[3], 5);
}

TEST(SerializerTest, SerializeAndDeserializeAttitude)
{
  crsf::AttitudePayload payload = {.pitch = 100, .roll = -200, .yaw = 300};
  auto data = crsf::PayloadSerialization::serialize(payload);
  auto deserializedPayload =
    crsf::PayloadSerialization::deserializeMessage<crsf::AttitudePayload>(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_EQ(deserializedPayload->pitch, payload.pitch);
  EXPECT_EQ(deserializedPayload->roll, payload.roll);
  EXPECT_EQ(deserializedPayload->yaw, payload.yaw);
}

TEST(SerializerTest, SerializeRCChannelPacked)
{
  crsf::RCChannelsPayload payload;
  for (size_t i = 0; i < 16; i++) {
    payload.channels[i] = 1500;
  }
  const std::vector<uint8_t> expected{0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0,
                                      0x81, 0x0f, 0x7c, 0xe0, 0x03, 0x1f, 0xf8, 0xc0,
                                      0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c};
  auto data = crsf::PayloadSerialization::serialize(payload);
  EXPECT_EQ(data, expected);
  auto deserializedPayload =
    crsf::PayloadSerialization::deserializeMessage<crsf::RCChannelsPayload>(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  for (size_t i = 0; i < 16; i++) {
    EXPECT_EQ(deserializedPayload->channels[i], 1500);
  }
}

TEST(SerializerTest, SerializeAndDeserializeHeartbeat)
{
  const std::vector<uint8_t> expected = {0x00, 0xc8};
  crsf::HeartbeatPayload payload = {
    .originDeviceAddress = crsf::Address::CRSF_ADDRESS_FLIGHT_CONTROLLER};
  auto data = crsf::PayloadSerialization::serialize(payload);
  auto deserializedPayload =
    crsf::PayloadSerialization::deserializeMessage<crsf::HeartbeatPayload>(expected);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_EQ(data, expected);
  EXPECT_EQ(
    deserializedPayload->originDeviceAddress, crsf::Address::CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

TEST(SerializerTest, SerializeAndDeserializeFlightMode)
{
  crsf::FlightModePayload payload = {.mode = "ANGLE"};
  auto data = crsf::PayloadSerialization::serialize(payload);
  auto deserializedPayload =
    crsf::PayloadSerialization::deserializeMessage<crsf::FlightModePayload>(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_EQ(deserializedPayload->mode, payload.mode);
}

TEST(SerializerTest, SerializeAndDeserializeDeviceInfo)
{
  crsf::DeviceInfoPayload payload{
    .ext_header =
      {.ext_src_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER,
       .ext_dest_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER},
    .device_name = "ELRS",
    .serial_number = 0x11223344u,
    .hardware_id = 0x55667788u,
    .firmware_id = 0x99AABBCCu,
    .parameters_total = 12,
    .parameter_version = 1};
  auto data = crsf::PayloadSerialization::serialize(payload);
  auto deserializedPayload =
    crsf::PayloadSerialization::deserializeMessage<crsf::DeviceInfoPayload>(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_EQ(deserializedPayload->ext_header.ext_src_addr, payload.ext_header.ext_src_addr);
  EXPECT_EQ(deserializedPayload->ext_header.ext_dest_addr, payload.ext_header.ext_dest_addr);
  EXPECT_EQ(deserializedPayload->device_name, payload.device_name);
  EXPECT_EQ(deserializedPayload->serial_number, payload.serial_number);
  EXPECT_EQ(deserializedPayload->hardware_id, payload.hardware_id);
  EXPECT_EQ(deserializedPayload->firmware_id, payload.firmware_id);
  EXPECT_EQ(deserializedPayload->parameters_total, payload.parameters_total);
  EXPECT_EQ(deserializedPayload->parameter_version, payload.parameter_version);
}

TEST(MessageTest, BatterySensorMessageRoundTrip)
{
  crsf::BatterySensorMessage message;
  message.payload = {12.3f, 4.6f, 789, 90};
  auto frame = message.to_frame();
  auto parsed = crsf::BatterySensorMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_FLOAT_EQ(parsed->payload.voltage, message.payload.voltage);
  EXPECT_FLOAT_EQ(parsed->payload.current, message.payload.current);
  EXPECT_EQ(parsed->payload.usedCapacity, message.payload.usedCapacity);
  EXPECT_EQ(parsed->payload.batteryPercent, message.payload.batteryPercent);
}

TEST(MessageTest, LinkStatisticsMessageRoundTrip)
{
  crsf::LinkStatisticsMessage message;
  message.payload = {
    .uplinkRssiAnt1 = 10,
    .uplinkRssiAnt2 = 11,
    .uplinkLinkQuality = 90,
    .uplinkSnr = -4,
    .activeAntenna = 1,
    .rfMode = 2,
    .uplinkTxPower = crsf::RFPower::POWER_100MW,
    .downlinkRssi = 12,
    .downlinkLinkQuality = 95,
    .downlinkSnr = 3};
  auto frame = message.to_frame();
  auto parsed = crsf::LinkStatisticsMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.uplinkRssiAnt1, message.payload.uplinkRssiAnt1);
  EXPECT_EQ(parsed->payload.uplinkRssiAnt2, message.payload.uplinkRssiAnt2);
  EXPECT_EQ(parsed->payload.uplinkLinkQuality, message.payload.uplinkLinkQuality);
  EXPECT_EQ(parsed->payload.uplinkSnr, message.payload.uplinkSnr);
  EXPECT_EQ(parsed->payload.activeAntenna, message.payload.activeAntenna);
  EXPECT_EQ(parsed->payload.rfMode, message.payload.rfMode);
  EXPECT_EQ(parsed->payload.uplinkTxPower, message.payload.uplinkTxPower);
  EXPECT_EQ(parsed->payload.downlinkRssi, message.payload.downlinkRssi);
  EXPECT_EQ(parsed->payload.downlinkLinkQuality, message.payload.downlinkLinkQuality);
  EXPECT_EQ(parsed->payload.downlinkSnr, message.payload.downlinkSnr);
}

TEST(MessageTest, RCChannelsMessageRoundTrip)
{
  crsf::RCChannelsMessage message;
  for (size_t i = 0; i < message.payload.channels.size(); ++i) {
    message.payload.channels[i] = 1500;
  }
  auto frame = message.to_frame();
  auto parsed = crsf::RCChannelsMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  for (size_t i = 0; i < message.payload.channels.size(); ++i) {
    EXPECT_EQ(parsed->payload.channels[i], message.payload.channels[i]);
  }
}

TEST(MessageTest, HeartbeatMessageRoundTrip)
{
  crsf::HeartbeatMessage message;
  message.payload.originDeviceAddress = crsf::Address::CRSF_ADDRESS_FLIGHT_CONTROLLER;
  auto frame = message.to_frame();
  auto parsed = crsf::HeartbeatMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.originDeviceAddress, message.payload.originDeviceAddress);
}

TEST(MessageTest, AttitudeMessageRoundTrip)
{
  crsf::AttitudeMessage message;
  message.payload = {.pitch = 100, .roll = -200, .yaw = 300};
  auto frame = message.to_frame();
  auto parsed = crsf::AttitudeMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.pitch, message.payload.pitch);
  EXPECT_EQ(parsed->payload.roll, message.payload.roll);
  EXPECT_EQ(parsed->payload.yaw, message.payload.yaw);
}

TEST(MessageTest, FlightModeMessageRoundTrip)
{
  crsf::FlightModeMessage message;
  message.payload.mode = "ANGLE";
  auto frame = message.to_frame();
  auto parsed = crsf::FlightModeMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.mode, message.payload.mode);
}

TEST(MessageTest, DevicePingMessageRoundTrip)
{
  crsf::DevicePingMessage message;
  message.payload.ext_header = {
    .ext_src_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER,
    .ext_dest_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER};
  auto frame = message.to_frame();
  auto parsed = crsf::DevicePingMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.ext_header.ext_src_addr, message.payload.ext_header.ext_src_addr);
  EXPECT_EQ(parsed->payload.ext_header.ext_dest_addr, message.payload.ext_header.ext_dest_addr);
}

TEST(MessageTest, DeviceInfoMessageRoundTrip)
{
  crsf::DeviceInfoMessage message;
  message.payload = {
    .ext_header =
      {.ext_src_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER,
       .ext_dest_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER},
    .device_name = "ELRS",
    .serial_number = 0x11223344u,
    .hardware_id = 0x55667788u,
    .firmware_id = 0x99AABBCCu,
    .parameters_total = 12,
    .parameter_version = 1};
  auto frame = message.to_frame();
  auto parsed = crsf::DeviceInfoMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.ext_header.ext_src_addr, message.payload.ext_header.ext_src_addr);
  EXPECT_EQ(parsed->payload.ext_header.ext_dest_addr, message.payload.ext_header.ext_dest_addr);
  EXPECT_EQ(parsed->payload.device_name, message.payload.device_name);
  EXPECT_EQ(parsed->payload.serial_number, message.payload.serial_number);
  EXPECT_EQ(parsed->payload.hardware_id, message.payload.hardware_id);
  EXPECT_EQ(parsed->payload.firmware_id, message.payload.firmware_id);
  EXPECT_EQ(parsed->payload.parameters_total, message.payload.parameters_total);
  EXPECT_EQ(parsed->payload.parameter_version, message.payload.parameter_version);
}

TEST(MessageTest, ParameterEntryMessageRoundTrip)
{
  crsf::ParameterEntryMessage message;
  message.payload.ext_header = {
    .ext_src_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER,
    .ext_dest_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER};
  message.payload.parameter_number = 2;
  message.payload.chunks_remaining = 1;
  message.payload.data = {0x00, 0x08, 'T', 'E', 'S', 'T', 0x00};
  auto frame = message.to_frame();
  auto parsed = crsf::ParameterEntryMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.ext_header.ext_src_addr, message.payload.ext_header.ext_src_addr);
  EXPECT_EQ(parsed->payload.ext_header.ext_dest_addr, message.payload.ext_header.ext_dest_addr);
  EXPECT_EQ(parsed->payload.parameter_number, message.payload.parameter_number);
  EXPECT_EQ(parsed->payload.chunks_remaining, message.payload.chunks_remaining);
  EXPECT_EQ(parsed->payload.data, message.payload.data);
}

TEST(MessageTest, ParameterReadMessageRoundTrip)
{
  crsf::ParameterReadMessage message;
  message.payload.ext_header = {
    .ext_src_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER,
    .ext_dest_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER};
  message.payload.parameter_number = 3;
  message.payload.chunk_number = 0;
  auto frame = message.to_frame();
  auto parsed = crsf::ParameterReadMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.ext_header.ext_src_addr, message.payload.ext_header.ext_src_addr);
  EXPECT_EQ(parsed->payload.ext_header.ext_dest_addr, message.payload.ext_header.ext_dest_addr);
  EXPECT_EQ(parsed->payload.parameter_number, message.payload.parameter_number);
  EXPECT_EQ(parsed->payload.chunk_number, message.payload.chunk_number);
}

TEST(MessageTest, ParameterWriteMessageRoundTrip)
{
  crsf::ParameterWriteMessage message;
  message.payload.ext_header = {
    .ext_src_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER,
    .ext_dest_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER};
  message.payload.parameter_number = 4;
  message.payload.data = {0x01, 0x02, 0x03};
  auto frame = message.to_frame();
  auto parsed = crsf::ParameterWriteMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.ext_header.ext_src_addr, message.payload.ext_header.ext_src_addr);
  EXPECT_EQ(parsed->payload.ext_header.ext_dest_addr, message.payload.ext_header.ext_dest_addr);
  EXPECT_EQ(parsed->payload.parameter_number, message.payload.parameter_number);
  EXPECT_EQ(parsed->payload.data, message.payload.data);
}

TEST(MessageTest, CommandMessageRoundTrip)
{
  crsf::CommandMessage message;
  message.payload.ext_header = {
    .ext_src_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER,
    .ext_dest_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER};
  message.payload.realm = crsf::CommandRealm::CRSF_COMMAND_SUBCMD_RX;
  message.payload.command = crsf::Command::CRSF_COMMAND_SUBCMD_RX_MODEL_SELECT_ID;
  message.payload.data = {0x36, 0x26};
  auto frame = message.to_frame();
  auto parsed = crsf::CommandMessage::from_frame(frame);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->payload.ext_header.ext_src_addr, message.payload.ext_header.ext_src_addr);
  EXPECT_EQ(parsed->payload.ext_header.ext_dest_addr, message.payload.ext_header.ext_dest_addr);
  EXPECT_EQ(parsed->payload.realm, message.payload.realm);
  EXPECT_EQ(parsed->payload.command, message.payload.command);
  EXPECT_EQ(parsed->payload.data, message.payload.data);
}
}  // namespace elrs_joy_crsf_protocol
