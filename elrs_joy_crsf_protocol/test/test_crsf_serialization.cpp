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

#include "elrs_joy_crsf_protocol/payload.hpp"
#include "elrs_joy_crsf_protocol/serialization.hpp"

TEST(SerializerTest, SerializeAndDeserializeBatterySensor)
{
  crsf::BatterySensorPayload payload = {12.3f, 4.56f, 789, 90};
  auto data = crsf::Serialization::serialize(payload);
  auto deserializedPayload = crsf::Serialization::deserializeBatterySensor(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_FLOAT_EQ(deserializedPayload->voltage, payload.voltage);
  EXPECT_FLOAT_EQ(deserializedPayload->current, payload.current);
  EXPECT_EQ(deserializedPayload->usedCapacity, payload.usedCapacity);
  EXPECT_EQ(deserializedPayload->batteryPercent, payload.batteryPercent);
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
  auto data = crsf::Serialization::serialize(payload);
  EXPECT_EQ(data, expected);
  auto deserializedPayload = crsf::Serialization::deserializeRCChannels(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  for (size_t i = 0; i < 16; i++) {
    EXPECT_EQ(deserializedPayload->channels[i], 1500);
  }
}

TEST(SerializerTest, SerializeAndDeserializeCommand)
{
  // \xc8\x08\x32\xee\xea\x10\x05\x36\x26\xa0
  const std::vector<uint8_t> expected = {0xc8, 0x08, 0x32, 0xEE, 0xEA,
                                         0x10, 0x05, 0x36, 0x26, 0xA0};
  crsf::CommandPayload payload{
    .ext_header =
      {.ext_src_addr = crsf::Address::CRSF_ADDRESS_RADIO_TRANSMITTER,
       .ext_dest_addr = crsf::Address::CRSF_ADDRESS_CRSF_TRANSMITTER},
    .realm = crsf::CommandRealm::CRSF_COMMAND_SUBCMD_RX,
    .command = crsf::Command::CRSF_COMMAND_SUBCMD_RX_MODEL_SELECT_ID,
    .data = {0x36, 0x26}};
  auto data = crsf::FrameSerializer::serialize(
    crsf::FrameType::COMMAND, crsf::Serialization::serialize(payload));
  auto deserializedPayload = crsf::Serialization::deserializeHeartbeat(data);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_EQ(data, expected);
}

TEST(SerializerTest, SerializeAndDeserializeHeartbeat)
{
  std::vector<uint8_t> expected = {0x00, 0xc8};
  crsf::HeartbeatPayload payload = {
    .originDeviceAddress = crsf::Address::CRSF_ADDRESS_FLIGHT_CONTROLLER};
  auto data = crsf::Serialization::serialize(payload);
  auto deserializedPayload = crsf::Serialization::deserializeHeartbeat(expected);
  ASSERT_TRUE(deserializedPayload.has_value());
  EXPECT_EQ(data, expected);
  EXPECT_EQ(
    deserializedPayload->originDeviceAddress, crsf::Address::CRSF_ADDRESS_FLIGHT_CONTROLLER);
}
