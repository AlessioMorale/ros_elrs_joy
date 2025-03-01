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

#include <cstdint>
#include <vector>

#include "elrs_joy_crsf_protocol/frame.hpp"

TEST(FrameTest, CalculateCRC8)
{
  std::vector<uint8_t> data = {0xc8, 0x18, 0x16, 0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07,
                               0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xe0, 0x03, 0x1f, 0xf8,
                               0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xad, 0x00};
  // calculate CRC, skip header bytes
  crsf::Frame frame{data};
  uint8_t crc = frame.calculate_crc();
  // the input string includes the CRC byte, thus the result must be 0
  EXPECT_EQ(crc, 0);  // Expected CRC value
}

// TEST(FrameTest, SerializeAndParse)
// {
//   std::vector<uint8_t> payload = {0x01, 0x02, 0x03};
//   auto frame = crsf::FrameSerializer::serialize(crsf::FrameType::BATTERY_SENSOR, payload);
//   auto result = crsf::FrameSerializer::deserialize(frame);
//   EXPECT_EQ(result.validation_status, crsf::ValidationStatus::OK);
//   ASSERT_TRUE(result.payload.has_value());
//   EXPECT_EQ(result.type, crsf::FrameType::BATTERY_SENSOR);
//   EXPECT_EQ(result.payload.value(), payload);
// }
