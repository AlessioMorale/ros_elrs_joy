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

#ifndef ELRS_JOY_CRSF_PROTOCOL__TEST_FRAME_PARSING_HPP_
#define ELRS_JOY_CRSF_PROTOCOL__TEST_FRAME_PARSING_HPP_

#include <gtest/gtest.h>

#include "elrs_joy_crsf_protocol/frame.hpp"
#include "elrs_joy_crsf_protocol/frame_parsing.hpp"

namespace crsf
{

class FramePacketTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    frame_packet.set_callback([this](const Frame & frame) {
      data = {frame.get_data().begin(), frame.get_data().end()};
      got_frame = true;
    });
  }
  FramePacket frame_packet;
  std::vector<uint8_t> data;
  Frame last_frame{data};
  bool got_frame = false;
};

TEST_F(FramePacketTest, InitialState)
{
  auto stats = frame_packet.get_statistics();
  EXPECT_EQ(stats.total_bytes_processed, 0);
  EXPECT_EQ(stats.frames_decoded, 0);
  EXPECT_EQ(stats.sync_errors, 0);
  EXPECT_EQ(stats.length_errors, 0);
  EXPECT_EQ(stats.crc_errors, 0);
}

TEST_F(FramePacketTest, ProcessValidFrame)
{
  // \xc8\x18\x16\xe0\x03\x1f\xf8\xc0\x07\x3e\xf0\x81\x0f\x7c\xe0\x03\x1f\xf8\xc0\x07\x3e\xf0\x81\x0f\x7c\xad
  //   const std::vector<uint8_t> valid_frame {0xc8, 0x18, 0x16, 0xe0, 0x03, 0x1f, 0xf8, 0xc0,
  //     0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xe0, 0x03,
  //     0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f,
  //     0x7c, 0xad};

  const std::vector<uint8_t> valid_frame{0xc8, 0x06, 0x2d, 0xee, 0xef, 0x11, 0x01, 0xa5};
  for (auto byte : valid_frame) {
    frame_packet.process_byte(byte);
  }
  EXPECT_TRUE(got_frame);
  EXPECT_EQ(frame_packet.get_statistics().total_bytes_processed, valid_frame.size());
  EXPECT_EQ(frame_packet.get_statistics().frames_decoded, 1);
  EXPECT_EQ(frame_packet.get_statistics().sync_errors, 0);
  EXPECT_EQ(frame_packet.get_statistics().length_errors, 0);
  EXPECT_EQ(frame_packet.get_statistics().crc_errors, 0);
  EXPECT_EQ(last_frame.get_storage_size(), valid_frame.size());
  EXPECT_EQ(last_frame.get_type(), FrameType::PARAMETER_WRITE);
}

TEST_F(FramePacketTest, ProcessInvalidSync)
{
  std::vector<uint8_t> invalid_sync_frame = {0xFF, 0x02, 0x01, 0x00, 0x03};  // Invalid sync byte
  for (auto byte : invalid_sync_frame) {
    frame_packet.process_byte(byte);
  }

  EXPECT_EQ(frame_packet.get_statistics().total_bytes_processed, invalid_sync_frame.size());
  EXPECT_EQ(frame_packet.get_statistics().frames_decoded, 0);
  EXPECT_GE(frame_packet.get_statistics().sync_errors, 0);
  EXPECT_EQ(frame_packet.get_statistics().length_errors, 0);
  EXPECT_EQ(frame_packet.get_statistics().crc_errors, 0);
  EXPECT_FALSE(got_frame);
}

TEST_F(FramePacketTest, ProcessInvalidLength)
{
  std::vector<uint8_t> invalid_length_frame = {
    0xC8, 0xFF, 0x01, 0x00, 0x03};  // Invalid length byte
  for (auto byte : invalid_length_frame) {
    frame_packet.process_byte(byte);
  }

  EXPECT_EQ(frame_packet.get_statistics().total_bytes_processed, invalid_length_frame.size());
  EXPECT_EQ(frame_packet.get_statistics().frames_decoded, 0);
  EXPECT_EQ(frame_packet.get_statistics().length_errors, 1);
  EXPECT_EQ(frame_packet.get_statistics().crc_errors, 0);
  EXPECT_FALSE(got_frame);
}

TEST_F(FramePacketTest, ProcessInvalidCRC)
{
  std::vector<uint8_t> invalid_crc_frame = {0xC8, 0x02, 0x01, 0xFF};  // Invalid CRC byte
  for (auto byte : invalid_crc_frame) {
    frame_packet.process_byte(byte);
  }

  EXPECT_EQ(frame_packet.get_statistics().total_bytes_processed, invalid_crc_frame.size());
  EXPECT_EQ(frame_packet.get_statistics().frames_decoded, 0);
  EXPECT_EQ(frame_packet.get_statistics().sync_errors, 0);
  EXPECT_EQ(frame_packet.get_statistics().length_errors, 0);
  EXPECT_EQ(frame_packet.get_statistics().crc_errors, 1);
  EXPECT_FALSE(got_frame);
}

}  // namespace crsf

#endif  // ELRS_JOY_CRSF_PROTOCOL__TEST_FRAME_PARSING_HPP_
