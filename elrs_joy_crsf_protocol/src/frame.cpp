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

#include "elrs_joy_crsf_protocol/frame.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>  // NOLINT
#include <vector>

#include "elrs_joy_crsf_protocol/serialization.hpp"

namespace crsf
{

bool Frame::isValidSync(uint8_t sync)
{
  return sync == SYNC_BYTE || sync == SYNC_BYTE_EDGETX || sync == SYNC_BYTE_BROADCAST;
}

std::vector<uint8_t> Frame::serialize(
  FrameType type, const std::vector<uint8_t> & payload, Address sync_byte)
{
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 4);  // sync + length + type + payload + crc

  Serialization::packU8(static_cast<uint8_t>(sync_byte), frame);
  Serialization::packU8(static_cast<uint8_t>(payload.size() + 2), frame);  // +2 for type and CRC
  Serialization::packU8(static_cast<uint8_t>(type), frame);
  frame.insert(frame.end(), payload.begin(), payload.end());

  // Calculate CRC from type to end of payload
  uint8_t crc = calculateCRC8({frame.data() + 2, static_cast<size_t>(frame.size() - 2)});
  Serialization::packU8(crc, frame);

  return frame;
}

inline ValidationStatus Frame::validate(const std::vector<uint8_t> & data)
{
  if (data.size() < MIN_FRAME_LEN + 2) {
    return ValidationStatus::INVALID_NOT_ENOUGH_DATA;
  }

  auto sync = data[0];
  auto length = data[1];
  [[maybe_unused]] auto type = static_cast<FrameType>(data[2]);

  if (!isValidSync(sync)) {
    return ValidationStatus::UNSUPPORTED_SYNC;
  }

  if (data.size() < length + 2u) {  // +2 for sync and length bytes
    return ValidationStatus::INVALID_NOT_ENOUGH_DATA;
  }

  uint8_t calculatedCRC = calculateCRC8({data.data() + 2, static_cast<size_t>(length - 1)});
  if (calculatedCRC != data[length + 1]) {
    return ValidationStatus::INVALID_CRC;
  }

  return ValidationStatus::OK;
}

uint8_t Frame::calculateCRC8(const std::span<const uint8_t> data)
{
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

Frame::ParseResult Frame::parse_frame(const std::vector<uint8_t> & data)
{
  ParseResult result{
    .validation_status = validate(data),
    .type = FrameType::INVALID,
    .payload = std::nullopt,
  };

  if (result.validation_status != ValidationStatus::OK) {
    return result;
  }

  size_t length = data[1];

  result.type = static_cast<FrameType>(data[2]);
  result.payload->assign(data.begin() + 3, data.begin() + length + 1);

  return result;
}
}  // namespace crsf
