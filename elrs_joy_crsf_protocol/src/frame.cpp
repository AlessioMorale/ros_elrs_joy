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

bool FrameSerializer::is_valid_sync(uint8_t sync)
{
  return sync == SYNC_BYTE || sync == SYNC_BYTE_EDGETX || sync == SYNC_BYTE_BROADCAST;
}

std::vector<uint8_t> FrameSerializer::serialize(
  FrameType type, const std::vector<uint8_t> & payload, Address sync_byte)
{
  std::vector<uint8_t> frame_storage;
  Frame frame{frame_storage};
  frame.set_sync(sync_byte);
  frame.set_size(payload.size() + 2);  // +2 for type and CRC
  frame.set_type(type);
  frame.set_payload({payload.begin(), payload.size()});
  frame.update_crc();
  return frame_storage;
}

inline ValidationStatus FrameSerializer::validate(Frame & frame)
{
  if (frame.get_storage_size() < MIN_FRAME_LEN + 2) {
    return ValidationStatus::INVALID_NOT_ENOUGH_DATA;
  }

  auto sync = frame.get_sync_byte();
  [[maybe_unused]] auto type = frame.get_type();

  if (!is_valid_sync(sync)) {
    return ValidationStatus::UNSUPPORTED_SYNC;
  }

  if (!frame.check_size()) {
    return ValidationStatus::INVALID_NOT_ENOUGH_DATA;
  }

  if (!frame.check_crc()) {
    return ValidationStatus::INVALID_CRC;
  }

  return ValidationStatus::OK;
}

FrameSerializer::SerializationResult FrameSerializer::deserialize(std::vector<uint8_t> & data)
{
  Frame frame(data);
  SerializationResult result{
    .validation_status = validate(frame), .type = FrameType::INVALID, .payload = {}};
  result.payload.reset();
  if (result.validation_status != ValidationStatus::OK) {
    return result;
  }

  [[maybe_unused]] size_t length = frame.get_length();

  result.type = static_cast<FrameType>(data[2]);
  auto payload = frame.get_payload();
  result.payload = {payload.begin(), payload.end()};

  return result;
}
}  // namespace crsf
