// Copyright 2026 Alessio Morale <alessiomorale-at-gmail.com>
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
// SPDX-FileCopyrightText: 2026 Alessio Morale <alessiomorale-at-gmail.com>
// SPDX-License-Identifier: mit
//

#include "elrs_joy_crsf_protocol/crsf/parameter.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace elrs_joy_crsf_protocol::crsf
{
namespace
{
// Bounds-checked big-endian reader over the joined entry data
class Reader
{
public:
  explicit Reader(const std::vector<uint8_t> & data) : data_(data) {}

  bool ok() const { return ok_; }
  bool at_end() const { return offset_ >= data_.size(); }

  uint8_t u8()
  {
    if (offset_ + 1 > data_.size()) {
      ok_ = false;
      return 0;
    }
    return data_[offset_++];
  }

  uint32_t uint(size_t size)
  {
    if (offset_ + size > data_.size()) {
      ok_ = false;
      return 0;
    }
    uint32_t value = 0;
    for (size_t i = 0; i < size; ++i) {
      value = (value << 8) | data_[offset_++];
    }
    return value;
  }

  int32_t integer(size_t size, bool is_signed)
  {
    const uint32_t raw = uint(size);
    if (!is_signed || size == 4) {
      return static_cast<int32_t>(raw);
    }
    const uint32_t sign_bit = 1U << (size * 8 - 1);
    if (raw & sign_bit) {
      return static_cast<int32_t>(raw) - static_cast<int32_t>(sign_bit << 1);
    }
    return static_cast<int32_t>(raw);
  }

  std::string string()
  {
    const auto begin = data_.begin() + static_cast<std::ptrdiff_t>(offset_);
    const auto end = std::find(begin, data_.end(), 0);
    if (end == data_.end()) {
      ok_ = false;
      return {};
    }
    std::string value(begin, end);
    offset_ = static_cast<size_t>(std::distance(data_.begin(), end)) + 1;
    return value;
  }

  // Unit strings are optional at the end of an entry in some firmwares
  std::string optional_string() { return at_end() ? std::string{} : string(); }

private:
  const std::vector<uint8_t> & data_;
  size_t offset_ = 0;
  bool ok_ = true;
};

std::vector<std::string> split_options(const std::string & options)
{
  std::vector<std::string> result;
  std::string current;
  for (const char c : options) {
    if (c == ';') {
      result.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  result.push_back(current);
  return result;
}

size_t integer_size(ParameterDataType type)
{
  switch (type) {
    case ParameterDataType::UINT8:
    case ParameterDataType::INT8:
      return 1;
    case ParameterDataType::UINT16:
    case ParameterDataType::INT16:
      return 2;
    default:
      return 4;
  }
}

bool is_signed(ParameterDataType type)
{
  return type == ParameterDataType::INT8 || type == ParameterDataType::INT16 ||
         type == ParameterDataType::INT32 || type == ParameterDataType::FLOAT;
}
}  // namespace

std::optional<ParameterInfo> parse_parameter_entry(
  uint8_t parameter_number, const std::vector<uint8_t> & data)
{
  Reader reader(data);
  ParameterInfo info;
  info.number = parameter_number;
  info.parent = reader.u8();
  const uint8_t raw_type = reader.u8();
  info.hidden = (raw_type & 0x80) != 0;
  info.type = static_cast<ParameterDataType>(raw_type & 0x7F);
  if (!reader.ok()) {
    return std::nullopt;
  }
  if (info.type == ParameterDataType::OUT_OF_RANGE) {
    return info;
  }
  info.name = reader.string();

  switch (info.type) {
    case ParameterDataType::UINT8:
    case ParameterDataType::INT8:
    case ParameterDataType::UINT16:
    case ParameterDataType::INT16:
    case ParameterDataType::UINT32:
    case ParameterDataType::INT32:
    case ParameterDataType::FLOAT: {
      const size_t size = integer_size(info.type);
      const bool sign = is_signed(info.type);
      info.value = reader.integer(size, sign);
      info.min = reader.integer(size, sign);
      info.max = reader.integer(size, sign);
      info.default_value = reader.integer(size, sign);
      if (info.type == ParameterDataType::FLOAT) {
        info.decimal_point = reader.u8();
        info.step = reader.integer(4, true);
      }
      info.unit = reader.optional_string();
      break;
    }
    case ParameterDataType::TEXT_SELECTION:
      info.options = split_options(reader.string());
      info.value = reader.u8();
      info.min = reader.u8();
      info.max = reader.u8();
      info.default_value = reader.u8();
      info.unit = reader.optional_string();
      break;
    case ParameterDataType::STRING:
      info.text = reader.string();
      if (!reader.at_end()) {
        info.max_length = reader.u8();
      }
      break;
    case ParameterDataType::INFO:
      info.text = reader.string();
      break;
    case ParameterDataType::FOLDER:
      // The children list is optional in older firmwares; 0xFF ends it
      while (!reader.at_end()) {
        const uint8_t child = reader.u8();
        if (child == 0xFF) {
          break;
        }
        info.children.push_back(child);
      }
      break;
    case ParameterDataType::COMMAND:
      info.status = static_cast<CommandStatus>(reader.u8());
      info.timeout = reader.u8();
      info.text = reader.optional_string();
      break;
    default:
      return std::nullopt;
  }

  if (!reader.ok()) {
    return std::nullopt;
  }
  return info;
}

std::vector<uint8_t> encode_parameter_value(ParameterDataType type, int32_t value)
{
  size_t size = 1;
  switch (type) {
    case ParameterDataType::UINT16:
    case ParameterDataType::INT16:
      size = 2;
      break;
    case ParameterDataType::UINT32:
    case ParameterDataType::INT32:
    case ParameterDataType::FLOAT:
      size = 4;
      break;
    default:
      size = 1;
      break;
  }
  std::vector<uint8_t> data(size);
  auto raw = static_cast<uint32_t>(value);
  for (size_t i = 0; i < size; ++i) {
    data[size - 1 - i] = static_cast<uint8_t>(raw & 0xFF);
    raw >>= 8;
  }
  return data;
}

void ParameterChunkAssembler::start(uint8_t parameter_number)
{
  active_ = true;
  parameter_number_ = parameter_number;
  next_chunk_ = 0;
  expected_remaining_.reset();
  data_.clear();
}

std::optional<std::vector<uint8_t>> ParameterChunkAssembler::feed(
  const ParameterEntryPayload & chunk)
{
  if (!active_ || chunk.parameter_number != parameter_number_) {
    return std::nullopt;
  }
  // Each chunk must count down by one; anything else means a chunk was lost or repeated
  if (expected_remaining_ && chunk.chunks_remaining != *expected_remaining_) {
    start(parameter_number_);
    return std::nullopt;
  }

  data_.insert(data_.end(), chunk.data.begin(), chunk.data.end());
  if (chunk.chunks_remaining == 0) {
    active_ = false;
    return std::move(data_);
  }
  expected_remaining_ = static_cast<uint8_t>(chunk.chunks_remaining - 1);
  ++next_chunk_;
  return std::nullopt;
}

}  // namespace elrs_joy_crsf_protocol::crsf
