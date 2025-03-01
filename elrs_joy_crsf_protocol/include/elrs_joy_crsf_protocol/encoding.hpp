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

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <span>  // NOLINT
#include <vector>

namespace crsf
{

class Decoder
{
public:
  explicit Decoder(const std::vector<uint8_t> & frame_data) : ro_data{frame_data} {}

  inline uint16_t readU16(size_t & position) const
  {
    auto ret = static_cast<uint16_t>(readU8(position) << 8);
    ret |= readU8(position);
    return ret;
  }

  inline uint8_t readU8(size_t & position) const { return ro_data.get()[position++]; }
  inline int8_t readI8(size_t & position) const { return static_cast<int8_t>(readU8(position)); }
  inline int16_t readI16(size_t & position) const
  {
    return static_cast<int16_t>(readU16(position));
  }
  inline int32_t readBS24(size_t & position) const
  {
    uint32_t value = static_cast<uint32_t>((readU8(position) << 16));
    value |= static_cast<uint32_t>(readU8(position) << 8);
    value |= readU8(position);
    bool sign = (value >> 23) & 1;
    int32_t magnitude = value & 0x7FFFFF;
    return sign ? -magnitude : magnitude;
  }

  void set_read_position(size_t position) { read_position = position; }
  inline uint8_t readU8() { return readU8(read_position); }
  inline uint16_t readU16() { return readU16(read_position); }
  inline int8_t readI8() { return readI8(read_position); }
  inline int16_t readI16() { return readI16(read_position); }
  inline int32_t readBS24() { return readBS24(read_position); }

private:
  std::reference_wrapper<const std::vector<uint8_t>> ro_data;
  size_t read_position = 0;
};

class Encoder
{
public:
  explicit Encoder(std::vector<uint8_t> & frame_data) : data(frame_data) {}

  inline void writeU8(uint8_t value, size_t & position)
  {
    auto & v = data.get();
    if (write_position >= v.size()) {
      v.resize(position + 1);
    }
    v[position++] = value;
  }

  inline void writeU16(uint16_t value, size_t & position)
  {
    writeU8(static_cast<uint8_t>((value >> 8) & 0xFF), position);
    writeU8(static_cast<uint8_t>(value & 0xFF), position);
  }

  inline void writeI8(int8_t value, size_t & position)
  {
    writeU8(static_cast<uint8_t>(value), position);
  }
  inline void writeI16(int16_t value, size_t & position)
  {
    writeU16(static_cast<uint16_t>(value), position);
  }
  inline void writeBS24(int32_t value, size_t & position)
  {
    bool sign = value < 0;
    value = std::abs(value);

    if (value >= (1 << 23)) {
      value = (1 << 23) - 1;
    }

    uint32_t packed = (sign ? 1U << 23 : 0) | (value & 0x7FFFFF);
    writeU8((packed >> 16) & 0xFF, position);
    writeU8((packed >> 8) & 0xFF, position);
    writeU8(packed & 0xFF, position);
  }

  void set_write_position(size_t position) { write_position = position; }
  inline void writeI8(int8_t value) { writeI8(value, write_position); }
  inline void writeI16(int16_t value) { writeI16(value, write_position); }
  inline void writeU8(uint8_t value) { writeU8(value, write_position); }
  inline void writeU16(uint16_t value) { writeU16(value, write_position); }
  inline void writeBS24(int32_t value) { writeBS24(value, write_position); }

protected:
  size_t write_position = 0;
  std::reference_wrapper<std::vector<uint8_t>> data;
};
}  // namespace crsf
