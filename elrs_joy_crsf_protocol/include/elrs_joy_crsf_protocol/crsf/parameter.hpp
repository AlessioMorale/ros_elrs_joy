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

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "elrs_joy_crsf_protocol/crsf/payload.hpp"

// Parameter protocol (0x2B entry, 0x2C read, 0x2D write) as used by the ExpressLRS Lua script.
// A parameter entry may be split in several 0x2B chunks; ParameterChunkAssembler joins them and
// parse_parameter_entry() decodes the joined data.
namespace elrs_joy_crsf_protocol::crsf
{

struct ParameterInfo
{
  uint8_t number = 0;
  uint8_t parent = 0;
  ParameterDataType type = ParameterDataType::OUT_OF_RANGE;
  bool hidden = false;
  std::string name;

  // UINT8..INT32 and FLOAT. For TEXT_SELECTION, value/min/max/default are option indices.
  int32_t value = 0;
  int32_t min = 0;
  int32_t max = 0;
  int32_t default_value = 0;
  uint8_t decimal_point = 0;  // FLOAT only
  int32_t step = 0;           // FLOAT only
  std::string unit;

  std::vector<std::string> options;             // TEXT_SELECTION
  std::string text;                             // STRING value, INFO text, COMMAND info
  uint8_t max_length = 0;                       // STRING only
  std::vector<uint8_t> children;                // FOLDER only
  CommandStatus status = CommandStatus::READY;  // COMMAND only
  uint8_t timeout = 0;                          // COMMAND only, LSB = 100 ms
};

// Decodes the joined data of a parameter entry (everything after parameter number and
// chunks remaining). Returns nullopt if the data is truncated or the type is unknown.
std::optional<ParameterInfo> parse_parameter_entry(
  uint8_t parameter_number, const std::vector<uint8_t> & data);

// Encodes a value for a 0x2D write. TEXT_SELECTION takes an option index, COMMAND a status.
std::vector<uint8_t> encode_parameter_value(ParameterDataType type, int32_t value);

// Joins the chunks of one parameter entry. Chunks must arrive in order (chunk 0 first).
class ParameterChunkAssembler
{
public:
  // Starts collecting a parameter; call before sending the 0x2C read for chunk 0.
  void start(uint8_t parameter_number);

  // Feeds a 0x2B payload. Returns the joined data once the last chunk is in; returns nullopt
  // while chunks are missing, or if the chunk does not belong to the current parameter.
  std::optional<std::vector<uint8_t>> feed(const ParameterEntryPayload & chunk);

  // Chunk number to request next with 0x2C
  uint8_t next_chunk() const { return next_chunk_; }
  uint8_t parameter_number() const { return parameter_number_; }
  bool active() const { return active_; }

private:
  bool active_ = false;
  uint8_t parameter_number_ = 0;
  uint8_t next_chunk_ = 0;
  std::optional<uint8_t> expected_remaining_;
  std::vector<uint8_t> data_;
};

}  // namespace elrs_joy_crsf_protocol::crsf
