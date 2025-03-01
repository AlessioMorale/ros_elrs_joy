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

#ifndef ELRS_JOY_CRSF_PROTOCOL__FRAME_PARSING_HPP_
#define ELRS_JOY_CRSF_PROTOCOL__FRAME_PARSING_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include "frame.hpp"
namespace crsf
{

class FramePacket
{
public:
  struct Statistics
  {
    uint32_t total_bytes_processed = 0;
    uint32_t frames_decoded = 0;
    uint32_t sync_errors = 0;
    uint32_t length_errors = 0;
    uint32_t crc_errors = 0;

    void reset()
    {
      total_bytes_processed = 0;
      frames_decoded = 0;
      sync_errors = 0;
      length_errors = 0;
      crc_errors = 0;
    }

    // Calculate frame success rate as percentage
    float get_success_rate() const
    {
      uint32_t total_attempts = frames_decoded + sync_errors + length_errors + crc_errors;
      if (total_attempts == 0) {
        return 0.0f;
      }
      return (static_cast<float>(frames_decoded) / static_cast<float>(total_attempts)) * 100.0f;
    }
  };

  // Parser states
  enum class State
  {
    WAITING_SYNC,
    WAITING_LENGTH,
    WAITING_TYPE,
    WAITING_PAYLOAD,
    WAITING_CRC
  };

  using FrameCallback = std::function<void(const crsf::Frame &)>;

  static constexpr std::array<uint8_t, 3> VALID_SYNC_BYTES = {0xC8, 0x00, 0xEE};
  static constexpr uint8_t MIN_FRAME_LENGTH = 2;
  static constexpr uint8_t MAX_FRAME_LENGTH = 62;

  // Constructor with optional callback
  explicit FramePacket(FrameCallback callback = nullptr) : frame_callback(callback) {}

  // Set callback after construction
  void set_callback(FrameCallback callback) { frame_callback = callback; }

  // Returns true if a complete frame was parsed
  bool process_byte(uint8_t byte)
  {
    stats.total_bytes_processed++;

    switch (current_state) {
      case State::WAITING_SYNC:
        if (is_valid_sync(byte)) {
          current_frame_data.clear();
          current_frame_data.push_back(byte);
          current_state = State::WAITING_LENGTH;
        } else {
          stats.sync_errors++;
        }
        break;

      case State::WAITING_LENGTH:
        if (is_valid_length(byte)) {
          current_frame_data.push_back(byte);
          current_state = State::WAITING_TYPE;
        } else {
          stats.length_errors++;
          current_state = State::WAITING_SYNC;
        }
        break;

      case State::WAITING_TYPE:
        current_frame_data.push_back(byte);
        if (current_frame.get_length() == MIN_FRAME_LENGTH) {
          current_state = State::WAITING_CRC;
        } else {
          current_state = State::WAITING_PAYLOAD;
        }
        break;

      case State::WAITING_PAYLOAD:
        current_frame_data.push_back(byte);
        if (current_frame.get_storage_size() < current_frame.get_length() + 1) {
          break;
        }
        current_state = State::WAITING_CRC;
        break;

      case State::WAITING_CRC:
        current_frame_data.push_back(byte);
        if (current_frame.check_crc()) {
          stats.frames_decoded++;
          if (frame_callback) {
            frame_callback(current_frame);
          }
          current_state = State::WAITING_SYNC;
          return true;
        } else {
          stats.crc_errors++;
          current_state = State::WAITING_SYNC;
        }
        break;
    }
    return false;
  }

  // Get current statistics
  const Statistics & get_statistics() const { return stats; }

  // Reset statistics
  void reset_statistics() { stats.reset(); }

private:
  State current_state = State::WAITING_SYNC;
  std::vector<uint8_t> current_frame_data;
  Frame current_frame = Frame(current_frame_data);
  Statistics stats;
  FrameCallback frame_callback;

  bool is_valid_sync(uint8_t byte) const
  {
    return std::find(VALID_SYNC_BYTES.begin(), VALID_SYNC_BYTES.end(), byte) !=
           VALID_SYNC_BYTES.end();
  }

  bool is_valid_length(uint8_t length) const
  {
    return length >= MIN_FRAME_LENGTH && length <= MAX_FRAME_LENGTH;
  }

  void reset_frame() { current_frame_data.clear(); }
};

}  // namespace crsf
#endif  // ELRS_JOY_CRSF_PROTOCOL__FRAME_PARSING_HPP_
