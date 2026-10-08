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

#ifndef ELRS_JOY_CRSF_PROTOCOL__CRSF__PACKETS_HPP_
#define ELRS_JOY_CRSF_PROTOCOL__CRSF__PACKETS_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <span>  // NOLINT
#include <vector>

#include "elrs_joy_crsf_protocol/crsf/message.hpp"

namespace elrs_joy_crsf_protocol::crsf
{
class Packets
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

  using FrameCallback = std::function<void(const Message::Frame &)>;

  // 0xC8: flight controller (RX side), 0x00: broadcast, 0xEE: TX module (frames to the module),
  // 0xEA: handset (frames from the TX module to the handset)
  static constexpr std::array<uint8_t, 4> VALID_SYNC_BYTES = {0xC8, 0x00, 0xEE, 0xEA};
  static constexpr uint8_t MIN_FRAME_LENGTH = 2;
  static constexpr uint8_t MAX_FRAME_LENGTH = 62;

  // Constructor with optional callback
  explicit Packets(FrameCallback callback = nullptr) : frame_callback(callback) {}

  // Set callback after construction
  void set_callback(FrameCallback callback) { frame_callback = callback; }

  // Returns true if a complete frame was parsed
  bool process_byte(uint8_t byte)
  {
    stats.total_bytes_processed++;

    switch (current_state) {
      case State::WAITING_SYNC:
        if (is_valid_sync(byte)) {
          current_frame.data.resize(0);
          current_frame.data.push_back(byte);
          current_state = State::WAITING_LENGTH;
        } else {
          stats.sync_errors++;
        }
        break;

      case State::WAITING_LENGTH:
        if (is_valid_length(byte)) {
          current_frame.data.push_back(byte);
          current_state = State::WAITING_TYPE;
        } else {
          stats.length_errors++;
          current_state = State::WAITING_SYNC;
        }
        break;

      case State::WAITING_TYPE:
        current_frame.data.push_back(byte);
        if (current_frame.get_length() == MIN_FRAME_LENGTH) {
          current_state = State::WAITING_CRC;
        } else {
          current_state = State::WAITING_PAYLOAD;
        }
        break;

      case State::WAITING_PAYLOAD:
        current_frame.data.push_back(byte);
        if (current_frame.data.size() == static_cast<size_t>(current_frame.get_length() + 1)) {
          current_state = State::WAITING_CRC;
        }
        break;

      case State::WAITING_CRC:
        current_frame.data.push_back(byte);
        if (verify_crc()) {
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

  // Get current parser state
  State get_current_state() const { return current_state; }

  // Calculate frame success rate as percentage
  float get_success_rate() const
  {
    float total_attempts = static_cast<float>(
      stats.frames_decoded + stats.sync_errors + stats.length_errors + stats.crc_errors);
    if (total_attempts == 0) return 0.0f;
    return (static_cast<float>(stats.frames_decoded) / total_attempts) * 100.0f;
  }

private:
  State current_state = State::WAITING_SYNC;
  Message::Frame current_frame;
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

  void reset_frame() { current_frame.data.clear(); }

  bool verify_crc() const
  {
    if (current_frame.data.size() < 4) {
      return false;
    }

    const auto crc_input =
      std::span<const uint8_t>(current_frame.data.data() + 2, current_frame.data.size() - 3);
    const auto calculated_crc = Message::calculateCRC8(crc_input);
    return calculated_crc == current_frame.get_crc();
  }
};
}  // namespace elrs_joy_crsf_protocol::crsf
#endif  // ELRS_JOY_CRSF_PROTOCOL__CRSF__PACKETS_HPP_
