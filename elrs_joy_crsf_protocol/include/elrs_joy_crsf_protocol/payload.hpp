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
#include <array>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "elrs_joy_crsf_protocol/frame.hpp"
namespace crsf
{

struct ExtendedHeader
{
  Address ext_src_addr;
  Address ext_dest_addr;
  static constexpr size_t SIZE = 2;  // Size in bytes
};

enum class CommandRealm : uint8_t
{
  CRSF_COMMAND_SUBCMD_RX = 0x10,      // Receiver
  CRSF_COMMAND_SUBCMD_GENERAL = 0x0A  // General
};

enum class Command : uint8_t
{
  CRSF_COMMAND_SUBCMD_RX_BIND = 0x01,                      // Enter binding mode
  CRSF_COMMAND_SUBCMD_RX_MODEL_SELECT_ID = 0x05,           // Set Receiver / Model ID
  CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL = 0x70,  // (CRSFv3) Proposed new CRSF port speed
  CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE =
    0x71  // (CRSFv3) response to the proposed CRSF port speed
};

// 500mW, 1000mW, 2000mW, 250mW, 50mW}
enum class RFPower : uint8_t
{
  POWER_0MW = 0,
  POWER_10MW,
  POWER_25MW,
  POWER_100MW,
  POWER_500MW,
  POWER_1000MW,
  POWER_2000MW,
  POWER_250MW,
  POWER_50MW
};

struct BatterySensorPayload
{
  float voltage;           // Voltage in V
  float current;           // Current in A
  int32_t usedCapacity;    // Used capacity in mAh
  uint8_t batteryPercent;  // Battery percentage

  static constexpr size_t SIZE = 8;  // Size in bytes
};

struct HeartbeatPayload
{
  Address originDeviceAddress;
  static constexpr std::size_t SIZE = 2;  // Size in bytes
};

struct LinkStatisticsPayload
{
  uint8_t uplinkRssiAnt1;     // dBm * -1
  uint8_t uplinkRssiAnt2;     // dBm * -1
  uint8_t uplinkLinkQuality;  // %
  int8_t uplinkSnr;           // dB
  uint8_t activeAntenna;
  uint8_t rfMode;
  RFPower uplinkTxPower;
  uint8_t downlinkRssi;         // dBm * -1
  uint8_t downlinkLinkQuality;  // %
  int8_t downlinkSnr;           // dB

  static constexpr std::size_t SIZE = 10;  // Size in bytes
};

struct RCChannelsPayload
{
  std::array<uint16_t, 16> channels;  // 16 channels, 11-bit values

  static constexpr inline uint16_t ticks_to_us(uint16_t x)
  {
    return static_cast<uint16_t>((x - 992) * 5 / 8 + 1500);
  }

  static constexpr inline uint16_t us_to_ticks(uint16_t x)
  {
    return static_cast<uint16_t>((x - 1500) * 8 / 5 + 992);
  }

  static constexpr std::size_t SIZE = 22;  // Size in bytes (11 * 16 bits = 176 bits = 22 bytes)
};

struct CommandPayload
{
  ExtendedHeader ext_header;
  CommandRealm realm;
  Command command;
  std::vector<uint8_t> data;
  static constexpr size_t BASE_SIZE = ExtendedHeader::SIZE + 2;  // Size in bytes
};

}  // namespace crsf
