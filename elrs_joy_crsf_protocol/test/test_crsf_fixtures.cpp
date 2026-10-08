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

// Golden-frame tests: every fixture in test/fixtures is parsed, decoded, and re-encoded
// byte-exact. The same files are read by the Rust bindings tests (remote_controller/crates).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "elrs_joy_crsf_protocol/crsf/messages.hpp"
#include "elrs_joy_crsf_protocol/crsf/packets.hpp"
#include "elrs_joy_crsf_protocol/crsf/parameter.hpp"

#ifndef CRSF_FIXTURES_DIR
#error "CRSF_FIXTURES_DIR must point at test/fixtures"
#endif

namespace elrs_joy_crsf_protocol::crsf
{
namespace
{
std::vector<uint8_t> load_fixture(const std::string & name)
{
  std::ifstream file(std::filesystem::path(CRSF_FIXTURES_DIR) / (name + ".hex"));
  EXPECT_TRUE(file.good()) << "missing fixture " << name;
  std::vector<uint8_t> bytes;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream tokens(line);
    std::string token;
    while (tokens >> token) {
      bytes.push_back(static_cast<uint8_t>(std::stoul(token, nullptr, 16)));
    }
  }
  return bytes;
}

// Runs the bytes through the streaming parser; a fixture must decode to exactly one frame
std::optional<Message::Frame> parse_single(const std::vector<uint8_t> & bytes)
{
  std::vector<Message::Frame> frames;
  Packets parser([&frames](const Message::Frame & frame) { frames.push_back(frame); });
  for (const uint8_t byte : bytes) {
    parser.process_byte(byte);
  }
  if (frames.size() != 1) {
    return std::nullopt;
  }
  return frames.front();
}

Address sync_of(const Message::Frame & frame) { return static_cast<Address>(frame.get_sync()); }

template <typename MessageType>
MessageType decode_and_check_roundtrip(const std::string & name)
{
  const auto bytes = load_fixture(name);
  const auto frame = parse_single(bytes);
  EXPECT_TRUE(frame.has_value()) << name << " did not parse to exactly one frame";
  if (!frame) {
    return {};
  }
  const auto message = MessageType::from_frame(*frame);
  EXPECT_TRUE(message.has_value()) << name << " did not decode";
  if (!message) {
    return {};
  }
  EXPECT_EQ(message->to_frame(sync_of(*frame)).data, bytes) << name << " re-encode differs";
  return *message;
}

std::vector<std::string> all_fixtures()
{
  std::vector<std::string> names;
  for (const auto & entry : std::filesystem::directory_iterator(CRSF_FIXTURES_DIR)) {
    if (entry.path().extension() == ".hex") {
      names.push_back(entry.path().stem().string());
    }
  }
  return names;
}
}  // namespace

TEST(FixtureTest, EveryFixtureHasAValidFrame)
{
  const auto names = all_fixtures();
  ASSERT_GE(names.size(), 14u);
  for (const auto & name : names) {
    const auto bytes = load_fixture(name);
    const auto frame = parse_single(bytes);
    ASSERT_TRUE(frame.has_value()) << name;
    EXPECT_EQ(frame->data, bytes) << name;
    EXPECT_EQ(Message::parse_message(bytes).validation_status, ValidationStatus::OK) << name;
  }
}

TEST(FixtureTest, RcChannelsToTx)
{
  const auto message = decode_and_check_roundtrip<RCChannelsMessage>("rc_channels_to_tx");
  const std::array<uint16_t, 6> expected{1500, 2000, 1000, 1500, 2000, 1000};
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(message.payload.channels[i], expected[i]) << "channel " << i;
  }
  EXPECT_EQ(load_fixture("rc_channels_to_tx").front(), 0xEE);
}

TEST(FixtureTest, LinkStatistics)
{
  const auto message = decode_and_check_roundtrip<LinkStatisticsMessage>("link_statistics");
  EXPECT_EQ(message.payload.uplinkRssiAnt1, 64);
  EXPECT_EQ(message.payload.uplinkRssiAnt2, 70);
  EXPECT_EQ(message.payload.uplinkLinkQuality, 100);
  EXPECT_EQ(message.payload.uplinkSnr, 9);
  EXPECT_EQ(message.payload.rfMode, 7);
  EXPECT_EQ(message.payload.uplinkTxPower, RFPower::POWER_100MW);
  EXPECT_EQ(message.payload.downlinkRssi, 66);
  EXPECT_EQ(message.payload.downlinkLinkQuality, 97);
  EXPECT_EQ(message.payload.downlinkSnr, 8);
}

TEST(FixtureTest, BatterySensor)
{
  const auto message = decode_and_check_roundtrip<BatterySensorMessage>("battery_sensor");
  EXPECT_FLOAT_EQ(message.payload.voltage, 15.6F);
  EXPECT_FLOAT_EQ(message.payload.current, 1.2F);
  EXPECT_EQ(message.payload.usedCapacity, 450);
  EXPECT_EQ(message.payload.batteryPercent, 72);
}

TEST(FixtureTest, FlightMode)
{
  EXPECT_EQ(decode_and_check_roundtrip<FlightModeMessage>("flight_mode").payload.mode, "RDY");
  EXPECT_EQ(
    decode_and_check_roundtrip<FlightModeMessage>("flight_mode_fault").payload.mode, "FLT:MOTOR_L");
  EXPECT_EQ(
    decode_and_check_roundtrip<FlightModeMessage>("flight_mode_from_fc").payload.mode, "WRN:TEMP");
}

TEST(FixtureTest, OpenTxSync)
{
  const auto message = decode_and_check_roundtrip<OpenTxSyncMessage>("opentx_sync");
  EXPECT_EQ(message.payload.ext_header.ext_dest_addr, Address::CRSF_ADDRESS_RADIO_TRANSMITTER);
  EXPECT_EQ(message.payload.ext_header.ext_src_addr, Address::CRSF_ADDRESS_CRSF_TRANSMITTER);
  EXPECT_EQ(message.payload.update_interval, 40000u);
  EXPECT_EQ(message.payload.offset, -1200);
}

TEST(FixtureTest, OpenTxSyncRejectsOtherRadioIdSubtypes)
{
  auto bytes = load_fixture("opentx_sync");
  const auto frame = parse_single(bytes);
  ASSERT_TRUE(frame.has_value());
  auto other = *frame;
  other.data[5] = 0x11;
  EXPECT_FALSE(OpenTxSyncMessage::from_frame(other).has_value());
}

TEST(FixtureTest, DevicePing)
{
  const auto message = decode_and_check_roundtrip<DevicePingMessage>("device_ping");
  EXPECT_EQ(message.payload.ext_header.ext_dest_addr, Address::CRSF_ADDRESS_BROADCAST);
  EXPECT_EQ(message.payload.ext_header.ext_src_addr, Address::CRSF_ADDRESS_RADIO_TRANSMITTER);
}

TEST(FixtureTest, DeviceInfo)
{
  const auto message = decode_and_check_roundtrip<DeviceInfoMessage>("device_info");
  EXPECT_EQ(message.payload.device_name, "ELRS TX 2400");
  EXPECT_EQ(message.payload.serial_number, 0x454C5253u);
  EXPECT_EQ(message.payload.firmware_id, 0x00030503u);
  EXPECT_EQ(message.payload.parameters_total, 25);
}

TEST(FixtureTest, ParameterEntrySingleChunk)
{
  const auto message = decode_and_check_roundtrip<ParameterEntryMessage>("parameter_entry_single");
  EXPECT_EQ(message.payload.parameter_number, 5);
  EXPECT_EQ(message.payload.chunks_remaining, 0);

  ParameterChunkAssembler assembler;
  assembler.start(5);
  const auto data = assembler.feed(message.payload);
  ASSERT_TRUE(data.has_value());
  const auto info = parse_parameter_entry(5, *data);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->parent, 4);
  EXPECT_EQ(info->type, ParameterDataType::TEXT_SELECTION);
  EXPECT_FALSE(info->hidden);
  EXPECT_EQ(info->name, "Max Power");
  EXPECT_EQ(info->options, (std::vector<std::string>{"10", "25", "50", "100", "250"}));
  EXPECT_EQ(info->value, 3);
  EXPECT_EQ(info->max, 4);
  EXPECT_EQ(info->unit, "mW");
}

TEST(FixtureTest, ParameterEntryChunked)
{
  ParameterChunkAssembler assembler;
  assembler.start(1);
  std::optional<std::vector<uint8_t>> data;
  for (int chunk = 0; chunk < 3; ++chunk) {
    EXPECT_EQ(assembler.next_chunk(), chunk);
    const auto message = decode_and_check_roundtrip<ParameterEntryMessage>(
      "parameter_entry_chunk" + std::to_string(chunk));
    EXPECT_EQ(message.payload.chunks_remaining, 2 - chunk);
    data = assembler.feed(message.payload);
    EXPECT_EQ(data.has_value(), chunk == 2);
  }
  ASSERT_TRUE(data.has_value());
  const auto info = parse_parameter_entry(1, *data);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->name, "Packet Rate");
  ASSERT_EQ(info->options.size(), 6u);
  EXPECT_EQ(info->options[3], "250Hz(-108dBm)");
  EXPECT_EQ(info->value, 3);
  EXPECT_EQ(info->max, 5);
}

TEST(FixtureTest, ParameterEntryChunkOutOfOrderRestarts)
{
  ParameterChunkAssembler assembler;
  assembler.start(1);
  const auto chunk0 = decode_and_check_roundtrip<ParameterEntryMessage>("parameter_entry_chunk0");
  const auto chunk2 = decode_and_check_roundtrip<ParameterEntryMessage>("parameter_entry_chunk2");
  EXPECT_FALSE(assembler.feed(chunk0.payload).has_value());
  EXPECT_FALSE(assembler.feed(chunk2.payload).has_value());  // chunk 1 skipped
  EXPECT_EQ(assembler.next_chunk(), 0);
  EXPECT_TRUE(assembler.active());
}

TEST(FixtureTest, ParameterRead)
{
  const auto message = decode_and_check_roundtrip<ParameterReadMessage>("parameter_read");
  EXPECT_EQ(message.payload.ext_header.ext_dest_addr, Address::CRSF_ADDRESS_CRSF_TRANSMITTER);
  EXPECT_EQ(message.payload.ext_header.ext_src_addr, Address::CRSF_ADDRESS_ELRS_LUA);
  EXPECT_EQ(message.payload.parameter_number, 1);
  EXPECT_EQ(message.payload.chunk_number, 0);
}

TEST(FixtureTest, ParameterWrite)
{
  const auto message = decode_and_check_roundtrip<ParameterWriteMessage>("parameter_write");
  EXPECT_EQ(message.payload.parameter_number, 5);
  EXPECT_EQ(message.payload.data, encode_parameter_value(ParameterDataType::TEXT_SELECTION, 3));
}

TEST(ParameterTest, ParsesOtherTypes)
{
  // FLOAT: value 1.5 (15, 1 decimal), min 0, max 100, default 10, step 5, unit "V"
  const std::vector<uint8_t> float_entry{0,  0x08, 'G', 'a', 'i', 'n', 0, 0, 0,   0,
                                         15, 0,    0,   0,   0,   0,   0, 0, 100, 0,
                                         0,  0,    10,  1,   0,   0,   0, 5, 'V', 0};
  const auto f = parse_parameter_entry(9, float_entry);
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->value, 15);
  EXPECT_EQ(f->decimal_point, 1);
  EXPECT_EQ(f->step, 5);
  EXPECT_EQ(f->unit, "V");

  // Hidden INT8 with a negative value
  const std::vector<uint8_t> int8_entry{0, 0x81, 'T', 0, 0xFE, 0x80, 0x7F, 0x00, 0};
  const auto i = parse_parameter_entry(3, int8_entry);
  ASSERT_TRUE(i.has_value());
  EXPECT_TRUE(i->hidden);
  EXPECT_EQ(i->type, ParameterDataType::INT8);
  EXPECT_EQ(i->value, -2);
  EXPECT_EQ(i->min, -128);
  EXPECT_EQ(i->max, 127);

  // FOLDER with children, COMMAND, INFO
  const std::vector<uint8_t> folder{0, 0x0B, 'T', 'X', 0, 5, 6, 7, 0xFF};
  EXPECT_EQ(parse_parameter_entry(4, folder)->children, (std::vector<uint8_t>{5, 6, 7}));
  const std::vector<uint8_t> command{0, 0x0D, 'B', 'i', 'n', 'd', 0, 2, 50, 'B', 'u', 's', 'y', 0};
  const auto c = parse_parameter_entry(10, command);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->status, CommandStatus::PROGRESS);
  EXPECT_EQ(c->timeout, 50);
  EXPECT_EQ(c->text, "Busy");
  const std::vector<uint8_t> info{0, 0x0C, 'V', 'e', 'r', 0, '3', '.', '5', 0};
  EXPECT_EQ(parse_parameter_entry(11, info)->text, "3.5");
}

TEST(ParameterTest, RejectsTruncatedEntries)
{
  const auto full = std::vector<uint8_t>{4, 0x09, 'P', 0, 'a', ';', 'b', 0, 1, 0, 1, 0, 0};
  ASSERT_TRUE(parse_parameter_entry(5, full).has_value());
  // The trailing unit string is optional, so only cuts before it (< 12 bytes) must fail
  for (std::ptrdiff_t len = 0; len < 12; ++len) {
    EXPECT_FALSE(
      parse_parameter_entry(5, std::vector<uint8_t>(full.begin(), full.begin() + len)).has_value())
      << len;
  }
}

TEST(ParameterTest, EncodesValuesBigEndian)
{
  EXPECT_EQ(
    encode_parameter_value(ParameterDataType::TEXT_SELECTION, 2), (std::vector<uint8_t>{2}));
  EXPECT_EQ(
    encode_parameter_value(ParameterDataType::INT16, -2), (std::vector<uint8_t>{0xFF, 0xFE}));
  EXPECT_EQ(
    encode_parameter_value(ParameterDataType::FLOAT, 0x01020304),
    (std::vector<uint8_t>{1, 2, 3, 4}));
}

}  // namespace elrs_joy_crsf_protocol::crsf
