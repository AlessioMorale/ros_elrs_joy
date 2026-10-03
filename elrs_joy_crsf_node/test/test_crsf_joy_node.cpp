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

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "elrs_joy_crsf_node/comm/port.hpp"
#include "elrs_joy_crsf_node/crsf_joy_node.hpp"
#include "elrs_joy_crsf_protocol/crsf/messages.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace elrs_joy_crsf_node
{
namespace
{
class FakePort : public comm::Port
{
public:
  void close() override {}

  void set_receive_callback(comm::receive_callback_t callback) override
  {
    callback_ = std::move(callback);
  }

  void send(const std::vector<uint8_t> & data) override { sent_frames_.push_back(data); }

  void emit(const std::vector<uint8_t> & data)
  {
    if (callback_) {
      callback_(data);
    }
  }

  std::vector<std::vector<uint8_t>> sent_frames_;

private:
  comm::receive_callback_t callback_;
};

bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor, const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < timeout) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

rclcpp::NodeOptions make_single_mapping_options(const std::string & joy_topic)
{
  rclcpp::NodeOptions options;
  options.context(rclcpp::contexts::get_global_default_context());
  options.append_parameter_override("joy_topic", joy_topic);
  options.append_parameter_override("axis_mappings.channels", std::vector<int64_t>{0});
  options.append_parameter_override("axis_mappings.scale", std::vector<double>{1.0});
  options.append_parameter_override("axis_mappings.offset", std::vector<double>{0.0});
  options.append_parameter_override("axis_mappings.invert", std::vector<bool>{false});
  options.append_parameter_override("axis_mappings.deadzone", std::vector<double>{0.0});
  options.append_parameter_override("axis_mappings.min", std::vector<double>{-1.0});
  options.append_parameter_override("axis_mappings.max", std::vector<double>{1.0});
  options.append_parameter_override("button_mappings.channels", std::vector<int64_t>{4});
  options.append_parameter_override("button_mappings.threshold", std::vector<double>{0.5});
  options.append_parameter_override("button_mappings.invert", std::vector<bool>{false});
  return options;
}

// Collect the latest status per diagnostic task name (suffix after "<node>: ")
struct DiagCollector
{
  std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> latest;

  void operator()(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    for (const auto & status : msg->status) {
      const auto pos = status.name.find(": ");
      latest[pos == std::string::npos ? status.name : status.name.substr(pos + 2)] = status;
    }
  }
};

std::string diag_value(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return "";
}

}  // namespace

TEST(CRSFJoyNodeTest, MapsRCChannelsToJoy)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions options;
  options.context(rclcpp::contexts::get_global_default_context());
  options.append_parameter_override("joy_topic", "test_joy");
  options.append_parameter_override("failsafe_timeout_ms", 2000);
  options.append_parameter_override("joy_publish_period_ms", 20);
  options.append_parameter_override("axis_mappings.channels", std::vector<int64_t>{0});
  options.append_parameter_override("axis_mappings.scale", std::vector<double>{1.0});
  options.append_parameter_override("axis_mappings.offset", std::vector<double>{0.0});
  options.append_parameter_override("axis_mappings.invert", std::vector<bool>{false});
  options.append_parameter_override("axis_mappings.deadzone", std::vector<double>{0.0});
  options.append_parameter_override("axis_mappings.min", std::vector<double>{-1.0});
  options.append_parameter_override("axis_mappings.max", std::vector<double>{1.0});
  options.append_parameter_override("button_mappings.channels", std::vector<int64_t>{4});
  options.append_parameter_override("button_mappings.threshold", std::vector<double>{0.5});
  options.append_parameter_override("button_mappings.invert", std::vector<bool>{false});

  auto fake_port = std::make_unique<FakePort>();
  auto * fake_port_raw = fake_port.get();

  auto node = std::make_shared<CRSFJoyPublisher>(std::move(fake_port), options);
  auto helper_node = std::make_shared<rclcpp::Node>("joy_test_helper");

  sensor_msgs::msg::Joy::SharedPtr last_joy;
  auto sub = helper_node->create_subscription<sensor_msgs::msg::Joy>(
    "test_joy", 10, [&](const sensor_msgs::msg::Joy::SharedPtr msg) { last_joy = msg; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  elrs_joy_crsf_protocol::crsf::RCChannelsMessage channels_message;
  channels_message.payload.channels.fill(1500);
  channels_message.payload.channels[0] = 2000;
  channels_message.payload.channels[4] = 2000;

  fake_port_raw->emit(channels_message.to_frame().data);

  const bool got_msg = spin_until(
    executor, [&]() { return static_cast<bool>(last_joy); }, std::chrono::milliseconds(400));
  ASSERT_TRUE(got_msg);
  ASSERT_EQ(last_joy->axes.size(), 1U);
  ASSERT_EQ(last_joy->buttons.size(), 1U);
  EXPECT_NEAR(last_joy->axes[0], 1.0, 0.05);
  EXPECT_EQ(last_joy->buttons[0], 1);

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)sub;
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, PublishesFailsafeOnceWhenConfigured)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions options;
  options.context(rclcpp::contexts::get_global_default_context());
  options.append_parameter_override("joy_topic", "failsafe_joy");
  options.append_parameter_override("failsafe_timeout_ms", 80);
  options.append_parameter_override("monitor_period_ms", 20);
  options.append_parameter_override("joy_publish_period_ms", 20);
  options.append_parameter_override("send_failsafe_continuously", false);
  options.append_parameter_override("axis_mappings.channels", std::vector<int64_t>{0});
  options.append_parameter_override("axis_mappings.scale", std::vector<double>{1.0});
  options.append_parameter_override("axis_mappings.offset", std::vector<double>{0.0});
  options.append_parameter_override("axis_mappings.invert", std::vector<bool>{false});
  options.append_parameter_override("axis_mappings.deadzone", std::vector<double>{0.0});
  options.append_parameter_override("axis_mappings.min", std::vector<double>{-1.0});
  options.append_parameter_override("axis_mappings.max", std::vector<double>{1.0});
  options.append_parameter_override("button_mappings.channels", std::vector<int64_t>{4});
  options.append_parameter_override("button_mappings.threshold", std::vector<double>{0.5});
  options.append_parameter_override("button_mappings.invert", std::vector<bool>{false});
  options.append_parameter_override("failsafe_axes", std::vector<double>{-0.25});
  options.append_parameter_override("failsafe_buttons", std::vector<int64_t>{1});

  auto node = std::make_shared<CRSFJoyPublisher>(std::make_unique<FakePort>(), options);
  auto helper_node = std::make_shared<rclcpp::Node>("failsafe_test_helper");

  size_t msg_count = 0;
  sensor_msgs::msg::Joy::SharedPtr last_joy;
  auto sub = helper_node->create_subscription<sensor_msgs::msg::Joy>(
    "failsafe_joy", 10, [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
      ++msg_count;
      last_joy = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  const bool got_failsafe =
    spin_until(executor, [&]() { return msg_count > 0; }, std::chrono::milliseconds(400));
  ASSERT_TRUE(got_failsafe);

  const size_t first_window_count = msg_count;
  const bool got_extra = spin_until(
    executor, [&]() { return msg_count > first_window_count; }, std::chrono::milliseconds(250));
  EXPECT_FALSE(got_extra);

  ASSERT_TRUE(last_joy);
  EXPECT_NEAR(last_joy->axes[0], -0.25, 0.001);
  EXPECT_EQ(last_joy->buttons[0], 1);

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)sub;
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, PublishesFailsafeContinuouslyWhenConfigured)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  auto options = make_single_mapping_options("failsafe_continuous_joy");
  options.append_parameter_override("failsafe_timeout_ms", 80);
  options.append_parameter_override("monitor_period_ms", 20);
  options.append_parameter_override("joy_publish_period_ms", 20);
  options.append_parameter_override("send_failsafe_continuously", true);
  options.append_parameter_override("failsafe_axes", std::vector<double>{-0.5});
  options.append_parameter_override("failsafe_buttons", std::vector<int64_t>{1});

  auto node = std::make_shared<CRSFJoyPublisher>(std::make_unique<FakePort>(), options);
  auto helper_node = std::make_shared<rclcpp::Node>("failsafe_continuous_helper");

  size_t msg_count = 0;
  sensor_msgs::msg::Joy::SharedPtr last_joy;
  auto sub = helper_node->create_subscription<sensor_msgs::msg::Joy>(
    "failsafe_continuous_joy", 10, [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
      ++msg_count;
      last_joy = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  const bool got_multiple_failsafe =
    spin_until(executor, [&]() { return msg_count >= 2; }, std::chrono::milliseconds(450));
  ASSERT_TRUE(got_multiple_failsafe);

  ASSERT_TRUE(last_joy);
  ASSERT_EQ(last_joy->axes.size(), 1U);
  ASSERT_EQ(last_joy->buttons.size(), 1U);
  EXPECT_NEAR(last_joy->axes[0], -0.5, 0.001);
  EXPECT_EQ(last_joy->buttons[0], 1);

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)sub;
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, LeavesFailsafeAndPublishesMappedInputWhenRcResumes)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  auto options = make_single_mapping_options("failsafe_recovery_joy");
  options.append_parameter_override("failsafe_timeout_ms", 80);
  options.append_parameter_override("monitor_period_ms", 20);
  options.append_parameter_override("joy_publish_period_ms", 20);
  options.append_parameter_override("send_failsafe_continuously", false);
  options.append_parameter_override("failsafe_axes", std::vector<double>{-0.25});
  options.append_parameter_override("failsafe_buttons", std::vector<int64_t>{0});

  auto fake_port = std::make_unique<FakePort>();
  auto * fake_port_raw = fake_port.get();
  auto node = std::make_shared<CRSFJoyPublisher>(std::move(fake_port), options);
  auto helper_node = std::make_shared<rclcpp::Node>("failsafe_recovery_helper");

  sensor_msgs::msg::Joy::SharedPtr last_joy;
  size_t msg_count = 0;
  auto sub = helper_node->create_subscription<sensor_msgs::msg::Joy>(
    "failsafe_recovery_joy", 10, [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
      ++msg_count;
      last_joy = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  const bool got_initial_failsafe =
    spin_until(executor, [&]() { return msg_count >= 1; }, std::chrono::milliseconds(350));
  ASSERT_TRUE(got_initial_failsafe);
  ASSERT_TRUE(last_joy);
  EXPECT_NEAR(last_joy->axes[0], -0.25, 0.001);

  elrs_joy_crsf_protocol::crsf::RCChannelsMessage channels_message;
  channels_message.payload.channels.fill(1500);
  channels_message.payload.channels[0] = 2000;
  channels_message.payload.channels[4] = 2000;
  fake_port_raw->emit(channels_message.to_frame().data);

  const bool got_mapped_after_recovery = spin_until(
    executor,
    [&]() {
      return last_joy && last_joy->axes.size() == 1U && last_joy->buttons.size() == 1U &&
             std::abs(last_joy->axes[0] - 1.0) < 0.05 && last_joy->buttons[0] == 1;
    },
    std::chrono::milliseconds(450));
  ASSERT_TRUE(got_mapped_after_recovery);

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)sub;
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, UsesSafeEmptyMappingWhenConfigIsInvalid)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  auto options = make_single_mapping_options("invalid_mapping_joy");
  options.append_parameter_override("joy_publish_period_ms", 20);
  options.append_parameter_override("failsafe_timeout_ms", 300);
  options.append_parameter_override("send_failsafe_continuously", false);
  options.append_parameter_override("axis_mappings.channels", std::vector<int64_t>{16});
  options.append_parameter_override("failsafe_axes", std::vector<double>{0.7});
  options.append_parameter_override("failsafe_buttons", std::vector<int64_t>{1});

  auto fake_port = std::make_unique<FakePort>();
  auto * fake_port_raw = fake_port.get();
  auto node = std::make_shared<CRSFJoyPublisher>(std::move(fake_port), options);
  auto helper_node = std::make_shared<rclcpp::Node>("invalid_mapping_helper");

  sensor_msgs::msg::Joy::SharedPtr last_joy;
  size_t msg_count = 0;
  auto sub = helper_node->create_subscription<sensor_msgs::msg::Joy>(
    "invalid_mapping_joy", 10, [&](const sensor_msgs::msg::Joy::SharedPtr msg) {
      ++msg_count;
      last_joy = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  const bool got_initial_safe_output =
    spin_until(executor, [&]() { return msg_count >= 1; }, std::chrono::milliseconds(350));
  ASSERT_TRUE(got_initial_safe_output);
  ASSERT_TRUE(last_joy);
  EXPECT_TRUE(last_joy->axes.empty());
  EXPECT_TRUE(last_joy->buttons.empty());

  elrs_joy_crsf_protocol::crsf::RCChannelsMessage channels_message;
  channels_message.payload.channels.fill(2000);
  fake_port_raw->emit(channels_message.to_frame().data);

  const bool got_post_rc_output =
    spin_until(executor, [&]() { return msg_count >= 2; }, std::chrono::milliseconds(450));
  ASSERT_TRUE(got_post_rc_output);
  ASSERT_TRUE(last_joy);
  EXPECT_TRUE(last_joy->axes.empty());
  EXPECT_TRUE(last_joy->buttons.empty());

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)sub;
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, ResetsFailsafeOutputsWhenSizesDoNotMatchMappings)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions options;
  options.context(rclcpp::contexts::get_global_default_context());
  options.append_parameter_override("joy_topic", "failsafe_size_sanitize_joy");
  options.append_parameter_override("failsafe_timeout_ms", 80);
  options.append_parameter_override("monitor_period_ms", 20);
  options.append_parameter_override("joy_publish_period_ms", 20);
  options.append_parameter_override("send_failsafe_continuously", false);
  options.append_parameter_override("axis_mappings.channels", std::vector<int64_t>{0, 1});
  options.append_parameter_override("axis_mappings.scale", std::vector<double>{1.0, 1.0});
  options.append_parameter_override("axis_mappings.offset", std::vector<double>{0.0, 0.0});
  options.append_parameter_override("axis_mappings.invert", std::vector<bool>{false, false});
  options.append_parameter_override("axis_mappings.deadzone", std::vector<double>{0.0, 0.0});
  options.append_parameter_override("axis_mappings.min", std::vector<double>{-1.0, -1.0});
  options.append_parameter_override("axis_mappings.max", std::vector<double>{1.0, 1.0});
  options.append_parameter_override("button_mappings.channels", std::vector<int64_t>{4});
  options.append_parameter_override("button_mappings.threshold", std::vector<double>{0.5});
  options.append_parameter_override("button_mappings.invert", std::vector<bool>{false});
  options.append_parameter_override("failsafe_axes", std::vector<double>{0.8});
  options.append_parameter_override("failsafe_buttons", std::vector<int64_t>{1, 1});

  auto node = std::make_shared<CRSFJoyPublisher>(std::make_unique<FakePort>(), options);
  auto helper_node = std::make_shared<rclcpp::Node>("failsafe_size_sanitize_helper");

  sensor_msgs::msg::Joy::SharedPtr last_joy;
  auto sub = helper_node->create_subscription<sensor_msgs::msg::Joy>(
    "failsafe_size_sanitize_joy", 10,
    [&](const sensor_msgs::msg::Joy::SharedPtr msg) { last_joy = msg; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  const bool got_failsafe = spin_until(
    executor, [&]() { return static_cast<bool>(last_joy); }, std::chrono::milliseconds(350));
  ASSERT_TRUE(got_failsafe);
  ASSERT_TRUE(last_joy);

  ASSERT_EQ(last_joy->axes.size(), 2U);
  ASSERT_EQ(last_joy->buttons.size(), 1U);
  EXPECT_NEAR(last_joy->axes[0], 0.0, 0.001);
  EXPECT_NEAR(last_joy->axes[1], 0.0, 0.001);
  EXPECT_EQ(last_joy->buttons[0], 0);

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)sub;
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, ConvertsBatteryStateToCRSFBatteryFrame)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions options;
  options.context(rclcpp::contexts::get_global_default_context());
  options.append_parameter_override("battery_topic", "battery_test");
  options.append_parameter_override("telemetry_battery_enabled", true);

  auto fake_port = std::make_unique<FakePort>();
  auto * fake_port_raw = fake_port.get();

  auto node = std::make_shared<CRSFJoyPublisher>(std::move(fake_port), options);
  auto helper_node = std::make_shared<rclcpp::Node>("battery_test_helper");
  auto pub = helper_node->create_publisher<sensor_msgs::msg::BatteryState>("battery_test", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  sensor_msgs::msg::BatteryState msg;
  msg.voltage = 24.0F;
  msg.current = 3.2F;
  msg.capacity = 10.0F;
  msg.charge = 7.5F;
  msg.percentage = 0.75F;
  pub->publish(msg);

  const bool sent = spin_until(
    executor, [&]() { return !fake_port_raw->sent_frames_.empty(); },
    std::chrono::milliseconds(300));
  ASSERT_TRUE(sent);

  const auto & frame_data = fake_port_raw->sent_frames_.back();
  elrs_joy_crsf_protocol::crsf::Message::Frame frame{frame_data};
  const auto battery = elrs_joy_crsf_protocol::crsf::BatterySensorMessage::from_frame(frame);
  ASSERT_TRUE(battery.has_value());
  EXPECT_NEAR(battery->payload.voltage, 24.0, 0.05);
  EXPECT_NEAR(battery->payload.current, 3.2, 0.05);
  EXPECT_EQ(battery->payload.usedCapacity, 2500);
  EXPECT_EQ(battery->payload.batteryPercent, 75);

  executor.remove_node(helper_node);
  executor.remove_node(node);
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, SanitizesInvalidBatteryValuesBeforeSendingCRSF)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  rclcpp::NodeOptions options;
  options.context(rclcpp::contexts::get_global_default_context());
  options.append_parameter_override("battery_topic", "battery_safety_test");
  options.append_parameter_override("telemetry_battery_enabled", true);

  auto fake_port = std::make_unique<FakePort>();
  auto * fake_port_raw = fake_port.get();

  auto node = std::make_shared<CRSFJoyPublisher>(std::move(fake_port), options);
  auto helper_node = std::make_shared<rclcpp::Node>("battery_safety_helper");
  auto pub =
    helper_node->create_publisher<sensor_msgs::msg::BatteryState>("battery_safety_test", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  sensor_msgs::msg::BatteryState msg;
  msg.voltage = std::numeric_limits<float>::quiet_NaN();
  msg.current = std::numeric_limits<float>::quiet_NaN();
  msg.capacity = std::numeric_limits<float>::quiet_NaN();
  msg.charge = std::numeric_limits<float>::quiet_NaN();
  msg.percentage = 1.5F;
  pub->publish(msg);

  const bool sent = spin_until(
    executor, [&]() { return !fake_port_raw->sent_frames_.empty(); },
    std::chrono::milliseconds(300));
  ASSERT_TRUE(sent);

  const auto & frame_data = fake_port_raw->sent_frames_.back();
  elrs_joy_crsf_protocol::crsf::Message::Frame frame{frame_data};
  const auto battery = elrs_joy_crsf_protocol::crsf::BatterySensorMessage::from_frame(frame);
  ASSERT_TRUE(battery.has_value());
  EXPECT_NEAR(battery->payload.voltage, 0.0, 0.001);
  EXPECT_NEAR(battery->payload.current, 0.0, 0.001);
  EXPECT_EQ(battery->payload.usedCapacity, 0);
  EXPECT_EQ(battery->payload.batteryPercent, 100);

  executor.remove_node(helper_node);
  executor.remove_node(node);
  rclcpp::shutdown();
}


TEST(CRSFJoyNodeTest, DiagnosticsReportLinkStatisticsAndBatteryTelemetry)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  auto options = make_single_mapping_options("diag_joy");
  options.append_parameter_override("failsafe_timeout_ms", 5000);
  options.append_parameter_override("diagnostic_updater.period", 0.1);

  auto fake_port = std::make_unique<FakePort>();
  auto * fake_port_raw = fake_port.get();
  auto node = std::make_shared<CRSFJoyPublisher>(std::move(fake_port), options);
  auto helper_node = std::make_shared<rclcpp::Node>("diag_test_helper");

  DiagCollector diag;
  auto diag_sub = helper_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { diag(msg); });
  auto battery_pub = helper_node->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  elrs_joy_crsf_protocol::crsf::RCChannelsMessage channels_message;
  channels_message.payload.channels.fill(1500);
  fake_port_raw->emit(channels_message.to_frame().data);

  elrs_joy_crsf_protocol::crsf::LinkStatisticsMessage link_message;
  link_message.payload = {};
  link_message.payload.uplinkRssiAnt1 = 57;
  link_message.payload.uplinkLinkQuality = 98;
  link_message.payload.uplinkSnr = 10;
  link_message.payload.uplinkTxPower = elrs_joy_crsf_protocol::crsf::RFPower::POWER_100MW;
  fake_port_raw->emit(link_message.to_frame().data);

  sensor_msgs::msg::BatteryState battery;
  battery.voltage = 15.4F;
  battery.current = 0.5F;
  battery.percentage = 0.8F;
  battery.capacity = std::numeric_limits<float>::quiet_NaN();
  battery.charge = std::numeric_limits<float>::quiet_NaN();

  const bool ok = spin_until(
    executor,
    [&]() {
      battery_pub->publish(battery);
      return diag.latest.count("RC link") && diag.latest.count("Battery telemetry") &&
             diag.latest["Battery telemetry"].level == 0 && diag.latest["RC link"].level == 0;
    },
    std::chrono::milliseconds(3000));
  ASSERT_TRUE(ok);

  const auto & link = diag.latest["RC link"];
  EXPECT_EQ(diag_value(link, "failsafe"), "False");
  EXPECT_EQ(diag_value(link, "uplink_rssi_ant1_dbm"), "-57");
  EXPECT_EQ(diag_value(link, "uplink_link_quality_pct"), "98");
  EXPECT_EQ(diag_value(link, "tx_power_mw"), "100");
  EXPECT_EQ(diag_value(link, "rc_frames"), "1");

  const auto & battery_diag = diag.latest["Battery telemetry"];
  EXPECT_NE(diag_value(battery_diag, "frames_sent"), "0");
  EXPECT_EQ(diag_value(battery_diag, "last_voltage_v"), "15.4");

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)diag_sub;
  rclcpp::shutdown();
}

TEST(CRSFJoyNodeTest, DiagnosticsReportFailsafeAndMissingBatteryMessages)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  auto options = make_single_mapping_options("diag_failsafe_joy");
  options.append_parameter_override("failsafe_timeout_ms", 50);
  options.append_parameter_override("monitor_period_ms", 20);
  options.append_parameter_override("diagnostic_updater.period", 0.1);

  auto node = std::make_shared<CRSFJoyPublisher>(std::make_unique<FakePort>(), options);
  auto helper_node = std::make_shared<rclcpp::Node>("diag_failsafe_helper");

  DiagCollector diag;
  auto diag_sub = helper_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { diag(msg); });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper_node);

  const bool ok = spin_until(
    executor,
    [&]() { return diag.latest.count("RC link") && diag.latest.count("Battery telemetry"); },
    std::chrono::milliseconds(3000));
  ASSERT_TRUE(ok);

  using Status = diagnostic_msgs::msg::DiagnosticStatus;
  EXPECT_EQ(diag.latest["RC link"].level, Status::ERROR);
  EXPECT_EQ(diag.latest["RC link"].message, "Failsafe: no RC input yet");
  EXPECT_EQ(diag.latest["Battery telemetry"].level, Status::WARN);
  EXPECT_EQ(diag.latest["Battery telemetry"].message, "No battery message received yet");

  executor.remove_node(helper_node);
  executor.remove_node(node);
  (void)diag_sub;
  rclcpp::shutdown();
}

}  // namespace elrs_joy_crsf_node
