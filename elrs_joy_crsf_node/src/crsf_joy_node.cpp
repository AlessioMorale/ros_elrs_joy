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

#include "elrs_joy_crsf_node/crsf_joy_node.hpp"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>

#include "elrs_joy_crsf_node/comm/serial_port.hpp"
#include "elrs_joy_crsf_protocol/crsf/messages.hpp"

using namespace std::literals::chrono_literals;  // NOLINT

namespace elrs_joy_crsf_node
{
CRSFJoyPublisher::CRSFJoyPublisher(
  std::unique_ptr<comm::Port> port, const rclcpp::NodeOptions & options)
: Node(NODE_NAME, options), port_(std::move(port)), packets_parser_(), latest_channels_()
{
  declare_and_load_parameters();
  validate_and_build_mappings();

  publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);

  if (telemetry_battery_enabled_) {
    battery_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_, 10,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { this->on_battery_state(msg); });
  }

  packets_parser_.set_callback([this](const elrs_joy_crsf_protocol::crsf::Message::Frame & frame) {
    this->on_parsed_frame(frame);
  });

  if (port_) {
    port_->set_receive_callback(
      [this](const std::vector<uint8_t> & data) { this->on_serial_data(data); });
    configure_serial_if_supported();
  }

  last_input_time_ = this->now();

  monitor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(monitor_period_ms_)),
    [this]() { this->handle_monitor(); });
  channels_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(joy_publish_period_ms_)),
    [this]() { this->handle_channels(); });

  RCLCPP_INFO(this->get_logger(), "CRSF joy node started");
}

void CRSFJoyPublisher::declare_and_load_parameters()
{
  joy_topic_ = this->declare_parameter<std::string>("joy_topic", "crsf_joy");
  battery_topic_ = this->declare_parameter<std::string>("battery_topic", "battery_state");
  serial_port_name_ = this->declare_parameter<std::string>("serial.port", "");
  serial_baudrate_ = this->declare_parameter<int64_t>("serial.baudrate", 416666);
  serial_timeout_ms_ = this->declare_parameter<int64_t>("serial.timeout_ms", 100);

  joy_publish_period_ms_ = this->declare_parameter<int64_t>("joy_publish_period_ms", 50);
  monitor_period_ms_ = this->declare_parameter<int64_t>("monitor_period_ms", 100);
  failsafe_timeout_ms_ = this->declare_parameter<int64_t>("failsafe_timeout_ms", 500);
  send_failsafe_continuously_ = this->declare_parameter<bool>("send_failsafe_continuously", true);
  telemetry_battery_enabled_ = this->declare_parameter<bool>("telemetry_battery_enabled", true);

  auto axis_channels =
    this->declare_parameter<std::vector<int64_t>>("axis_mappings.channels", {0, 1, 2, 3});
  auto axis_scale =
    this->declare_parameter<std::vector<double>>("axis_mappings.scale", {1.0, 1.0, 1.0, 1.0});
  auto axis_offset =
    this->declare_parameter<std::vector<double>>("axis_mappings.offset", {0.0, 0.0, 0.0, 0.0});
  auto axis_invert = this->declare_parameter<std::vector<bool>>(
    "axis_mappings.invert", {false, false, false, false});
  auto axis_deadzone =
    this->declare_parameter<std::vector<double>>("axis_mappings.deadzone", {0.0, 0.0, 0.0, 0.0});
  auto axis_min =
    this->declare_parameter<std::vector<double>>("axis_mappings.min", {-1.0, -1.0, -1.0, -1.0});
  auto axis_max =
    this->declare_parameter<std::vector<double>>("axis_mappings.max", {1.0, 1.0, 1.0, 1.0});

  auto button_channels =
    this->declare_parameter<std::vector<int64_t>>("button_mappings.channels", {4, 5, 6, 7});
  auto button_threshold =
    this->declare_parameter<std::vector<double>>("button_mappings.threshold", {0.5, 0.5, 0.5, 0.5});
  auto button_invert = this->declare_parameter<std::vector<bool>>(
    "button_mappings.invert", {false, false, false, false});

  failsafe_axes_ = this->declare_parameter<std::vector<double>>(
    "failsafe_axes", std::vector<double>(axis_channels.size(), 0.0));
  failsafe_buttons_raw_ = this->declare_parameter<std::vector<int64_t>>(
    "failsafe_buttons", std::vector<int64_t>(button_channels.size(), 0));

  const size_t axis_count = axis_channels.size();
  if (
    axis_scale.size() != axis_count || axis_offset.size() != axis_count ||
    axis_invert.size() != axis_count || axis_deadzone.size() != axis_count ||
    axis_min.size() != axis_count || axis_max.size() != axis_count) {
    RCLCPP_WARN(
      this->get_logger(),
      "Axis mapping array lengths mismatch; reverting to default 4-axis mapping.");
    axis_channels = {0, 1, 2, 3};
    axis_scale = {1.0, 1.0, 1.0, 1.0};
    axis_offset = {0.0, 0.0, 0.0, 0.0};
    axis_invert = {false, false, false, false};
    axis_deadzone = {0.0, 0.0, 0.0, 0.0};
    axis_min = {-1.0, -1.0, -1.0, -1.0};
    axis_max = {1.0, 1.0, 1.0, 1.0};
  }

  const size_t button_count = button_channels.size();
  if (button_threshold.size() != button_count || button_invert.size() != button_count) {
    RCLCPP_WARN(
      this->get_logger(),
      "Button mapping array lengths mismatch; reverting to default 4-button mapping.");
    button_channels = {4, 5, 6, 7};
    button_threshold = {0.5, 0.5, 0.5, 0.5};
    button_invert = {false, false, false, false};
  }

  axis_mappings_.clear();
  axis_mappings_.reserve(axis_channels.size());
  for (size_t i = 0; i < axis_channels.size(); ++i) {
    axis_mappings_.push_back(AxisMapping{
      static_cast<int>(axis_channels[i]), axis_scale[i], axis_offset[i], axis_invert[i],
      axis_deadzone[i], axis_min[i], axis_max[i]});
  }

  button_mappings_.clear();
  button_mappings_.reserve(button_channels.size());
  for (size_t i = 0; i < button_channels.size(); ++i) {
    button_mappings_.push_back(
      ButtonMapping{static_cast<int>(button_channels[i]), button_threshold[i], button_invert[i]});
  }
}

bool CRSFJoyPublisher::validate_and_build_mappings()
{
  bool valid = true;

  for (const auto & mapping : axis_mappings_) {
    if (mapping.channel_index < 0 || mapping.channel_index > 15) {
      RCLCPP_ERROR(
        this->get_logger(), "Axis mapping channel index out of range: %d", mapping.channel_index);
      valid = false;
    }
    if (
      !std::isfinite(mapping.scale) || !std::isfinite(mapping.offset) ||
      !std::isfinite(mapping.deadzone) || !std::isfinite(mapping.min) ||
      !std::isfinite(mapping.max)) {
      RCLCPP_ERROR(this->get_logger(), "Axis mapping contains non-finite values.");
      valid = false;
    }
  }

  for (const auto & mapping : button_mappings_) {
    if (mapping.channel_index < 0 || mapping.channel_index > 15) {
      RCLCPP_ERROR(
        this->get_logger(), "Button mapping channel index out of range: %d", mapping.channel_index);
      valid = false;
    }
    if (!std::isfinite(mapping.threshold) || mapping.threshold < 0.0 || mapping.threshold > 1.0) {
      RCLCPP_ERROR(
        this->get_logger(), "Button threshold out of range [0,1]: %.3f", mapping.threshold);
      valid = false;
    }
  }

  if (!valid) {
    RCLCPP_WARN(this->get_logger(), "Invalid mapping config detected; using safe empty mapping.");
    axis_mappings_.clear();
    button_mappings_.clear();
    failsafe_axes_.clear();
    failsafe_buttons_raw_.clear();
  }

  if (failsafe_axes_.size() != axis_mappings_.size()) {
    failsafe_axes_.assign(axis_mappings_.size(), 0.0);
  }
  if (failsafe_buttons_raw_.size() != button_mappings_.size()) {
    failsafe_buttons_raw_.assign(button_mappings_.size(), 0);
  }

  return valid;
}

void CRSFJoyPublisher::configure_serial_if_supported()
{
  auto * serial_port = dynamic_cast<comm::SerialPort *>(port_.get());
  if (serial_port == nullptr) {
    return;
  }

  if (serial_port_name_.empty()) {
    RCLCPP_WARN(this->get_logger(), "serial.port parameter is empty, serial input disabled.");
    return;
  }

  const bool opened =
    serial_port->open(serial_port_name_, static_cast<unsigned int>(serial_baudrate_));
  if (!opened) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open serial port '%s' at %" PRIu64 " baud.",
      serial_port_name_.c_str(), serial_baudrate_);
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Opened serial port '%s' at %" PRIu64 " baud.", serial_port_name_.c_str(),
      serial_baudrate_);
  }
}

void CRSFJoyPublisher::on_serial_data(const std::vector<uint8_t> & data)
{
  for (const uint8_t byte : data) {
    packets_parser_.process_byte(byte);
  }
}

void CRSFJoyPublisher::on_parsed_frame(const elrs_joy_crsf_protocol::crsf::Message::Frame & frame)
{
  auto channels_message = elrs_joy_crsf_protocol::crsf::RCChannelsMessage::from_frame(frame);
  if (!channels_message.has_value()) {
    return;
  }

  {
    std::scoped_lock<std::mutex> lock(channels_mutex_);
    latest_channels_ = channels_message->payload;
  }

  channels_updated_.store(true);
  last_input_time_ = this->now();

  if (failsafe_active_.exchange(false)) {
    failsafe_message_sent_.store(false);
    RCLCPP_WARN(this->get_logger(), "RC input restored, leaving failsafe.");
  }
}

void CRSFJoyPublisher::on_battery_state(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (port_ == nullptr || !telemetry_battery_enabled_) {
    return;
  }

  const float voltage = std::isfinite(msg->voltage) ? msg->voltage : 0.0F;
  const float current = std::isfinite(msg->current) ? msg->current : 0.0F;

  int32_t used_capacity_mah = 0;
  if (std::isfinite(msg->capacity) && std::isfinite(msg->charge) && msg->capacity > 0.0F) {
    const float used_capacity_ah = std::max(0.0F, msg->capacity - msg->charge);
    used_capacity_mah = static_cast<int32_t>(std::lround(used_capacity_ah * 1000.0F));
  }

  uint8_t battery_percent = 0;
  if (std::isfinite(msg->percentage)) {
    const double pct = clamp(static_cast<double>(msg->percentage), 0.0, 1.0);
    battery_percent = static_cast<uint8_t>(std::lround(pct * 100.0));
  }

  elrs_joy_crsf_protocol::crsf::BatterySensorMessage battery_message;
  battery_message.payload = {
    .voltage = voltage,
    .current = current,
    .usedCapacity = used_capacity_mah,
    .batteryPercent = battery_percent,
  };

  const auto frame = battery_message.to_frame().data;
  if (!frame.empty()) {
    port_->send(frame);
  }
}

void CRSFJoyPublisher::handle_monitor()
{
  const auto elapsed = (this->now() - last_input_time_).nanoseconds();
  const auto timeout_ns = failsafe_timeout_ms_ * 1000000LL;

  if (elapsed > timeout_ns) {
    if (!failsafe_active_.exchange(true)) {
      failsafe_message_sent_.store(false);
      RCLCPP_WARN(
        this->get_logger(), "Entering failsafe: no RC input for %" PRId64 " ms.",
        failsafe_timeout_ms_);
    }
  }
}

void CRSFJoyPublisher::handle_channels()
{
  if (failsafe_active_.load()) {
    if (send_failsafe_continuously_ || !failsafe_message_sent_.load()) {
      publisher_->publish(build_failsafe_joy_message());
      failsafe_message_sent_.store(true);
    }
    return;
  }

  publisher_->publish(build_mapped_joy_message());
}

sensor_msgs::msg::Joy CRSFJoyPublisher::build_mapped_joy_message()
{
  sensor_msgs::msg::Joy message;
  message.header.stamp = this->now();

  elrs_joy_crsf_protocol::crsf::RCChannelsPayload channels{};
  {
    std::scoped_lock<std::mutex> lock(channels_mutex_);
    channels = latest_channels_;
  }

  message.axes.resize(axis_mappings_.size(), 0.0F);
  for (size_t i = 0; i < axis_mappings_.size(); ++i) {
    const auto & mapping = axis_mappings_[i];
    const auto channel = channels.channels[static_cast<size_t>(mapping.channel_index)];
    message.axes[i] = static_cast<float>(channel_to_axis_value(channel, mapping));
  }

  message.buttons.resize(button_mappings_.size(), 0);
  for (size_t i = 0; i < button_mappings_.size(); ++i) {
    const auto & mapping = button_mappings_[i];
    const auto channel = channels.channels[static_cast<size_t>(mapping.channel_index)];
    message.buttons[i] = channel_to_button_value(channel, mapping);
  }

  return message;
}

sensor_msgs::msg::Joy CRSFJoyPublisher::build_failsafe_joy_message() const
{
  sensor_msgs::msg::Joy message;
  message.axes.resize(failsafe_axes_.size(), 0.0F);
  for (size_t i = 0; i < failsafe_axes_.size(); ++i) {
    message.axes[i] = static_cast<float>(failsafe_axes_[i]);
  }

  message.buttons.resize(failsafe_buttons_raw_.size(), 0);
  for (size_t i = 0; i < failsafe_buttons_raw_.size(); ++i) {
    message.buttons[i] = failsafe_buttons_raw_[i] == 0 ? 0 : 1;
  }

  return message;
}

double CRSFJoyPublisher::channel_to_axis_value(
  uint16_t channel_us, const AxisMapping & mapping) const
{
  double normalized = (static_cast<double>(channel_us) - 1500.0) / 500.0;
  normalized = clamp(normalized, -1.0, 1.0);

  if (mapping.invert) {
    normalized *= -1.0;
  }

  if (std::abs(normalized) < mapping.deadzone) {
    normalized = 0.0;
  }

  const double value = normalized * mapping.scale + mapping.offset;
  return clamp(value, mapping.min, mapping.max);
}

int CRSFJoyPublisher::channel_to_button_value(
  uint16_t channel_us, const ButtonMapping & mapping) const
{
  double normalized = (static_cast<double>(channel_us) - 1000.0) / 1000.0;
  normalized = clamp(normalized, 0.0, 1.0);

  bool state = normalized >= mapping.threshold;
  if (mapping.invert) {
    state = !state;
  }

  return state ? 1 : 0;
}

double CRSFJoyPublisher::clamp(double value, double min_value, double max_value)
{
  return std::clamp(value, min_value, max_value);
}

}  // namespace elrs_joy_crsf_node
