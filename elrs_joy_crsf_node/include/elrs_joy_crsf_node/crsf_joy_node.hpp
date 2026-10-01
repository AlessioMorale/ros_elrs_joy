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

#ifndef ELRS_JOY_CRSF_NODE__CRSF_JOY_NODE_HPP_
#define ELRS_JOY_CRSF_NODE__CRSF_JOY_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "elrs_joy_crsf_node/comm/port.hpp"
#include "elrs_joy_crsf_protocol/crsf/packets.hpp"
#include "elrs_joy_crsf_protocol/crsf/payload.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
namespace elrs_joy_crsf_node
{
class CRSFJoyPublisher : public rclcpp::Node
{
private:
  constexpr static const char * const NODE_NAME = "crsf_joy_node";

  struct AxisMapping
  {
    int channel_index;
    double scale;
    double offset;
    bool invert;
    double deadzone;
    double min;
    double max;
  };

  struct ButtonMapping
  {
    int channel_index;
    double threshold;
    bool invert;
  };

public:
  explicit CRSFJoyPublisher(
    std::unique_ptr<comm::Port> port, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_and_load_parameters();
  bool validate_and_build_mappings();
  void configure_serial_if_supported();

  void on_serial_data(const std::vector<uint8_t> & data);
  void on_parsed_frame(const elrs_joy_crsf_protocol::crsf::Message::Frame & frame);
  void on_battery_state(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  void handle_monitor();
  void handle_channels();

  sensor_msgs::msg::Joy build_mapped_joy_message();
  sensor_msgs::msg::Joy build_failsafe_joy_message() const;

  double channel_to_axis_value(uint16_t channel_us, const AxisMapping & mapping) const;
  int channel_to_button_value(uint16_t channel_us, const ButtonMapping & mapping) const;

  static double clamp(double value, double min_value, double max_value);

  std::unique_ptr<comm::Port> port_;
  elrs_joy_crsf_protocol::crsf::Packets packets_parser_;

  std::mutex channels_mutex_;
  elrs_joy_crsf_protocol::crsf::RCChannelsPayload latest_channels_{};

  rclcpp::TimerBase::SharedPtr channels_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::Time last_input_time_{0, 0, RCL_ROS_TIME};

  std::string joy_topic_;
  std::string battery_topic_;
  std::string serial_port_name_;
  int64_t serial_baudrate_{416666};
  int64_t serial_timeout_ms_{100};
  int64_t joy_publish_period_ms_{50};
  int64_t monitor_period_ms_{100};
  int64_t failsafe_timeout_ms_{500};
  bool send_failsafe_continuously_{true};
  bool telemetry_battery_enabled_{true};

  std::vector<double> failsafe_axes_;
  std::vector<int64_t> failsafe_buttons_raw_;

  std::vector<AxisMapping> axis_mappings_;
  std::vector<ButtonMapping> button_mappings_;

  std::atomic<bool> channels_updated_{false};
  std::atomic<bool> failsafe_active_{true};
  std::atomic<bool> failsafe_message_sent_{false};

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
};
}  // namespace elrs_joy_crsf_node
#endif  // ELRS_JOY_CRSF_NODE__CRSF_JOY_NODE_HPP_
