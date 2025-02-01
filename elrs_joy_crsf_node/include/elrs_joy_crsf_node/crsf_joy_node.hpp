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
#include <memory>
#include <utility>

#include "elrs_joy_crsf_node/comm/port.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::literals::chrono_literals;  // NOLINT
namespace elrs_joy_crsf_node
{
class CRSFJoyPublisher : public rclcpp::Node
{
private:
  constexpr static const char * const NODE_NAME = "crsf_joy_node";
  constexpr static const char * const TOPIC_NAME = "crsf_joy";

public:
  explicit CRSFJoyPublisher(std::unique_ptr<comm::Port> port)
  : Node(NODE_NAME), port_(std::move(port)), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(TOPIC_NAME, 10);
    auto monitor_timer_callback = [this]() -> void { this->handle_monitor(); };
    auto channels_timer_callback = [this]() -> void { this->handle_channels(); };
    monitor_timer_ = this->create_wall_timer(500ms, monitor_timer_callback);
    channels_timer_ = this->create_wall_timer(50ms, channels_timer_callback);
  }

private:
  void handle_monitor() { RCLCPP_INFO(this->get_logger(), "Monitor timer callback"); }

  void handle_channels()
  {
    count_++;
    auto message = sensor_msgs::msg::Joy();
    message.set__buttons({0, 1, 0, 0});
    RCLCPP_INFO(this->get_logger(), "Publishing: '%zu'", this->count_);
    this->publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Channels timer callback");
  }
  std::unique_ptr<comm::Port> port_;
  rclcpp::TimerBase::SharedPtr channels_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  std::atomic<bool> channels_updated_{false};
  std::atomic<bool> failsafe_active_{true};
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  size_t count_;
};
}  // namespace elrs_joy_crsf_node
#endif  // ELRS_JOY_CRSF_NODE__CRSF_JOY_NODE_HPP_
