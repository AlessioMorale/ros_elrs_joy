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

#ifndef ELRS_JOY_CRSF_NODE_JOYSTICK_CONTROLLER_HPP_
#define ELRS_JOY_CRSF_NODE_JOYSTICK_CONTROLLER_HPP_

// joystick_controller.hpp
#pragma once

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick_controller
{

class JoystickController : public rclcpp::Node
{
private:
  static constexpr const char * JOY_TOPIC_NAME = "joy";

public:
  explicit JoystickController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("joystick_controller", options)
  {
    // Declare and get parameters
    auto publish_rate = this->declare_parameter("publish_rate", 20.0);
    failsafe_timeout_sec_ = this->declare_parameter("failsafe_timeout", 0.5);

    // Get failsafe values
    auto failsafe_axes = this->declare_parameter("failsafe_axes", std::vector<double>{});
    auto failsafe_buttons = this->declare_parameter("failsafe_buttons", std::vector<bool>{});

    // Convert failsafe axes from double to float
    failsafe_axes_.resize(failsafe_axes.size());
    for (size_t i = 0; i < failsafe_axes.size(); ++i) {
      failsafe_axes_[i] = static_cast<float>(failsafe_axes[i]);
    }
    failsafe_buttons_ = failsafe_buttons;

    // Create publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Joy>(JOY_TOPIC_NAME, 10);

    // Set up timer for publishing at autorepeat_rate
    double pub_rate = publish_rate;
    if (pub_rate > 0) {
      timer_interval_ = std::chrono::duration<double>(1.0 / pub_rate);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_interval_),
        [this] { publish_joy(); });
    }

    // Initialize diagnostic updater
    updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    updater_->setHardwareID("joystick_controller");
    updater_->add("Joystick Status", this, &JoystickController::check_joystick_status);
    updater_->add("Joystick Statistics", this, &JoystickController::report_statistics);

    double min_freq = pub_rate * 0.8;  // If you update these values, the
    double max_freq = pub_rate * 1.2;  // HeaderlessTopicDiagnostic will use the new values.
    diagnostic_updater::HeaderlessTopicDiagnostic pub_freq_(
      JOY_TOPIC_NAME, *updater_.get(),
      diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));

    diagnostic_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),  // Update diagnostics every second
      [this] { update_diagnostics(); });

    RCLCPP_INFO(
      this->get_logger(),
      "Started joystick controller with publishing rate %.2f Hz and failsafe timeout %.2f seconds",
      pub_rate, failsafe_timeout_sec_);
  }

  // Public interface function to update joystick inputs
  void update_joystick_inputs(std::vector<float> & axes, std::vector<bool> & buttons)
  {
    std::lock_guard<std::mutex> guard(joy_mutex_);

    auto current_time = this->now();

    // Update the joy message
    joy_msg_.header.stamp = current_time;
    joy_msg_.axes = axes;
    joy_msg_.buttons.clear();
    joy_msg_.buttons.reserve(buttons.size());

    // Convert from bool to uint8

    for (const auto button : buttons) {
      joy_msg_.buttons.push_back(button ? 1 : 0);
    }

    // Update timing statistics
    if (last_update_time_.nanoseconds() > 0) {
      double dt = (current_time - last_update_time_).seconds();

      // Update statistics
      min_interval_ = std::min(min_interval_, dt);
      max_interval_ = std::max(max_interval_, dt);
      auto received_count = static_cast<double>(received_joy_count_);
      avg_interval_ = (avg_interval_ * received_count + dt) / (received_count + 1);
    }

    // Update last received time
    last_update_time_ = current_time;

    // Reset failsafe flag as we've received fresh data
    failsafe_triggered_ = false;

    // Increment counter
    received_joy_count_++;
  }

private:
  void publish_joy()
  {
    auto current_time = this->now();

    std::lock_guard<std::mutex> guard(joy_mutex_);

    // Check if we haven't received data for longer than failsafe_timeout
    if (
      !failsafe_triggered_ &&
      (current_time - last_update_time_).seconds() > failsafe_timeout_sec_) {
      // Set the failsafe flag to true - we'll only send failsafe values once
      failsafe_triggered_ = true;

      if (!failsafe_axes_.empty() || !failsafe_buttons_.empty()) {
        // Prepare failsafe message
        sensor_msgs::msg::Joy failsafe_msg;
        failsafe_msg.header.stamp = current_time;
        failsafe_msg.axes = failsafe_axes_;

        // Convert bool buttons to uint8
        failsafe_msg.buttons.clear();
        failsafe_msg.buttons.reserve(failsafe_buttons_.size());
        for (const auto button : failsafe_buttons_) {
          failsafe_msg.buttons.push_back(button ? 1 : 0);
        }

        // Publish the failsafe message
        pub_->publish(failsafe_msg);

        RCLCPP_WARN(
          this->get_logger(),
          "Joystick input timed out after %.2f seconds. Published failsafe values.",
          failsafe_timeout_sec_);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Joystick input timed out after %.2f seconds. No failsafe values configured.",
          failsafe_timeout_sec_);
      }

      return;
    }

    // Don't publish if failsafe has been triggered (we only publish failsafe once)
    if (failsafe_triggered_) {
      return;
    }

    // Publish the latest joy message
    pub_->publish(joy_msg_);
    publish_joy_count_++;
  }

  // Diagnostic callbacks
  void check_joystick_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> guard(joy_mutex_);

    auto current_time = this->now();
    double time_since_update = (current_time - last_update_time_).seconds();

    if (failsafe_triggered_) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Joystick failsafe triggered. No joystick data received for " +
          std::to_string(time_since_update) + " seconds.");
    } else if (time_since_update > (failsafe_timeout_sec_ * 0.8)) {
      // Warning if we're approaching the timeout
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "Joystick data delayed. Last update was " +
                                                        std::to_string(time_since_update) +
                                                        " seconds ago.");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Joystick controller is healthy.");
    }

    stat.add("Time since last joystick update (seconds)", time_since_update);
    stat.add("Failsafe triggered", failsafe_triggered_ ? "Yes" : "No");
    stat.add("Failsafe timeout (seconds)", failsafe_timeout_sec_);
    stat.add("Axes count", joy_msg_.axes.size());
    stat.add("Buttons count", joy_msg_.buttons.size());
  }

  void report_statistics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> guard(joy_mutex_);

    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Joystick statistics");
    stat.add("Received messages", received_joy_count_);
    stat.add("Published messages", publish_joy_count_);

    if (received_joy_count_ > 0) {
      stat.add("Minimum update interval (seconds)", min_interval_);
      stat.add("Maximum update interval (seconds)", max_interval_);
      stat.add("Average update interval (seconds)", avg_interval_);
      stat.add("Average update frequency (Hz)", avg_interval_ > 0 ? 1.0 / avg_interval_ : 0.0);
    }
  }

  void update_diagnostics() { updater_->force_update(); }

  // Publisher and timer
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::chrono::duration<double> timer_interval_;

  // Diagnostic updater
  std::unique_ptr<diagnostic_updater::Updater> updater_;
  diagnostic_updater::HeaderlessTopicDiagnostic pub_freq_;
  // Mutex to protect data access
  std::mutex joy_mutex_;

  // Joy message
  sensor_msgs::msg::Joy joy_msg_;

  // Time tracking
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double failsafe_timeout_sec_;
  bool failsafe_triggered_{false};

  // Failsafe values
  std::vector<float> failsafe_axes_;
  std::vector<bool> failsafe_buttons_;

  // Statistics
  uint64_t received_joy_count_{};
  uint64_t publish_joy_count_{};
  double min_interval_{std::numeric_limits<double>::max()};
  double max_interval_{};
  double avg_interval_{};
};

}  // namespace joystick_controller
#endif  // ELRS_JOY_CRSF_NODE_JOYSTICK_CONTROLLER_HPP_
