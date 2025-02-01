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

#ifndef ELRS_JOY_CRSF_NODE__DIAGNOSTIC_HPP_
#define ELRS_JOY_CRSF_NODE__DIAGNOSTIC_HPP_

#include <memory>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "rclcpp/node.hpp"
class Diagnostic
{
public:
  explicit Diagnostic(std::shared_ptr<rclcpp::Node> node) : updater_(node)
  {
    updater_.setHardwareID("elrs_joy_crsf_node");
    updater_.add("Diagnostic Status", this, &Diagnostic::produceDiagnostics);
  }

private:
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    // Example diagnostic message
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All systems are operational.");
    stat.add("Example Key", "Example Value");
  }

  diagnostic_updater::Updater updater_;
};

#endif  // ELRS_JOY_CRSF_NODE__DIAGNOSTIC_HPP_
