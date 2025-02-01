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

#include <chrono>
#include <memory>

#include "elrs_joy_crsf_node/comm/serial_port.hpp"
#include "elrs_joy_crsf_node/crsf_joy_node.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elrs_joy_crsf_node::CRSFJoyPublisher>(
    std::make_unique<elrs_joy_crsf_node::comm::SerialPort>()));
  rclcpp::shutdown();
  return 0;
}
