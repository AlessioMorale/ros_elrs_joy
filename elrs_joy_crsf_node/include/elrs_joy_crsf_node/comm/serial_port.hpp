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

#ifndef ELRS_JOY_CRSF_NODE__COMM__SERIAL_PORT_HPP_
#define ELRS_JOY_CRSF_NODE__COMM__SERIAL_PORT_HPP_

#include <deque>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "boost/asio.hpp"
#include "port.hpp"
namespace elrs_joy_crsf_node::comm
{

class SerialPort : public Port
{
public:
  SerialPort()
  : io_context_(),
    work_guard_(boost::asio::make_work_guard(io_context_)),
    serial_port_(io_context_),
    read_buffer_(1024),
    tx_strand_(boost::asio::make_strand(io_context_)),
    rx_strand_(boost::asio::make_strand(io_context_))
  {
    // Start the IO thread
    io_thread_ = std::thread([this]() {
      try {
        io_context_.run();
      } catch (const std::exception & e) {
        // Log error or handle exception
        std::cerr << "IO Context exception: " << e.what() << std::endl;
      }
    });
  }

  virtual ~SerialPort()
  {
    work_guard_.reset();  // Allow io_context to complete
    io_context_.stop();   // Stop io_context

    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }
  bool open(const std::string & port, unsigned int baud_rate)
  {
    try {
      serial_port_.open(port);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(
        boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      serial_port_.set_option(
        boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

      start_receive();
      return true;
    } catch (const boost::system::system_error & e) {
      return false;
    }
  }

  void close() override
  {
    // Use both strands to ensure all operations complete before closing
    boost::asio::post(tx_strand_, [this]() {
      boost::asio::post(rx_strand_, [this]() {
        if (serial_port_.is_open()) {
          boost::system::error_code ec;
          serial_port_.close(ec);
        }
      });
    });
  }

  void set_receive_callback(receive_callback_t callback) override
  {
    boost::asio::post(rx_strand_, [this, callback = std::move(callback)]() {
      receive_callback_ = std::move(callback);
    });
  }

  void send(const std::vector<uint8_t> & data) override
  {
    boost::asio::post(tx_strand_, [this, data = std::move(data)]() mutable {
      bool write_in_progress = !write_queue_.empty();
      write_queue_.push_back(std::move(data));

      if (!write_in_progress) {
        start_send();
      }
    });
  }

private:
  void start_receive()
  {
    serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_),
      boost::asio::bind_executor(
        rx_strand_, [this](const boost::system::error_code & error, std::size_t bytes_transferred) {
          if (!error) {
            if (receive_callback_) {
              receive_callback_(std::vector<uint8_t>(
                read_buffer_.begin(), read_buffer_.begin() + bytes_transferred));
            }
            start_receive();
          }
        }));
  }

  void start_send()
  {
    if (write_queue_.empty()) {
      return;
    }

    boost::asio::async_write(
      serial_port_, boost::asio::buffer(write_queue_.front()),
      boost::asio::bind_executor(
        tx_strand_,
        [this](const boost::system::error_code & error, std::size_t /*bytes_transferred*/) {
          if (!error) {
            write_queue_.pop_front();
            if (!write_queue_.empty()) {
              start_send();
            }
          }
        }));
  }
  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  boost::asio::serial_port serial_port_;
  std::vector<uint8_t> read_buffer_;
  boost::asio::strand<boost::asio::io_context::executor_type> tx_strand_;  // Transmit strand
  boost::asio::strand<boost::asio::io_context::executor_type> rx_strand_;  // Receive strand
  std::deque<std::vector<uint8_t>> write_queue_;
  receive_callback_t receive_callback_;
  std::thread io_thread_;
};
};  // namespace elrs_joy_crsf_node::comm
#endif  // ELRS_JOY_CRSF_NODE__COMM__SERIAL_PORT_HPP_
