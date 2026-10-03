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

#include "elrs_joy_crsf_node/comm/custom_baud.hpp"

#include <asm/termbits.h>
#include <sys/ioctl.h>

namespace elrs_joy_crsf_node::comm
{
bool set_custom_baud_rate(int fd, unsigned int baud_rate)
{
  struct termios2 tio{};
  if (ioctl(fd, TCGETS2, &tio) != 0) {
    return false;
  }
  tio.c_cflag &= ~static_cast<tcflag_t>(CBAUD);
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baud_rate;
  tio.c_ospeed = baud_rate;
  return ioctl(fd, TCSETS2, &tio) == 0;
}
}  // namespace elrs_joy_crsf_node::comm
