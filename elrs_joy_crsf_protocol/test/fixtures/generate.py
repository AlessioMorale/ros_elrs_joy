#!/usr/bin/env python3
# Copyright 2026 Alessio Morale <alessiomorale-at-gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# SPDX-FileCopyrightText: 2026 Alessio Morale <alessiomorale-at-gmail.com>
# SPDX-License-Identifier: mit
"""
Generates the synthetic golden CRSF frames in this directory.

The frames are built from the CRSF spec (.docs/crsf.md) and the ExpressLRS/EdgeTX framing,
independently of the C++ library, so the fixture tests check the library against the spec.
Replace a file with a hardware capture (same name, `# source: hardware ...`) when one is
available; the tests only read the hex bytes.

Usage: python3 generate.py   (rewrites the *.hex files next to this script)
"""
import pathlib
import struct

HERE = pathlib.Path(__file__).resolve().parent

SYNC_FC = 0xC8
SYNC_HANDSET = 0xEA
SYNC_TX = 0xEE
ADDR_BROADCAST = 0x00
ADDR_HANDSET = 0xEA
ADDR_TX = 0xEE
ADDR_LUA = 0xEF


def crc8_d5(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(sync: int, ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype]) + payload
    return bytes([sync, len(body) + 1]) + body + bytes([crc8_d5(body)])


def us_to_ticks(us: int) -> int:
    return (us - 1500) * 8 // 5 + 992


def pack_channels(channels_us):
    bits = 0
    for i, us in enumerate(channels_us):
        bits |= (us_to_ticks(us) & 0x7FF) << (11 * i)
    return bits.to_bytes(22, "little")


def cstr(s: str) -> bytes:
    return s.encode("ascii") + b"\0"


def param_entry(sync, dest, origin, number, remaining, data):
    return frame(sync, 0x2B, bytes([dest, origin, number, remaining]) + data)


def write(name, description, data: bytes):
    lines = [f"# {description}", "# source: synthetic (generate.py)"]
    hexbytes = [f"{b:02x}" for b in data]
    for i in range(0, len(hexbytes), 16):
        lines.append(" ".join(hexbytes[i : i + 16]))
    (HERE / f"{name}.hex").write_text("\n".join(lines) + "\n")


def main():
    # RC channels from the handset to the TX module: AETR + AUX1 high, AUX2 low.
    # Values are 1500 + 5k us so they survive the us -> ticks -> us round trip exactly.
    channels = [1500, 2000, 1000, 1500, 2000, 1000] + [1500] * 10
    write(
        "rc_channels_to_tx",
        "RC_CHANNELS_PACKED to TX module; us: " + ",".join(map(str, channels)),
        frame(SYNC_TX, 0x16, pack_channels(channels)),
    )

    # Link statistics from the TX module: RSSI -64/-70 dBm, LQ 100, SNR 9, 100 mW, downlink -66/97/8
    write(
        "link_statistics",
        "LINK_STATISTICS from TX module; rssi1=64 rssi2=70 lq=100 snr=9 "
        "ant=0 rf_mode=7 power=3(100mW) d_rssi=66 d_lq=97 d_snr=8",
        frame(SYNC_HANDSET, 0x14, struct.pack(">BBBbBBBBBb", 64, 70, 100, 9, 0, 7, 3, 66, 97, 8)),
    )

    # Robot battery relayed by the TX module: 15.6 V, 1.2 A, 450 mAh, 72 %
    write(
        "battery_sensor",
        "BATTERY_SENSOR from TX module; 15.6 V, 1.2 A, 450 mAh, 72 %",
        frame(
            SYNC_HANDSET,
            0x08,
            struct.pack(">Hh", 156, 12) + (450).to_bytes(3, "big") + bytes([72]),
        ),
    )

    write(
        "flight_mode",
        "FLIGHT_MODE from TX module; robot status 'RDY'",
        frame(SYNC_HANDSET, 0x21, cstr("RDY")),
    )
    write(
        "flight_mode_fault",
        "FLIGHT_MODE from TX module; robot status 'FLT:MOTOR_L'",
        frame(SYNC_HANDSET, 0x21, cstr("FLT:MOTOR_L")),
    )
    write(
        "flight_mode_from_fc",
        "FLIGHT_MODE sent by the robot to the RX (sync 0xC8); 'WRN:TEMP'",
        frame(SYNC_FC, 0x21, cstr("WRN:TEMP")),
    )

    # Timing correction: RADIO_ID (0x3A) sub-type 0x10, 4 ms interval, frames 120 us late
    write(
        "opentx_sync",
        "OPENTX_SYNC (RADIO_ID 0x3A / 0x10); interval 40000 (4 ms), offset -1200 (-120 us)",
        frame(
            SYNC_HANDSET,
            0x3A,
            bytes([ADDR_HANDSET, ADDR_TX, 0x10]) + struct.pack(">Ii", 40000, -1200),
        ),
    )

    write(
        "device_ping",
        "DEVICE_PING broadcast from the handset",
        frame(SYNC_TX, 0x28, bytes([ADDR_BROADCAST, ADDR_HANDSET])),
    )

    write(
        "device_info",
        "DEVICE_INFO from TX module; 'ELRS TX 2400', serial 'ELRS', fw 3.5.3, 25 params",
        frame(
            SYNC_HANDSET,
            0x29,
            bytes([ADDR_HANDSET, ADDR_TX])
            + cstr("ELRS TX 2400")
            + struct.pack(">IIIBB", 0x454C5253, 0, 0x00030503, 25, 0),
        ),
    )

    # Single-chunk TEXT_SELECTION: "Max Power", options in mW, value 3 (100), unit mW
    max_power = (
        bytes([4, 0x09])
        + cstr("Max Power")
        + cstr("10;25;50;100;250")
        + bytes([3, 0, 4, 3])
        + cstr("mW")
    )
    write(
        "parameter_entry_single",
        "PARAMETER_SETTINGS_ENTRY param 5 'Max Power' TEXT_SELECTION, 1 chunk",
        param_entry(SYNC_HANDSET, ADDR_LUA, ADDR_TX, 5, 0, max_power),
    )

    # Multi-chunk TEXT_SELECTION: "Packet Rate" with a long option list (3 chunks of <= 56 bytes)
    packet_rate = (
        bytes([0, 0x09])
        + cstr("Packet Rate")
        + cstr(
            "50Hz(-115dBm);100Hz Full(-112dBm);150Hz(-112dBm);250Hz(-108dBm);"
            "333Hz Full(-105dBm);500Hz(-105dBm)"
        )
        + bytes([3, 0, 5, 3])
        + cstr("")
    )
    chunk_size = 56
    chunks = [packet_rate[i : i + chunk_size] for i in range(0, len(packet_rate), chunk_size)]
    assert len(chunks) == 3, len(chunks)
    for idx, chunk in enumerate(chunks):
        write(
            f"parameter_entry_chunk{idx}",
            f"PARAMETER_SETTINGS_ENTRY param 1 'Packet Rate' TEXT_SELECTION, chunk {idx} of {len(chunks)}",
            param_entry(SYNC_HANDSET, ADDR_LUA, ADDR_TX, 1, len(chunks) - 1 - idx, chunk),
        )

    write(
        "parameter_read",
        "PARAMETER_READ param 1 chunk 0, Lua origin to TX module",
        frame(SYNC_TX, 0x2C, bytes([ADDR_TX, ADDR_LUA, 1, 0])),
    )
    write(
        "parameter_write",
        "PARAMETER_WRITE param 5 = 3 (Max Power 100 mW), Lua origin to TX module",
        frame(SYNC_TX, 0x2D, bytes([ADDR_TX, ADDR_LUA, 5, 3])),
    )


if __name__ == "__main__":
    main()
