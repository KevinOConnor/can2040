#!/usr/bin/env python
# Tool for testing canbus crc implementations
#
# Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys

def report(msg):
    sys.stderr.write(msg + "\n")

# Basic bit-by-bit canbus crc implementation
def crc_bitloop(data):
    crc = 0
    l = len(data) - 1
    val = sum([v << ((l-i)*8) for i, v in enumerate(data)])
    count = len(data) * 8
    for i in range(count-1, -1, -1):
        bit = (val >> i) & 1
        if ((crc >> 14) & 1) ^ bit:
            crc = (crc << 1) ^ 0x4599
        else:
            crc = crc << 1
    return crc & 0x7fff

# Simple bit manipulation improvement
def crc_bitloop2(data):
    crc = 0
    l = len(data) - 1
    val = sum([v << ((l-i)*8) for i, v in enumerate(data)])
    count = len(data) * 8
    for i in range(count-1, -1, -1):
        bit = (val >> i) << 15
        crc <<= 1
        if (crc ^ bit) & (1 << 15):
            crc ^= 0x4599
    return crc & 0x7fff

# Byte at a time version
def crc_byteloop(data):
    crc = 0
    for v in data:
        crc ^= v << 7
        for i in range(7, -1, -1):
            crc <<= 1
            if crc & 0x8000:
                crc ^= 0x4599
    return crc & 0x7fff

# A 4-bit at a time table lookup version
def crc_4bit_table(table, data):
    crc = 0
    l = len(data) - 1
    val = sum([v << ((l-i)*8) for i, v in enumerate(data)])
    count = len(data) * 8
    for i in range(count-4, -4, -4):
        pos = ((crc >> 11) ^ (val >> i)) & 0x0f
        crc = (crc << 4) ^ table[pos]
    return crc & 0x7fff

# A 8-bit at a time table lookup version
def crc_8bit_table(table, data):
    crc = 0
    for v in data:
        crc = (crc << 8) ^ table[((crc >> 7) ^ v) & 0xff]
    return crc & 0x7fff

TESTDATA = b"Some string of data"

def main():
    # Build 4bit table version
    bit4_table = [crc_bitloop([i]) for i in range(16)]
    print("Table 4bit:", ", ".join(["0x%04x" % i for i in bit4_table]))
    # Build 8bit table version
    bit8_table = [crc_bitloop([i]) for i in range(256)]
    print("Table 8bit:", ",".join(["0x%04x" % i for i in bit8_table]))
    # Test versions
    for i in range(len(TESTDATA)):
        d = TESTDATA[i:]
        crc1 = crc_bitloop(d)
        crc2 = crc_8bit_table(bit8_table, d)
        if crc1 != crc2:
            report("Got mismatch on '%s' %02x vs %02x" % (d, crc1, crc2))
            sys.exit(-1)
    report("Test completed successfully")

if __name__ == '__main__':
    main()
