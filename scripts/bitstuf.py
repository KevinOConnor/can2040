#!/usr/bin/env python
# Tool for testing bitstuffing implementations
#
# Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys

TESTBITS=20

def report(msg):
    sys.stderr.write(msg + "\n")

def bitstuf(b, num_bits):
    edges = b ^ (b >> 1)
    count = num_bits
    i = num_bits-1
    while i >= 0:
        if not ((edges >> i) & 0xf):
            mask = (1 << (i + 1)) - 1
            low = b & mask
            high = (b & ~(mask >> 1)) << 1
            b = high ^ low ^ (1 << i)
            i -= 3
            count += 1
            edges = b ^ (b >> 1)
        i -= 1
    return b, count

def bitunstuf(sb, num_bits):
    edges = sb ^ (sb >> 1);
    ub = 0
    cu = TESTBITS
    cs = num_bits
    while 1:
        if not cu:
            return ub
        if not cs:
            report("Ran out of bits")
            return -1
        cs -= 1
        if (edges >> (cs+1)) & 0xf:
            # Normal data
            cu -= 1
            ub |= ((sb >> cs) & 1) << cu
        elif ((edges >> cs) & 0x1f) == 0x00:
            # Six consecutive bits - a bitstuff error
            report("Bitstuff error")
            if (sb >> cs) & 1:
                return -1;
            return -2;

def main():
    for i in range(1<<20):
        val = i | (1 << (TESTBITS+1))
        sv, sc = bitstuf(val, TESTBITS)
        uv = bitunstuf(sv, sc)
        if i != uv:
            report("Mismatch on %d: %s -> %s -> %s (%d)"
                   % (i, format(val, '025b'), format(sv, '025b')
                      , format(uv, '025b'), sv))
            sys.exit(-1)
    report("Test completed successfully")

if __name__ == '__main__':
    main()
