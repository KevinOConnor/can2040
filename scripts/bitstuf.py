#!/usr/bin/env python
# Tool for testing bitstuffing implementations
#
# Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys

TESTBITS=20

def report(msg):
    sys.stderr.write("\n" + msg + "\n")

LOOP = UNLOOP = 0

def bitstuf(b, num_bits):
    global LOOP
    edges = b ^ (b >> 1)
    count = num_bits
    i = num_bits-1
    while i >= 0:
        LOOP += 1
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

def bitstuf_batch(b, num_bits):
    global LOOP
    count = num_bits
    while 1:
        edges = b ^ (b >> 1)
        e2 = edges | (edges >> 1)
        e4 = e2 | (e2 >> 2)
        add_bits = ~e4
        try_cnt = num_bits
        while 1:
            LOOP += 1
            try_mask = ((1 << try_cnt) - 1) << (num_bits - try_cnt)
            if not add_bits & try_mask:
                # No stuff bits needed in try_cnt bits
                if try_cnt >= num_bits:
                    return b, count
                num_bits -= try_cnt
                try_cnt = (num_bits + 1) // 2
                continue
            if add_bits & (1 << (num_bits - 1)):
                # A stuff bit must be inserted prior to the high bit
                low_mask = (1 << num_bits) - 1
                low = b & low_mask
                high = (b & ~(low_mask >> 1)) << 1
                b = high ^ low ^ (1 << (num_bits - 1))
                count += 1
                if num_bits <= 4:
                    return b, count
                num_bits -= 4
                break
            # High bit doesn't need stuff bit - accept it, limit try_cnt, retry
            num_bits -= 1
            try_cnt //= 2

def bitunstuf(sb, num_bits):
    global UNLOOP
    edges = sb ^ (sb >> 1)
    unstuffed_bits = 0
    cu = TESTBITS
    cs = num_bits
    while 1:
        UNLOOP += 1
        if not cu:
            # Extracted desired bits
            return unstuffed_bits
        if not cs:
            # Need more data
            return -999
        cs -= 1
        if (edges >> (cs+1)) & 0xf:
            # Normal data
            cu -= 1
            unstuffed_bits |= ((sb >> cs) & 1) << cu
        elif ((edges >> cs) & 0x1f) == 0x00:
            # Six consecutive bits - a bitstuff error
            if (sb >> cs) & 1:
                return -cs
            return -cs

def bitunstuf_batch(sb, num_bits):
    global UNLOOP
    edges = sb ^ (sb >> 1)
    e2 = edges | (edges >> 1)
    e4 = e2 | (e2 >> 2)
    rm_bits = ~e4
    unstuffed_bits = 0
    cu = TESTBITS
    cs = num_bits
    while 1:
        try_cnt = cu if cu < cs else cs
        while 1:
            UNLOOP += 1
            try_mask = ((1 << try_cnt) - 1) << (cs + 1 - try_cnt)
            if not (rm_bits & try_mask):
                # No stuff bits in try_cnt bits - copy into unstuffed_bits
                cu -= try_cnt
                cs -= try_cnt
                unstuffed_bits |= ((sb >> cs) & ((1 << try_cnt) - 1)) << cu
                if not cu:
                    # Extracted desired bits
                    return unstuffed_bits
                break
            cs -= 1
            if rm_bits & (1 << (cs + 1)):
                # High bit of try_cnt a stuff bit
                if rm_bits & (1 << cs):
                    # Six consecutive bits - a bitstuff error
                    if (sb >> cs) & 1:
                        return -cs
                    return -cs
                break
            # High bit not a stuff bit - limit try_cnt and retry
            cu -= 1
            unstuffed_bits |= ((sb >> cs) & 1) << cu
            try_cnt //= 2
        if not cs:
            # Need more data
            return -999

def bitunstuf_batch_pass4(sb, num_bits):
    global UNLOOP
    edges = sb ^ (sb >> 1)
    e2 = edges | (edges >> 1)
    e4 = e2 | (e2 >> 2)
    rm_bits = ~e4
    unstuffed_bits = 0
    cu = TESTBITS
    cs = num_bits
    try_cnt = cu if cu < cs else cs
    while 1:
        UNLOOP += 1
        pass_cnt = try_cnt
        try_mask = ((1 << try_cnt) - 1) << (cs + 1 - try_cnt)
        if rm_bits & try_mask:
            # There is a stuff bit somewhere in try_cnt
            if rm_bits & (1 << cs):
                # High bit is a stuff bit
                cs -= 1
                if rm_bits & (1 << cs):
                    # Six consecutive bits - a bitstuff error
                    if (sb >> cs) & 1:
                        return -cs
                    return -cs
                rem_cnt = cu if cu < cs else cs
                pass_cnt = try_cnt = rem_cnt if rem_cnt < 4 else 4
            else:
                # High bit is not a stuff bit - pass 1, limit try_cnt, retry
                pass_cnt = 1
        # Copy pass_cnt bits into unstuffed_bits
        cu -= pass_cnt
        cs -= pass_cnt
        unstuffed_bits |= ((sb >> cs) & ((1 << pass_cnt) - 1)) << cu
        if not cu:
            # Extracted desired bits
            return unstuffed_bits
        if not cs:
            # Need more data
            return -999
        if pass_cnt >= try_cnt:
            try_cnt = cu if cu < cs else cs
        else:
            try_cnt //= 2

def main():
    stuf_func = bitstuf_batch
    unstuf_func = bitunstuf_batch
    for i in range(1<<TESTBITS):
        if not i % (1<<12):
            sys.stdout.write('.')
            sys.stdout.flush()
        val = i | (1 << (TESTBITS+1))
        sv, sc = stuf_func(val, TESTBITS)
        uv = unstuf_func(sv, sc)
        if i != uv:
            report("Mismatch on %d: %s -> %s -> %s (%d)"
                   % (i, format(val, '025b'), format(sv, '025b')
                      , format(uv, '025b'), sv))
            sys.exit(-1)
    report("Test completed successfully (avg passes: %.3f stuff, %.3f unstuff)"
           % (LOOP / (1<<TESTBITS), UNLOOP / (1<<TESTBITS)))
    # Verify bit stuff errors are detected
    if (unstuf_func(0b1100111100000011, 16) != -2
        or unstuf_func(0b1101111110000000, 16) != -7):
        report("Didn't detect stuff error")
        sys.exit(-1)

if __name__ == '__main__':
    main()
