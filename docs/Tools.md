This document describes some available tools that use the can2040
code. These tools may facilitate development and may be useful as
coding examples.

# Klipper

The [Klipper](https://www.klipper3d.org/) code utilizes can2040 for
micro-controller CAN bus communication on rp2040 chips.

Klipper can also be compiled in a [USB to CAN bus bridge
mode](https://www.klipper3d.org/CANBUS.html#usb-to-can-bus-bridge-mode)
so that an rp2040 device appears as a standard Linux USB to CAN bus
adapter (using the "gs_usb" Linux driver).  Compiling the Klipper
micro-controller code in this mode may be useful for CAN bus
diagnostics even when not using Klipper.

In particular, the Linux
[can-utils](https://github.com/linux-can/can-utils) package (eg, `sudo
apt-get install can-utils`) can then be used to send and receive
packets via can2040 running on an rp2040.  Once the CAN bus interface
has been configured in Linux (eg, `sudo ip link set can0 up type can
bitrate 1000000`) then the `candump` tool (eg, `candump -t z -Ddex
can0,#FFFFFFFF`) may be used to report activity on the CAN bus and the
`cansend` tool (eg, `cansend can0 123#121212121212`) may be used to
send messages on the CAN bus.

# CanBoot

The [CanBoot](https://github.com/Arksine/CanBoot) code implements a
cross-platform bootloader that supports flashing an rp2040
micro-controller over CAN bus.  It utilizes can2040 on rp2040 chips.

# Testing environment

Note that can2040 (and CAN bus in general) requires a functional
hardware bus for proper message generation.  At a minimum, can2040
requires a functioning transceiver, functioning "CAN H" and "CAN L"
wiring, two 120 Ohm resistors on that wiring, at least one additional
CAN bus enabled chip with its own transceiver, and all chips much be
configured with the same CAN bus frequency.  If any of the above
hardware is missing or not properly connected/configured then the bus
will not function correctly; not even for debugging purposes.
