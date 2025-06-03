This document is intended for developers interested in understanding
the can2040 implementation itself.  It provides some high-level
information on the can2040 code organization and objectives.

# PIO state machines

The can2040 code uses the rp2040/rp2350 PIO (Programmable Input
Output) hardware feature.  See the [rp2040
datasheet](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
for information on the hardware.

The PIO instructions used by can2040 are contained in the can2040.c
code.  However, a version of the PIO instructions with additional
comments and other reference information can be found in the
[can2040.pio](../pio/can2040.pio) file.

The rp2040 chip contains two PIO hardware blocks and each PIO hardware
block contains four PIO state machines.  The can2040 code uses one PIO
block and uses all four state machines of that block.

## PIO "sync" state machine

The main task of the PIO "sync" state machine is to synchronize bit
sampling to the incoming data.  This enables communication between CAN
bus nodes that have a slightly different clock rate.  That is, the
sampling time for each transmitted bit is adjusted to account for a
slightly faster or slower bitrate of the transmitter.  The PIO "sync"
state machine raises a "sample" irq (pio irq 4) at each sampling point
and other state machines read the "CAN rx" line after receiving that
signal.

A secondary task of the PIO "sync" state machine is to detect when a
new transmission may start.  It will raise a "maytx" irq (pio irq 0)
after 11 passive bits are detected (in some cases, based on settings
from the ARM core, this may instead be after 17 passive bits).  The
"maytx" irq will also be raised if another transmitter starts a
transmission after 10 passive bits.  This ensures a local message
transmission can participate in line arbitration and that these bit
transmissions are synchronized.

A third task of the PIO "sync" state machine is to disable PIO
"sample" irqs when the CAN bus is idle (after ten passive bits are
detected) and to resume "sample" irqs upon the start of the next
message.  This reduces ARM core processing overhead.

## PIO "rx" state machine

The main task of the PIO "rx" state machine is to relay bits on the
CAN bus to the ARM processing core.  The state machine reads bits from
the "CAN rx" line after each "sample" irq and accumulates the data in
the rx fifo.  After every 10 raw input bits the ARM core is notified
of the pending data.

## PIO "match" state machine

The main task of the PIO "match" state machine is to detect an
anticipated CRC bit sequence and to generate a "matched" signal (PIO
irq 2).  This mechanism is used to transmit an ack bit (a single
dominant bit) at the appropriate point in a received message so as to
acknowledge that message.

The "match" state machine tracks the last 21 raw bits (as read from
the "CAN rx" line after each "sample" irq) along with the total number
of bits that have been read since the PIO hardware block was started
(stored as an 11 bit integer).  The ARM core can send a 32 bit
"match_key" to the PIO containing this combination of raw bits and bit
position.  A "matched" irq is raised when both the sampled raw bits
and counter match the requested match_key.

A secondary task of the PIO "match" state machine is to provide
"matched" signals to the main ARM core for timely notification of
message completion.  The same "match_key" mechanism described above is
used.

## PIO "tx" state machine

The main task of the PIO "tx" state machine is to transmit messages on
the "CAN tx" line.  For each bit to be transmitted, the state machine
sets the tx line, reads the "CAN rx" line, and will stop a
transmission if a dominant/passive bit conflict is found.  This
enables transmissions to properly participate in CAN bus line
arbitration.  To transmit a message, the ARM core fills the tx fifo
with the raw bits of the message and arranges for the "tx" state
machine to start after a "maytx" irq.  The "tx" state machine performs
its own bit time synchronization to resynchronize bit timing to other
simultaneous transmitters that may be transmitting at a slightly
faster bit rate.

A secondary task of the PIO "tx" state machine is to inject an ack bit
to acknowledge messages received from other CAN bus nodes.  To perform
this task, the ARM core fills the tx fifo with a single dominant bit
and arranges for the "tx" state machine to start after a "matched"
irq.  After the ack bit is transmitted the PIO "tx" state machine will
raise an "ackdone" signal (PIO irq 3).

# ARM core state tracking

The ARM core C code must process the raw bits read from the PIO and
arrange for timely signals to be sent to the PIO.  In effect, the PIO
could be thought of as a separate processor with four running threads.
Unfortunately, due to limitations in the PIO's ability to track state,
the ARM C code has a high burden to ensure proper synchronization.

To facilitate this state tracking, the C code maintains three state
tracking variables: `parse_state`, `report_state`, and `tx_state`.

## Parse state

The `parse_state` variable tracks the current message position (eg,
header, data, trailer) as data is read from the "rx" state machine
fifo.  When the "rx" state machine fills an rx fifo the ARM core is
alerted and the ARM PIO irq handler will read that data.  The raw bits
are then checked for bit stuffing and the resulting unstuffed data is
accumulated to assemble the incoming message.

The [CAN bus wikipedia article](https://en.wikipedia.org/wiki/CAN_bus)
is a useful reference for the parts of a CAN bus message and the
corresponding `parse_state` states that are tracked.

Each rx fifo entry is filled after every ten raw bits of data.  The
bits in each fifo entry will not be aligned to message boundaries (a
message start-of-header bit could occur in any of the ten bits in a
fifo entry).  Updates to `parse_state` tracking will typically lag the
actual raw bitstream seen by the PIO by a minimum of five bits, but
could be much larger due to ARM core "irq latency".  Also, the "sync"
state machine disables "sample" irqs after ten passive bits, which
could result in up to nine bits of a message not being observed by the
`parse_state` tracking until the start of the next message (a
potentially significant delay).  The `report_state` tracker is used to
account for these timing limitations.

## Report state

The `report_state` tracking combines information from the rx fifo (as
tracked by `parse_state`) along with direct PIO irq signals to provide
more timely handling of incoming messages.

The `report_state` handling has notable complexity due to its need to
handle several asynchronous events - for example, the rx fifo may be
one or more entries behind the PIO state, ARM core irq latency may
delay observation of a PIO irq until after the PIO state has changed,
a PIO irq may be observed before or after the corresponding bits are
observed from the rx fifo, a scheduled PIO irq may not be scheduled in
time to observe the event and thus never be received, etc. .
Unfortunately, due to limitations in the PIO's ability to perform
state tracking, handling this state tracking complexity in the ARM
core seems to be necessary.

There are five states that `report_state` may be in: `RS_IDLE`,
`RS_NEED_RX_ACK`, `RS_NEED_RX_EOF`, `RS_NEED_TX_ACK`, and
`RS_NEED_TX_EOF`.

The main goal of the `report_state` tracking is to "look ahead" of the
current message state found in `parse_state` to detect successful ack
and eof states as soon as they are observed by the PIO.  This allows
the can2040 code to more quickly report messages to the user code.
This code also arranges for the PIO to generate timely ack bits for
incoming messages and to timely queue outgoing messages in the PIO so
that they are available for line arbitration.

## Transmit state

The `tx_state` variable tracks the current state of messages queued
for transmission in the PIO "tx" state machine.  It is used to
determine if a message needs to be re-queued (for example, if a
previous transmit attempt was preempted by a higher priority message).
It is also used to determine if the current incoming message (as
tracked by `parse_state`) is actually feedback from a locally
transmitted message.
