This document describes features and limitations of the can2040 CAN
bus implementation.

# Main features

* Support for sending and receiving CAN 2.0B data frames.  Support for
  both standard headers (11-bit ids) and extended headers (29-bit
  ids).

* Support for bus speeds up to 1Mbit/s.

* Interoperates with other CAN bus implementations.  A bus may consist
  of one or more can2040 nodes along with non-can2040 nodes.

* The implementation uses one of the two rp2040 PIO hardware blocks.
  It is possible for a single rp2040 chip to have two separate CAN bus
  interfaces by using both PIO blocks.

* Works with standard CAN bus transceivers.  Any two rp2040 gpio pins
  may be used for the "can rx" and "can tx" wires.

# Protocol details

This section provides some low-level details on can2040's
implementation of the CAN bus protocol.  This section is targeted at
CAN bus experts interested in implementation details.

* Supports clock synchronization for bit timing.  That is, the
  sampling time is resynchronized on each passive to dominant bit
  transition.

* Supports verification of SOF (start-of-frame), header, data, CRC,
  ack, and EOF (end-of-frame) portions of each frame on the CAN bus.

* Will inject an ack bit after verification of message CRC when
  receiving a message.

* Transmissions will perform "line arbitration".  That is, lower
  priority transmissions (as based on message id) will yield the bus
  to higher priority messages.  In addition, the start of
  transmissions will be aligned to the SOF bit of other transmitters
  for proper bit timing during line arbitration.

* Support for automatic retransmissions.  If a transmission does not
  complete successfully (for example, due to an error, lack of
  acknowledgment, or interrupted by a higher priority message) then
  it will be automatically retransmitted at the next opportunity.

* Supports sending and receiving "remote frames".

* Support for receiving "overload frames" (they will successfully
  delay transmission of messages).  The can2040 code does not transmit
  "overload frames".

* Support for receiving "error frames" (they will halt an active
  transmission).  However, can2040 does not transmit "error frames".

There are some known limitations with CAN bus error handling:

* The CAN bus specification defines three error handling states:
  "error active" (ie, normal state), "error passive", and "bus off".
  Automatic transition between these states is not implemented.  The
  can2040 code does not transmit "error frames".  The can2040 code
  will not automatically enter a "bus off" state.  In this regard, the
  can2040 code may be thought of as always being in the "error
  passive" state.

# Software utilization

The can2040 system is a software CAN bus implementation.  It utilizes
the rp2040 PIO device as well as processing time on one of the rp2040
ARM cores.

One may dedicate an ARM core to can2040.  It is also possible for
can2040 to share an ARM core with application code.  This section is
targeted at software developers interested in understanding the
software overhead of can2040 when sharing an ARM core.

* The ARM core processing time is dependent on the amount of bus
  traffic, even if that bus traffic is not intended for the node. It
  is expected that a fully saturated CAN bus at the fastest supported
  rate of 1Mbit/s may use up to ~25% of one of the two rp2040 ARM
  cores (when the ARM core is running at 125Mhz).  A slower CAN bus
  speed would have a lower worst case processing time (for example, a
  bus speed of 500kbit/s is expected to have half the worst case
  processing time of a 1Mbit/s bus).  A CAN bus that is idle does not
  consume any ARM core processing time.

* The can2040 code requires low ARM core "irq latency" for proper
  functionality.  If an ARM core is shared with both application code
  and can2040, and that application code disables irqs, has high
  priority irqs that preempt the can2040 irq handler, or takes similar
  actions that increase irq latency then it may impact can2040.

  It is desirable to keep application irq latency low.  The following
  paragraphs provide some guidance on what may happen if irq latency
  exceeds certain thresholds.  The "bit time" refers to the amount of
  time it takes to transmit one bit on the CAN bus.  It is dependent
  on the CAN bus speed (for example, a bus frequency of 500000 would
  have a bit time of 2 microseconds).

  Above ~3 bit times of irq latency: After successfully transmitting a
  message, the can2040 code may not have sufficient time to schedule
  the next transmission before another node starts a transmission of
  lower priority.  As a result, back-to-back high-priority transmits
  may be delayed access to a highly contended bus.  That is, even if
  the node has many high-priority messages it may only get access to
  the bus on every other message.

  Above ~7 bit times: The code may not have sufficient time to
  schedule an acknowledgment for a received message, and/or the code
  may not have sufficient time to schedule a high-priority
  transmission before another node starts a transmission of lower
  priority.  This may result in message retransmissions and it may
  result in a high-priority message from the node being delayed during
  periods of high bus contention.

  Above ~81 bit times: The code may not have sufficient time to read
  all bytes before the "PIO FIFO queue" overflows.  As a result,
  received messages on the CAN bus sent to the node may be permanently
  lost and messages transmitted from the node may be transmitted
  multiple times.

  The can2040 irq handler itself may introduce some irq latency (to
  itself and other irq handlers of equal or lower priority).  With an
  ARM core running at 125Mhz, it is expected that each can2040 irq
  will typically take between 1 to 3 microseconds.  (An rp2040
  instruction flash cache miss and/or higher priority irqs may
  increase this time.)
