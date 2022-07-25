This document describes how one may use can2040 in an application.

Note that can2040 uses the GNU GPLv3 license.  See the [COPYING
file](../COPYING) for more information.

# Compiling

The can2040 code is intended to be compiled using the "gcc" C
compiler. The can2040 implementation is contained in the
[can2040.c](../src/can2040.c) and [can2040.h](../src/can2040.h) C
files.  The code is intended to be compiled at `-O2` (or higher)
optimization.

The code depends on a few files from the [rp2040
sdk](https://github.com/raspberrypi/pico-sdk.git) that must be in the
include path when compiling can2040.  For example:
`arm-none-eabi-gcc -O2 -I/path/to/sdk/src/rp2040/ -I/path/to/sdk/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include/ ...`

# Startup

The following provides example startup C code for can2040:

```c
static struct can2040 cbus;

static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    // Add message processing code here...
}

static void
PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void
canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
    NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}
```

The `can2040.h` header file provides the definition of `struct
can2040`.  This definition is exported so that one may statically
allocate an instance of it.  (It is also valid to dynamically allocate
it if desired.)  The content of the `struct can2040` is considered
"private" - callers should not inspect or modify its content for any
reason.  All interaction with can2040 is done via can2040 API
functions.

The can2040 code does not dynamically allocate memory; all storage is
contained in the `struct can2040` that the caller allocates.

# can2040_msg

The `can2040.h` header file provides a definition for `struct
can2040_msg`.  This struct is used for transmitting and receiving CAN
bus messages.

The `id` field contains the CAN bus message id.  It may also have the
`CAN2040_ID_RTR` bit and/or `CAN2040_ID_EFF` bits set.  The
`CAN2040_ID_RTR` bit indicates that a "remote" frame should be used.
The `CAN2040_ID_EFF` bit indicates that a 29-bit extended header
should be used.

The `dlc` field specifies the number of data bytes the CAN bus message
contains.  In accordance with the CAN bus specification, the value may
be between 0 and 15, however there are at most 8 data bytes.  (A dlc
value between 8 to 15 will have 8 data bytes.)

The `data` field contains the data bytes of the CAN bus message.

# Function reference

## can2040_setup

`void can2040_setup(struct can2040 *cd, uint32_t pio_num)`

This function initializes the `struct can2040` passed in the `cd`
parameter.  The caller must either statically or dynamically allocate
it prior to calling this function.

The `pio_num` should be either `0` or `1` to use either the `PIO0` or
`PIO1` rp2040 hardware block.

## can2040_callback_config

`void can2040_callback_config(struct can2040 *cd, can2040_rx_cb rx_cb)`

This function specifies the main can2040 callback (as specified in the
`rx_cb` parameter).

The `can2040_rx_cb` callback function will be invoked with each
successfully received and transmitted message.  It must be provided by
the user code.

The callback uses the function prototype:
`void can2040_rx_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)`

The `cd` parameter of the callback contains the same pointer passed to
`can2040_callback_config()`.

The `notify` parameter of the callback contains one of
`CAN2040_NOTIFY_RX`, `CAN2040_NOTIFY_TX`, `CAN2040_NOTIFY_ERROR`.
* A `CAN2040_NOTIFY_RX` event indicates a message has been
  successfully read. The message contents will be in the `msg`
  parameter.
* A `CAN2040_NOTIFY_TX` event indicates a message has been
  successfully transmitted on the CAN bus. The transmitted message
  contents will be in the `msg` parameter.
* A `CAN2040_NOTIFY_ERROR` event indicates that the internal receive
  buffers have overflowed and that some number of CAN bus messages may
  have been lost.  The `msg` parameter will point to an allocated but
  otherwise empty `struct can2040_msg` in this event.

The callback is invoked in IRQ context (it is called from
`can2040_pio_irq_handler()`).  To keep irq latency low, it is
recommended that the callback copy the given message to memory to be
processed during normal (non-irq) context.

The `msg` pointer is only valid during the duration of the callback.
The callback code should copy any desired content from `msg` to its
own storage during the callback.

The callback is invoked for all valid received messages on the CAN
bus.  The can2040 code does not implement receive message filtering.
It is expected that the caller will implement any desired filtering in
their callback function.

## can2040_start

`void can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate, uint32_t gpio_rx, uint32_t gpio_tx)`

This function starts the main can2040 CAN bus implementation.  The
provided GPIO pins will be configured and associated with the rp2040
PIO hardware block.

The `sys_clock` parameter specifies the system clock rate (for example
`125000000` for an rp2040 ARM core running at 125Mhz).

The `bitrate` parameter specifies the CAN bus speed (for example
`500000` for a 500Kbit/s CAN bus).

The `gpio_rx` parameter specifies the rp2040 gpio number that is
routed to the "CAN RX" pin of the CAN bus transceiver.  It should be
between 0 and 29 (for GPIO0 to GPIO29).

The `gpio_tx` parameter specifies the rp2040 gpio number that is
routed to the "CAN TX" pin of the CAN bus transceiver.  It should be
between 0 and 29 (for GPIO0 to GPIO29).

After calling this function, activity on the CAN bus may result in the
user specified `can2040_rx_cb` callback being invoked.

## can2040_pio_irq_handler

`void can2040_pio_irq_handler(struct can2040 *cd)`

This function should be invoked on each PIO hardware IRQ event.  The
caller should define the underlying IRQ handler function, enable it,
and arrange to call `can2040_pio_irq_handler()` with each event.  The
can2040 code uses the first IRQ of each PIO block (`PIO0_IRQ_0_IRQn`
for PIO0, or `PIO1_IRQ_0_IRQn` for PIO1).

This function is intended to be invoked from IRQ context.  It is
reentrant safe with respect to other can2040 function calls made on
the same ARM core.

## can2040_transmit

`int can2040_transmit(struct can2040 *cd, struct can2040_msg *msg)`

This function schedules a message for transmission on the CAN bus.
The `msg` parameter should contain a pointer to the `struct
can2040_msg` that should be sent.  The function returns `0` if the
message is successfully queued, or a negative number if there is no
space for the message in the internal queue.  If scheduled, the
contents of the `msg` pointer are copied to internal storage during
the `can2040_transmit()` call.

When a scheduled message is successfully transmitted on the CAN bus
the user supplied `can2040_rx_cb` callback will be invoked with a
`CAN2040_NOTIFY_TX` event.

The can2040 code may buffer up to four messages for transmission.  If
multiple messages are buffered then they are transmitted in "first in
first out" order.  (The buffered transmit messages are not reordered
by message id priority.)

It is valid to invoke `can2040_transmit()` from the user supplied
`can2040_rx_cb` callback function, however doing so is not
recommended.  The `can2040_transmit()` function has processing
overhead (it performs CRC calculation and bitstuffing on the message)
and calling it from IRQ context may increase irq latency.

## can2040_check_transmit

`int can2040_check_transmit(struct can2040 *cd)`

This function may be called to determine if there is space available
to schedule a message transmission.  That is, it may be used to
determine if a call to `can2040_transmit()` will succeed.

It is valid to invoke `can2040_check_transmit()` from the user
supplied `can2040_rx_cb` callback function.

# Not reentrant safe

Unless explicitly stated otherwise, the can2040 code is not reentrant
safe.  For example, unless stated otherwise, one may not invoke
can2040 functions from IRQ context (where they might preempt a can2040
function running in another context), one may not invoke can2040
functions from multiple ARM cores (such that two can2040 functions
could be running simultaneously), nor invoke can2040 functions from
the user supplied `can2040_rx_cb` callback function.

# Multiple can2040 instances

Each instance of can2040 uses one of the two PIO hardware blocks on
the rp2040.  If there are two separate CAN transceivers (and two
separate sets of CAN rx and CAN tx gpio pins) then one may create two
simultaneous instances of can2040.

To use this functionality, the [startup code](#startup) should be run
twice, each with their own separate instance of a `struct can2040`.

In this case, the multiple instances of can2040 do not share state.
Therefore, no particular synchronization is needed between instances.
That is, one must ensure each instance is not reentrant with respect
to itself, but it is not required to synchronize between instances.
One may run both can2040 instances on the same ARM core or different
ARM cores.
