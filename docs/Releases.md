History of can2040 releases.

# v1.7.0

Available on 20250102. Major features in this release:
* Support for rp2350 chips. The can2040 code can now run on either
  rp2040 chips or rp2350 chips.
* Support for v2.0.0 of the pico-sdk.

# v1.6.0

Available on 20231001. Major features in this release:
* Added a new `can2040_stop()` API function. This function can be used
  to either temporary pause or permanently halt the processing of CAN
  bus messages. It is effectively the inverse of the existing
  `can2040_start()` function.
* Added a new `can2040_get_statistics()` API function. This can be
  used to monitor the low-level performance of the CAN bus interface.
  In particular it can monitor the number of can2040 parse errors and
  retransmit attempts.

# v1.5.0

Available on 20230520. Major features in this release:
* Message transmit bit timing will now only synchronize to faster
  transmitters (as is recommended by the CAN bus specification). This
  avoids poor transmit timing due to delays between CAN Tx and CAN Rx
  signals.
* Improved multi-core support in `can2040_transmit()`. This function
  can now be called on one core while the other core may be running
  `can2040_pio_irq_handler()`.
* Fixed a possible "transmit hang" that could occur if a transmit
  fails due to reception of an error frame.

# v1.4.0

Available on 20221121. Major features in this release:
* Reduce scheduling latency of new transmit requests that occur while
  parsing the end of an incoming message.
* Reduce ARM core processing load by reading 10 bits at a time from
  the PIO.
* Fixed message retransmit handling when no other node on the bus is
  transmitting.
* Fixed selection of PIO1 hardware block.
* Assorted documentation and internal code cleanups.

The v1.4.0 release was retroactively tagged on 20230313.

## v1.4.1

Available on 20230314.  This is a bug fix only release:
* Fixed error that could cause a "duplicate ack" to be generated in
  rare situations.

# v1.3.0

Available on 20220725. Major features in this release:
* Reduce scheduling latency for transmissions immediately after
  receiving a message.
* Assorted documentation and internal code cleanups.

The v1.3.0 release was retroactively tagged on 20230313.

# v0.2.0

Available on 20220630. Major features in this release:
* Initial API and user documentation.
* Support "extended" and "remote" frames.
* Increase PIO frequency to improve receive message sample timing.
* No need to require DMA hardware.  This simplifies the user-facing API.
* Improved handling of invalid data found during message parsing.
* Assorted ARM CPU processing optimizations.
* Assorted internal code cleanups.

The v0.2.0 release was retroactively tagged on 20230313.

# v0.1.0

Available on 20220602. Major features in this release:
* Initial code release.
* Basic support for sending and receiving CAN 2.0B data frames.

The v0.1.0 release was retroactively tagged on 20230313.
