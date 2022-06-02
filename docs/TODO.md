This is an unsorted list of known limitations and areas for
improvement.

* Current transmit scheduling is sub-optimal.  By the time the ARM
  core observes a message completion, it is likely the "inter frame
  space" will have already completed.  That is likely to result in
  outgoing packets not getting priority during bus contention.

* Support CAN2.0B extended frames.
  * Possibly support "remote" frames?

* Possibly use 32 clock ticks per bit (instead of 16)
  * Current code samples on phase 10 of 16 (62.5% +- 12.5%) while most
    references indicate sampling at 87.5% is preferable.

* Improve error handling:
  * CAN specs have guidelines for tracking "active" and "passive"
    errors, along with guidelines for disabling bus on excessive
    errors.
  * On a failed transmit, the code should have an additional delay
    before retrying.

* Optimizations:
  * Further optimize bitstuff/bitunstuff code
  * Optimize crc calculation
  * Possibly reduce DMA irqs in some cases (up to 24bits per wakeup
    instead of current wakeup every 8 bits)

* Improve support for running across rp2040 ARM cores.

* Add documentation:
  * Improve code documentation
  * Document the public API
