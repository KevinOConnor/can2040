This is an unsorted list of known limitations and areas for
improvement.

* Improve error handling:
  * CAN specs have guidelines for tracking "active" and "passive"
    errors, along with guidelines for disabling bus on excessive
    errors.
  * On a failed transmit, the code should have an additional delay
    before retrying.

* Optimizations:
  * Further optimize bitstuff/bitunstuff code
  * Optimize crc calculation

* Improve support for running across rp2040 ARM cores.

* Add documentation:
  * Improve code documentation
  * Document the public API
