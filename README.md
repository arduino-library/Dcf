# Dcf

Library for decoding the DCF77 radio clock signal originating from an external off-the-shelf DCF77 receiver module.  

Main features:

* Decoding of DCF77 time and date stamp
* Triggering on rising or falling edge of an external pin interrupt
* Plausibility checking of the decoded timestamp

Usage example:

* https://github.com/microfarad-de/nixie-clock

Git commands:

* `git submodule add ssh://git@github.com/arduino-library/Dcf src/Dcf`
* `git submodule add https://github.com/arduino-library/Dcf src/Dcf`
