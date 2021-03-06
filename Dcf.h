/*
 * Class for decoding the DCF77 signal
 *
 * Implemented using the "Pollin DCF1" module with
 * with inverted output signal (bit starts on falling edge).
 *
 * This source file can be found under:
 * http://www.github.com/arduino-library/Dcf
 *
 * This source file is used by the Nixie Clock Arduino firmware
 * found under http://www.github.com/microfarad-de/nixie-clock
 *
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 *   http://www.github.com/arduino-library
 *
 * Copyright (C) 2019 Karim Hraibi (khraibi at gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DCF_H
#define __DCF_H

#include <Arduino.h>
#include <time.h>

/*
 * Activates debug values
 */
//#define DCF_DEBUG_VALUES

#ifdef DCF_DEBUG_VALUES
  #define DCF_NUM_DEBUG_VALUES 4
#endif

/*
 * Total number of bits in a DCF77 word
 */
#define DCF_BIT_COUNT 60

/*
 * DCF bit values
 */
enum DcfBit_e {
  DCF_BIT_LOW = 0,
  DCF_BIT_HIGH = 1,
  DCF_BIT_SYNC = 2,
  DCF_BIT_NONE = 3
};


/*
 * Main class
 */
class DcfClass {
  public:
    /*
     * Initialize the DCF receiver
     * Parameters:
     *   dcfPin     : digital input pin connected to the DCF module
     *   bitStart   : whether DCF77 bits start with a rising or falling edge, set to RISING or FALLING
     *   dcfPinMode : whether to use the internal pullup resitor, set to INPUT or INPUT_PULLUP
     */
    void initialize (uint8_t dcfPin, uint8_t bitStart=FALLING, uint8_t dcfPinMode=INPUT);

    /*
     * Pause DCF reception by disabling the DCF77 signal interrupt
     */
    void pauseReception (void);

    /*
     * Resume DCF reception by re-enabling the DCF77 signal  interrupt
     */
    void resumeReception (void);


    /*
     * Read the DCF time
     * This method must be called in a fast loop until it returns 0 for success.
     * Upon success, the current time is stored in the DCF.currentTm variable.
     * Parameters:
     *   void
     * Return value:
     *    0     : success, DCF.currentTm updated with a new time value
     *    1..13 : decoded values out of range, DCF.currentTm contains invalid data!
     *   21..23 : parity check failed, DCF.currentTm contains invalid data!
     *   31     : too many bits detected
     *   32     : too few bits detected
     *   33     : a new bit has been detected
     *   41     : detection in progress
     */
    uint8_t getTime (void);

    /*
     * Current time is stored in this variable once getTime succeeds
     */
    struct tm currentTm;

    /*
     * Stores the detected DCF pulse edge
     * LOW:  falling edge
     * HIGH: rising edge
     */
    uint8_t edge;

    /*
     * Index of the last received DCF bit
     * This variable can be safely reset from outside this class
     * Used for debug purposes
     */
    uint32_t lastIdx = 0;

#ifdef DCF_DEBUG_VALUES
    /*
     * General purpose debug values
     */
    bool debug[DCF_NUM_DEBUG_VALUES] = { false };
#endif
    
  private:
    DcfBit_e readBit (void);
    uint8_t  verify (void);
    bool     configured = false;
    bool     active     = false;
    uint8_t  dcfPin;
    uint8_t  startEdge;
    uint8_t  bits[DCF_BIT_COUNT];
};

/*
 * DCF class is instantiated as a singleton
 */
extern DcfClass Dcf;

#endif // __DCF_H
