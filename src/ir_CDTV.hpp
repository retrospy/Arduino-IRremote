/*
 * ir_CDTV.hpp
 *
 *  Contains functions for receiving and sending Denon/Sharp IR Protocol
 *
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 ************************************************************************************
 * MIT License
 *
 * Copyright (c) 2020-2023 Armin Joachimsmeyer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************************
 */
 #ifndef _IR_CDTV_HPP
#define _IR_CDTV_HPP

#if defined(DEBUG) && !defined(LOCAL_DEBUG)
#define LOCAL_DEBUG
#else
#define LOCAL_DEBUG // This enables debug output only for this file
#endif

/** \addtogroup Decoder Decoders and encoders for different protocols
 * @{
 */
//==============================================================================
//                            COMMODORE AMIGA CD-TV
//==============================================================================

//==============================================================================
//                           CCCC  DDDD   TTTTT V       V
//                          C      D   D    T    V     V
//                          C      D   D    T     V   V
//                          C      D   D    T      V V
//                           CCCC  DDDD     T       V
//==============================================================================

//==============================================================================
//                                 MEASUREMENTS
//==============================================================================

/*Encoding: UNKNOWN
	Code : 72A03D6B(32 bits)
	Timing[51] :
	+8900, -4450 + 400, -1200 + 350, -400 + 400, -400
	+ 400, -400 + 400, -1200 + 400, -400 + 350, -450
	+ 350, -400 + 400, -400 + 400, -350 + 450, -400
	+ 350, -450 + 350, -400 + 400, -1200 + 400, -1200
	+ 350, -1200 + 400, -450 + 350, -1200 + 400, -1200
	+ 350, -1200 + 400, -1200 + 350, -1250 + 350, -1200
	+ 400, -1200 + 350
	unsigned int rawData[51] = { 8900,4450, 400,1200, 350,400, 400,400, 400,400, 400,1200, 400,400, 350,450, 350,400, 400,400, 400,350, 450,400, 350,450, 350,400, 400,1200, 400,1200, 350,1200, 400,450, 350,1200, 400,1200, 350,1200, 400,1200, 350,1250, 350,1200, 400,1200, 350 }; // UNKNOWN 72A03D6B
*/

// NOTE: The array dump above begins at index = 1; therefore offset = 1 in the code below

//==============================================================================

#define CDTV_BITS          24
// timing intervals in usec
#define CDTV_HDR_MARK    8850			// start burst
#define CDTV_HDR_SPACE   4450			// pause after start
#define CDTV_BIT_MARK     350			// pulse
#define CDTV_ONE_SPACE   1250			// receive a '1'
#define CDTV_ZERO_SPACE   450			// receive a '0'
#define CDTV_RPT_SPACE   2250			// repeat signal
// message sizes measured in raw pulses
#define CDTV_RAW_REPEAT_LENGTH	   4    // CDTV_HDR_MARK + CDTV_HDR_SPACE + CDTV_BIT_MARK
#define CDTV_REPEAT_PERIOD		   50000
#define CDTV_RAW_SIGNAL_LENGTH	   52   // CDTV_HDR_MARK + CDTV_HDR_SPACE + CDTV_BITS * (CDTV_BIT_MARK + CDTV_ZERO_SPACE | CDTV_ONE_SPACE)

struct PulseDistanceWidthProtocolConstants CDTVProtocolConstants = { CDTV, CDTV_KHZ, CDTV_HDR_MARK, CDTV_HDR_SPACE,
CDTV_BIT_MARK, CDTV_ONE_SPACE, CDTV_BIT_MARK, CDTV_ZERO_SPACE, PROTOCOL_IS_MSB_FIRST, (CDTV_REPEAT_PERIOD / MICROS_IN_ONE_MILLI), NULL };

//+=============================================================================

void  IRsend::sendCDTV(unsigned long data, int nbits) {

	// set IR carrier frequency
	enableIROut(40);

	// send header
	mark(CDTV_HDR_MARK);
	space(CDTV_HDR_SPACE);

	// send data
	for (unsigned long mask = 1UL << (nbits - 1); mask; mask >>= 1) {
		if (data & mask) {
			mark(CDTV_BIT_MARK);
			space(CDTV_ONE_SPACE);
		} else {
			mark(CDTV_BIT_MARK);
			space(CDTV_ZERO_SPACE);
		}
	}

	// send footer
	mark(CDTV_BIT_MARK);
	// always end with the LED off
	space(0);

}


//+=============================================================================
// CDTV have a repeat signal that is 4-bits long [#FFFFFF]
//

bool  IRrecv::decodeCDTV() {

	// check if header mark is within range
	if (!MATCH_MARK(decodedIRData.rawDataPtr->rawbuf[1], CDTV_HDR_MARK)) {
		return false;
	}

	// check for 4-bit repeat signal
	if ((decodedIRData.rawDataPtr->rawlen == CDTV_RAW_REPEAT_LENGTH)
		&& MATCH_MARK(decodedIRData.rawDataPtr->rawbuf[1], CDTV_HDR_MARK)\
		&& MATCH_SPACE(decodedIRData.rawDataPtr->rawbuf[2], CDTV_RPT_SPACE)) {
		decodedIRData.numberOfBits = 4;
		decodedIRData.command = 0xFFFFFF;
		decodedIRData.protocol = CDTV;
		return true;
	}

    // we have no start bit, so check for the exact amount of data bits
    // Check we have the right amount of data (52).
    if (decodedIRData.rawDataPtr->rawlen != CDTV_RAW_SIGNAL_LENGTH) {
        return false;
    }

	// check header space
	if (!MATCH_SPACE(decodedIRData.rawDataPtr->rawbuf[2], CDTV_HDR_SPACE)) {
		return false;
	}
	
    // Try to decode as CDTV protocol
    if (!decodePulseDistanceWidthData(&CDTVProtocolConstants, CDTV_BITS, 3)) {
#if defined(LOCAL_DEBUG)
        Serial.print(F("CDTV: "));
        Serial.println(F("Decode failed"));
#endif
        return false;
	}
	
	
	// validate checksum by comparing lower 12-bits with inverted higher 12-bits
	unsigned long lo = decodedIRData.decodedRawData & 0xFFF;		// extract lower 12-bit
	unsigned long hi = decodedIRData.decodedRawData >> 12;			// extract higher 12-bit

	// success if (low XOR high == 0xFFF)
	if (lo^hi == 0xFFF) {
		decodedIRData.numberOfBits = CDTV_BITS;
		decodedIRData.protocol = CDTV;
		return true;
	}

	return false;

}
/** @}*/
#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _IR_CDTV_HPP

