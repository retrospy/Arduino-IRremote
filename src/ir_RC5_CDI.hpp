/*
 * ir_RC5_CDI_RC6.hpp
 *
 *  Contains functions for receiving and sending RC5, RC5X, RC6 protocols
 *
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 ************************************************************************************
 * MIT License
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
#ifndef _IR_RC5_CDI_HPP
#define _IR_RC5_CDI_HPP

#if defined(DEBUG) && !defined(LOCAL_DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file
#endif

/** \addtogroup Decoder Decoders and encoders for different protocols
 * @{
 */
uint8_t sCDILastSendToggleValue = 1; // To start first command with toggle 0
//uint8_t sLastReceiveToggleValue = 3; // 3 -> start value

//==============================================================================
//     RRRR    CCCC  55555
//     R   R  C      5
//     RRRR   C      5555
//     R  R   C          5
//     R   R   CCCC  5555
//==============================================================================
/*
 Protocol=RC5 Address=0x11 Command=0x36 Raw-Data=0x1476 13 bits MSB first
 + 900,- 900
 +1800,-1750 +1800,- 850 + 900,- 850 + 900,-1750
 + 950,- 850 + 900,- 850 +1800,-1750 + 950,- 850
 +1800
 Sum: 23100

 RC5X with 7.th MSB of command set
 Protocol=RC5 Address=0x11 Command=0x76 Toggle=1 Raw-Data=0xC76 13 bits MSB first
 +1800,-1750
 + 850,- 900 +1800,- 850 + 950,- 850 + 900,-1750
 + 900,- 850 + 950,- 850 +1800,-1750 + 900,- 850
 +1800
 Sum: 23050
 */
//
// see: https://www.sbprojects.net/knowledge/ir/rc5.php
// https://en.wikipedia.org/wiki/Manchester_code
// https://forum.arduino.cc/t/sending-rc-5-extended-code-using-irsender/1045841/10 - Protocol Maranz Extended
// mark->space => 0
// space->mark => 1
// MSB first 1 start bit, 1 field bit, 1 toggle bit + 5 bit address + 6 bit command, no stop bit
// Field bit is 1 for RC5 and inverted 7. command bit for RC5X. That way the first 64 commands of RC5X remain compatible with the original RC5.
// SF TAAA  AACC CCCC
// IR duty factor is 25%,
//
#define RC5_CDI_ADDRESS_BITS        5
#define RC5_CDI_COMMAND_BITS        6
#define RC5_CDI_COMMAND_FIELD_BIT   1
#define RC5_CDI_TOGGLE_BIT          1

#define RC5_CDI_BITS            (RC5_CDI_COMMAND_FIELD_BIT + RC5_CDI_TOGGLE_BIT + RC5_CDI_ADDRESS_BITS + RC5_CDI_COMMAND_BITS) // 13

#define RC5_CDI_UNIT            450 

#define MIN_RC5_CDI_MARKS       ((RC5_CDI_BITS + 1) / 2) // 7. Divided by 2 to handle the bit sequence of 01010101 which gives one mark and space for each 2 bits

#define RC5_CDI_DURATION        (15L * RC5_CDI_UNIT) // 13335
#define RC5_CDI_REPEAT_PERIOD   (128L * RC5_CDI_UNIT) // 113792
#define RC5_CDI_REPEAT_DISTANCE (RC5_CDI_REPEAT_PERIOD - RC5_CDI_DURATION) // 100 ms
#define RC5_CDI_MAXIMUM_REPEAT_DISTANCE     (RC5_CDI_REPEAT_DISTANCE + (RC5_CDI_REPEAT_DISTANCE / 4)) // Just a guess

/************************************
 * Start of send and decode functions
 ************************************/

/**
 * @param aCommand If aCommand is >=0x40 then we switch automatically to RC5X.
 * @param aEnableAutomaticToggle Send toggle bit according to the state of the static sCDILastSendToggleValue variable.
 */
void IRsend::sendRC5_CDI(uint8_t aAddress, uint8_t aCommand, int_fast8_t aNumberOfRepeats, bool aEnableAutomaticToggle) {
    // Set IR carrier frequency
    enableIROut (RC5_CDI_KHZ);

    uint16_t tIRData = ((aAddress & 0x1F) << RC5_CDI_COMMAND_BITS);

    if (aCommand < 0x40) {
        // Auto discovery of RC5X, set field bit to 1
        tIRData |= 1 << (RC5_CDI_TOGGLE_BIT + RC5_CDI_ADDRESS_BITS + RC5_CDI_COMMAND_BITS);
    } else {
        // Mask bit 7 of command and let field bit 0
        aCommand &= 0x3F;
    }
    tIRData |= aCommand;

    if (aEnableAutomaticToggle) {
        if (sCDILastSendToggleValue == 0) {
            sCDILastSendToggleValue = 1;
            // set toggled bit
            tIRData |= 1 << (RC5_CDI_ADDRESS_BITS + RC5_CDI_COMMAND_BITS);
        } else {
            sCDILastSendToggleValue = 0;
        }
    }

    uint_fast8_t tNumberOfCommands = aNumberOfRepeats + 1;
    while (tNumberOfCommands > 0) {

        // start bit is sent by sendBiphaseData
        sendBiphaseData(RC5_CDI_UNIT, tIRData, RC5_CDI_BITS);

        tNumberOfCommands--;
        // skip last delay!
        if (tNumberOfCommands > 0) {
            // send repeated command in a fixed raster
            delay(RC5_CDI_REPEAT_DISTANCE / MICROS_IN_ONE_MILLI);
        }
    }
}

/**
 * Try to decode data as RC5 protocol
 *                             _   _   _   _   _   _   _   _   _   _   _   _   _
 * Clock                 _____| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |
 *                                ^   ^   ^   ^   ^   ^   ^   ^   ^   ^   ^   ^    End of each data bit period
 *                               _   _     - Mark
 * 2 Start bits for RC5    _____| |_| ...  - Data starts with a space->mark bit
 *                                         - Space
 *                               _
 * 1 Start bit for RC5X    _____| ...
 *
 */
bool IRrecv::decodeRC5_CDI() {
    uint8_t tBitIndex;
    uint32_t tDecodedRawData = 0;

    // Set Biphase decoding start values
    initBiphaselevel(1, RC5_CDI_UNIT); // Skip gap space

    // Check we have the right amount of data (11 to 26). The +2 is for initial gap and start bit mark.
    if (decodedIRData.rawDataPtr->rawlen < ((RC5_CDI_BITS + 1) / 2) + 2 && (RC5_CDI_BITS + 2) < decodedIRData.rawDataPtr->rawlen) {
        // no debug output, since this check is mainly to determine the received protocol
        IR_DEBUG_PRINT(F("RC5: "));
        IR_DEBUG_PRINT(F("Data length="));
        IR_DEBUG_PRINT(decodedIRData.rawDataPtr->rawlen);
        IR_DEBUG_PRINTLN(F(" is not between 9 and 15"));
        return false;
    }

// Check start bit, the first space is included in the gap
    if (getBiphaselevel() != MARK) {
        IR_DEBUG_PRINT(F("RC5_CDI: "));
        IR_DEBUG_PRINTLN(F("first getBiphaselevel() is not MARK"));
        return false;
    }

    /*
     * Get data bits - MSB first
     */
    for (tBitIndex = 0; sBiphaseDecodeRawbuffOffset < decodedIRData.rawDataPtr->rawlen; tBitIndex++) {
        // get next 2 levels and check for transition
        uint8_t tStartLevel = getBiphaselevel();
        uint8_t tEndLevel = getBiphaselevel();

        if ((tStartLevel == SPACE) && (tEndLevel == MARK)) {
            // we have a space to mark transition here
            tDecodedRawData = (tDecodedRawData << 1) | 1;
        } else if ((tStartLevel == MARK) && (tEndLevel == SPACE)) {
            // we have a mark to space transition here
            tDecodedRawData = (tDecodedRawData << 1) | 0;
        } else {
#if defined(LOCAL_DEBUG)
            Serial.print(F("RC5_CDI: "));
            Serial.println(F("no transition found, decode failed"));
#endif
            return false;
        }
    }

    // Success
    decodedIRData.numberOfBits = tBitIndex; // must be RC5_CDI_BITS

    LongUnion tValue;
    tValue.ULong = tDecodedRawData;
    decodedIRData.decodedRawData = tDecodedRawData;
    decodedIRData.command = tValue.UByte.LowByte & 0x3F;
    decodedIRData.address = (tValue.UWord.LowWord >> RC5_CDI_COMMAND_BITS) & 0x1F;

    // Get the inverted 7. command bit for RC5X, the inverted value is always 1 for RC5 and serves as a second start bit.
    if ((tValue.UWord.LowWord & (1 << (RC5_CDI_TOGGLE_BIT + RC5_CDI_ADDRESS_BITS + RC5_CDI_COMMAND_BITS))) == 0) {
        decodedIRData.command += 0x40;
    }

    decodedIRData.flags = IRDATA_FLAGS_IS_MSB_FIRST;
    if (tValue.UByte.MidLowByte & 0x8) {
        decodedIRData.flags = IRDATA_FLAGS_TOGGLE_BIT | IRDATA_FLAGS_IS_MSB_FIRST;
    }
    decodedIRData.protocol = RC5_CDI;

    // check for repeat
    checkForRepeatSpaceTicksAndSetFlag(RC5_CDI_MAXIMUM_REPEAT_DISTANCE / MICROS_PER_TICK);

    return true;
}

/** @}*/
#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _IR_RC5_CDI_HPP
