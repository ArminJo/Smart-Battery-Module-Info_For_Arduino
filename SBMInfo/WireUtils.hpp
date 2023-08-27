/*
 * WireUtils.hpp
 *
 *  Assumes, that Wire timeout is enabled e.g. with:
 *  Wire.setWireTimeout(); // Sets default timeout of 25 ms.
 *
 *  Copyright (C) 2023  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include <Wire.h>

#ifndef _WIRE_UTILS_HPP
#define _WIRE_UTILS_HPP

void printWireError(Print *aSerial, uint8_t aWireReturnCode);
#define I2C_SCAN_NO_DEVICE  -1
#define I2C_SCAN_TIMEOUT    -5
int8_t scanForAttachedI2CDevice(Print *aSerial, uint8_t aI2CAddressToStartWith = 0);
bool checkForAttachedI2CDevice(Print *aSerial, uint8_t aI2CDeviceAddress);

unsigned int sScanCount = 0;

uint8_t sGlobalWireReturnCode; // global return code / error flag set by all endTransmission() commands

uint8_t writeI2CByte(uint8_t aI2CDeviceAddress, uint8_t aByteToWrite) {
    Wire.beginTransmission(aI2CDeviceAddress);
    Wire.write(aByteToWrite);
    sGlobalWireReturnCode = Wire.endTransmission(); // send stop
    return sGlobalWireReturnCode;
}

uint16_t readI2CWordLSBFirst(uint8_t aI2CDeviceAddress) {
    Wire.requestFrom(aI2CDeviceAddress, (uint8_t) 2);
    uint8_t tLSB = Wire.read();
    uint8_t tMSB = Wire.read();
    return (uint16_t) tLSB | (((uint16_t) tMSB) << 8);
}

uint16_t readI2CWordMSBFirst(uint8_t aI2CDeviceAddress) {
    Wire.requestFrom(aI2CDeviceAddress, (uint8_t) 2);
    uint8_t tMSB = Wire.read();
    uint8_t tLSB = Wire.read();
    return (uint16_t) tLSB | (((uint16_t) tMSB) << 8);
}

/*
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 *          5 .. timeout
 * @return true if error happened
 */
void printWireError(Print *aSerial, uint8_t aWireReturnCode) {
    switch (aWireReturnCode) {
    case 1: // too long for transmit buffer
        aSerial->print(F("Too long for transmit buffer"));
        break;
    case 2: // received NACK on transmit of address
        aSerial->print(F("Address sent, NACK received. Device not connected?"));
        break;
    case 3: // received NACK on transmit of data
        aSerial->print(F("Address sent, NACK received."));
        break;
    case 4: // other error
        aSerial->print(F("Other error"));
        break;
    case 5: // other error
        aSerial->print(F("Timeout while waiting until twi is ready"));
        break;
    }
}

/*
 * Check if I2C device with given address is attached.
 * @return true if attached device found
 */
bool checkForAttachedI2CDevice(Print *aSerial, uint8_t aI2CDeviceAddress) {
    do {
        Wire.beginTransmission(aI2CDeviceAddress);
        if (Wire.getWireTimeoutFlag()) {
            aSerial->println(F("Timeout accessing I2C bus. Wait for bus becoming available"));
            Wire.clearWireTimeoutFlag();
            delay(100);
        } else {
            break;
        }
    } while (true);

    sGlobalWireReturnCode = Wire.endTransmission();
    bool tRetCode;
    if (sGlobalWireReturnCode == 0) {
        aSerial->print(F("Found attached"));
        tRetCode = true;
    } else {
        aSerial->print(F("Transmission error code: \""));
        aSerial->print(sGlobalWireReturnCode);
        aSerial->print(F(" | "));
        printWireError(&Serial, sGlobalWireReturnCode);
        aSerial->print(F("\", while checking for"));
        tRetCode = false;
    }
    aSerial->print(F(" I2C device at address 0x"));
    aSerial->println(aI2CDeviceAddress, HEX);
    aSerial->flush();
    return tRetCode;
}

/*
 * Scans address 0 to 127
 * @return I2C address from 0 to 127, where transmission was successful
 *         -1, I2C_SCAN_NO_DEVICE
 *         -5, I2C_SCAN_TIMEOUT
 */
int8_t scanForAttachedI2CDevice(Print *aSerial, uint8_t aI2CAddressToStartWith) {
// the next 2 statements disable TWI hangup, if SDA and SCL are connected and disconnected from ground.
#if defined(TWCR)
    TWCR = 0;
#endif
    Wire.begin();

    auto tStartMillis = millis();
    // We cannot use uint_fast8_t here, since it is ambiguous parameter for beginTransmission()  on 16/32 bit CPU
    for (uint8_t tI2CAddress = aI2CAddressToStartWith; tI2CAddress < 127; tI2CAddress++) {
        Wire.beginTransmission(tI2CAddress);
        uint8_t tOK = Wire.endTransmission(true);
        if (tOK == 0) {
            aSerial->print(F("Found I2C device attached at address 0x"));
            aSerial->println(tI2CAddress, HEX);
            return tI2CAddress;
        }
    }
    sScanCount++;

    if (millis() - tStartMillis > 2000) {
        aSerial->print(F("I2C Scan timeout. It seems that at least one of SCA or SCL is connected to ground"));
        return I2C_SCAN_TIMEOUT;
    }

    aSerial->print(F("Scan found no attached I2C device"));
    aSerial->println(sScanCount);
    return I2C_SCAN_NO_DEVICE;
}

#endif // _WIRE_UTILS_HPP
