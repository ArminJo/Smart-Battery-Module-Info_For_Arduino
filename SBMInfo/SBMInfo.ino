/*
 *  SBMInfo.ino
 *  Shows Smart Battery Info
 *
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  https://github.com/ArminJo/Smart-Battery-Module-Info_For_Arduino
 *
 *  SBMInfo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "SBMInfo.h"
#include <Wire.h>
#include "LiquidCrystal.h"

#define LCD_COLUMNS 20
#define LCD_ROWS 4

#define VERSION_EXAMPLE "4.0.0"

/*
 * Version 4.0.0 - 5/2021
 * Major improvements in I2C communication and output.
 * Detection of disconnect.
 *
 * Version 3.3.0 - 3/2021
 * - Improved standalone output.
 *
 * Version 3.2.0 - 3/2020
 * - Improved error handling.
 *
 * Version 3.1.1 - 3/2020
 * - Better prints at scanning.
 */

//#define DEBUG
/*
 *  Uses A4/A5 - the hardware I2C pins on Arduino
 */

#define DATA_BUFFER_LENGTH 32
uint8_t sI2CDataBuffer[DATA_BUFFER_LENGTH];

uint8_t sI2CDeviceAddress = SBM_DEVICE_ADDRESS; // >= 128 means invalid

//LiquidCrystal myLCD(2, 3, 4, 5, 6, 7);
LiquidCrystal myLCD(7, 8, A0, A1, A2, A3);

bool printBinary(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aValue);
bool printSigned(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aValue);
bool printCapacity(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aCapacity);
bool printPercentage(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aPercentage);

bool printTime(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aMinutes);
bool printBatteryMode(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aMode);
bool printBatteryStatus(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aStatus);
bool printManufacturerDate(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aDate);
bool printVoltage(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aVoltage);
bool printCurrent(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aCurrent);
bool printTemperature(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aTemperature);

void printFunctionDescriptionArray(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint8_t aLengthOfArray,
        bool aOnlyPrintIfValueChanged);
void readWordAndPrint(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, bool aOnlyPrintIfValueChanged);
void printSBMStaticInfo(void);
void printSBMManufacturerInfo(void);
void printSBMNonStandardInfo(bool aOnlyPrintIfValueChanged);
void printSBMATRateInfo(void);

void printInitialInfo();

bool checkForAttachedI2CDevice(uint8_t aI2CDeviceAddress);
uint8_t scanForAttachedI2CDevice(void);
bool testReadAndPrint();

void TogglePin(uint8_t aPinNr);
uint16_t readWord(uint8_t aCommand);
void writeWord(uint8_t aCommand, uint16_t aValue);
int readWordFromManufacturerAccess(uint16_t aManufacturerCommand);
uint8_t readBlock(uint8_t aCommand, uint8_t *aDataBufferPtr, uint8_t aDataBufferLength);

/*
 * Command definitions
 */
const char Serial_Number[] PROGMEM = "Serial Number: ";
const char Manufacture_Date[] PROGMEM = "Manufacture Date (YYYY-MM-DD):";
const char Design_Capacity[] PROGMEM = "Design Capacity: ";
const char Design_Voltage[] PROGMEM = "Design Voltage: ";
const char Charging_Current[] PROGMEM = "Charging Current: ";
const char Charging_Voltage[] PROGMEM = "Charging Voltage: ";
const char Remaining_Capacity_Alarm[] PROGMEM = "Remaining Capacity Alarm: ";
const char Specification_Info[] PROGMEM = "Specification Info: ";
const char Cycle_Count[] PROGMEM = "Cycle Count: ";
const char Max_Error_of_charge_calculation[] PROGMEM = "Max Error of charge calculation: ";
const char RemainingTimeAlarm[] PROGMEM = "RemainingTimeAlarm: ";
const char Battery_Mode[] PROGMEM = "Battery Mode (BIN): 0b";
const char Pack_Status[] PROGMEM = "Pack Status (BIN): ";

struct SBMFunctionDescriptionStruct sBatteryModeFuctionDescription = { BATTERY_MODE, Battery_Mode, &printBatteryMode };
/*
 * Design voltage must be read before reading other capacity values for conversion of mWh to mAh
 */
struct SBMFunctionDescriptionStruct sSBMStaticFunctionDescriptionArray[] = { {
SERIAL_NUM, Serial_Number }, {
MFG_DATE, Manufacture_Date, &printManufacturerDate }, {
DESIGN_VOLTAGE, Design_Voltage, &printVoltage }, {
DESIGN_CAPACITY, Design_Capacity, &printCapacity }, {
CHARGING_CURRENT, Charging_Current, &printCurrent }, {
CHARGING_VOLTAGE, Charging_Voltage, &printVoltage }, {
SPEC_INFO, Specification_Info }, {
CYCLE_COUNT, Cycle_Count }, {
MAX_ERROR, Max_Error_of_charge_calculation, &printPercentage }, {
REMAINING_TIME_ALARM, RemainingTimeAlarm, &printTime }, {
REMAINING_CAPACITY_ALARM, Remaining_Capacity_Alarm, &printCapacity }, {
PACK_STATUS, Pack_Status, &printBinary } };

#define INDEX_OF_DESIGN_CAPACITY 3 // to retrieve last value for comparison with full charge capacity

const char Full_Charge_Capacity[] PROGMEM = "Full Charge Capacity: ";
const char Remaining_Capacity[] PROGMEM = "Remaining Capacity: ";
const char Relative_Charge[] PROGMEM = "Relative Charge: ";
const char Absolute_Charge[] PROGMEM = "Absolute Charge: ";
const char Minutes_remaining_until_empty[] PROGMEM = "Minutes remaining until empty: ";
const char Average_minutes_remaining_until_empty[] PROGMEM = "Average minutes remaining until empty: ";
const char Minutes_remaining_for_full_charge[] PROGMEM = "Minutes remaining for full charge: ";
const char Battery_Status[] PROGMEM = "Battery Status (BIN): 0b";
const char Voltage[] PROGMEM = "Voltage: ";
const char Current[] PROGMEM = "Current: ";
const char Average_Current_of_last_minute[] PROGMEM = "Average Current of last minute: ";
const char Temperature[] PROGMEM = "Temperature: ";

struct SBMFunctionDescriptionStruct sSBMDynamicFunctionDescriptionArray[] = { {
FULL_CHARGE_CAPACITY, Full_Charge_Capacity, &printCapacity }, {
REMAINING_CAPACITY, Remaining_Capacity, &printCapacity, " remCapacity" }, {
RELATIVE_SOC, Relative_Charge, &printPercentage, " rel Charge " }, {
ABSOLUTE_SOC, Absolute_Charge, &printPercentage }, {
VOLTAGE, Voltage, &printVoltage, " " }/* must not != NULL*/, {
CURRENT, Current, &printCurrent, " " } /* must not != NULL*/, {
AverageCurrent, Average_Current_of_last_minute, &printCurrent }, {
TEMPERATURE, Temperature, &printTemperature }, {
RUN_TIME_TO_EMPTY, Minutes_remaining_until_empty, &printTime, " min to Empty " }, {
AVERAGE_TIME_TO_EMPTY, Average_minutes_remaining_until_empty, &printTime }, {
TIME_TO_FULL, Minutes_remaining_for_full_charge, &printTime, " min to Full " }, {
BATTERY_STATUS, Battery_Status, &printBatteryStatus } };

/*
 * These aren't part of the standard, but work with some packs.
 */
const char Cell_1_Voltage[] PROGMEM = "Cell 1 Voltage: ";
const char Cell_2_Voltage[] PROGMEM = "Cell 2 Voltage: ";
const char Cell_3_Voltage[] PROGMEM = "Cell 3 Voltage: ";
const char Cell_4_Voltage[] PROGMEM = "Cell 4 Voltage: ";
const char State_of_Health[] PROGMEM = "State of Health: ";

#define NON_STANDARD_INFO_NOT_INTIALIZED    0
#define NON_STANDARD_INFO_SUPPORTED         1
#define NON_STANDARD_INFO_NOT_SUPPORTED     2
int sNonStandardInfoSupportedByPack = NON_STANDARD_INFO_NOT_INTIALIZED;
struct SBMFunctionDescriptionStruct sSBMNonStandardFunctionDescriptionArray[] = { {
CELL1_VOLTAGE, Cell_1_Voltage, &printVoltage }, {
CELL2_VOLTAGE, Cell_2_Voltage, &printVoltage }, {
CELL3_VOLTAGE, Cell_3_Voltage, &printVoltage }, {
CELL4_VOLTAGE, Cell_4_Voltage, &printVoltage }, {
STATE_OF_HEALTH, State_of_Health } };

bool sCapacityModePower = false; // false = current, true = power
uint16_t sDesignVoltage; // to retrieve last value for mWh to mA conversion
uint16_t sCurrent; // to decide if print "time to" values
uint8_t sGlobalReadError;
uint8_t sLastGlobalReadError;

/*
 * Value depends on capacity mode
 */
const char TimeToFull_at_rate[] PROGMEM = "TimeToFull at rate: ";
const char TimeToEmpty_at_rate[] PROGMEM = "TimeToEmpty at rate: ";
const char Can_be_delivered_for_10_seconds_at_rate[] PROGMEM = "Can be delivered for 10 seconds at rate: ";

struct SBMFunctionDescriptionStruct sSBMATRateFunctionDescriptionArray[] = { {
AtRateTimeToFull, TimeToFull_at_rate, &printTime }, {
AtRateTimeToEmpty, TimeToEmpty_at_rate, &printTime }, {
AtRateOK, Can_be_delivered_for_10_seconds_at_rate } };

const char Charging_Status[] PROGMEM = "Charging Status: ";
const char Operation_Status[] PROGMEM = "Operation Status: ";
const char Pack_Voltage[] PROGMEM = "Pack Voltage: ";
struct SBMFunctionDescriptionStruct sSBMbq20z70FunctionDescriptionArray[] = { {
BQ20Z70_ChargingStatus, Charging_Status, &printBinary }, {
BQ20Z70_OperationStatus, Operation_Status, &printBinary }, {
BQ20Z70_PackVoltage, Pack_Voltage, &printVoltage } };

/*
 * From the specs:
 * Its clock frequency range is 10 kHz to 100 kHz.
 * The charger must NOT charge a battery when it senses the resistance between the Safety Signal pin and ground to be in the range between 425 and 3150 ohms.
 * E.g. NiMH battery may use a 103AT thermistor for this.
 * Only Read Word, Write Word, Read Block or Write Block protocol is used.
 * bq2084 spec: With SMBus, the most-significant bit (MSB) of a data byte is transmitted first.
 * BUT: 16 bit values like DesignVoltage are interpreted as if LSB were sent first.
 */
/*
 * Program starts here
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // Shutdown SPI, timers, and ADC
    PRR = (1 << PRSPI) | (1 << PRTIM1) | (1 << PRTIM2) | (1 << PRADC);
    // Disable  digital input on all unused ADC channel pins to reduce power consumption
    DIDR0 = ADC0D | ADC1D | ADC2D | ADC3D;

    // set up the LCD's number of columns and rows:
    myLCD.begin(LCD_COLUMNS, LCD_ROWS);
    myLCD.print(F("SBMInfo " VERSION_EXAMPLE));
    myLCD.setCursor(0, 1);
    myLCD.print(F(__DATE__));
    /*
     * The workaround to set __FILE__ with #line __LINE__ "LightToServo.cpp" disables source output including in .lss file (-S option)
     */

    Wire.begin();
    Wire.setWireTimeout(); // Sets default timeout of 25 ms.
    Wire.setClock(32000); // lowest rate available is 31000
//    Wire.setClock(50000); // seen this for sony packs

    delay(100); // wait for 100 ms after enable pin set to low

    /*
     * Check for I2C device and blink until device attached
     * This sets the I2C stop condition for the next commands
     */
    if (checkForAttachedI2CDevice(SBM_DEVICE_ADDRESS)) {
        sI2CDeviceAddress = SBM_DEVICE_ADDRESS;
    } else {
        Serial.println(F("Start scanning for device at I2C bus"));
        Serial.flush();
        do {
            sI2CDeviceAddress = scanForAttachedI2CDevice();
            delay(500);
            TogglePin(LED_BUILTIN);
        } while (sI2CDeviceAddress == SBM_INVALID_ADDRESS);
    }

//    writeWord(MANUFACTURER_ACCESS, 0x0A00); // plus a read. Seen it for old (2005) Dell/Panasonic batteries
    printInitialInfo();
}

void loop() {
//    Serial.print(F("sGlobalReadError="));
//    Serial.println(sGlobalReadError);
    if (sGlobalReadError == 0) {
        printFunctionDescriptionArray(sSBMDynamicFunctionDescriptionArray,
                (sizeof(sSBMDynamicFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), true);
        printSBMNonStandardInfo(true);
        myLCD.setCursor(19, 0);
        myLCD.print(' ');
    } else {
        // Test connection with readWord(). This sets the flag accordingly.
        readWord(MANUFACTURER_ACCESS);
    }

    /*
     * Manage display of sGlobalReadError
     */
    if (sLastGlobalReadError != sGlobalReadError) {
        sLastGlobalReadError = sGlobalReadError;
        Serial.print(F("\r\nsGlobalReadError changed to: "));
        Serial.println(sGlobalReadError);
        Serial.flush();
        if (sGlobalReadError == 0) {
            printInitialInfo();

//            myLCD.setCursor(19, 0);
//            myLCD.print(' ');
        } else {
            myLCD.setCursor(19, 0);
            myLCD.print('H');
        }
    }

    delay(3000);
    TogglePin(LED_BUILTIN);
}

void TogglePin(uint8_t aPinNr) {
    if (digitalRead(aPinNr) == HIGH) {
        digitalWrite(aPinNr, LOW);
    } else {
        digitalWrite(aPinNr, HIGH);
    }
}

bool checkForAttachedI2CDevice(uint8_t aStandardDeviceAddress) {
    do {
        Wire.beginTransmission(aStandardDeviceAddress);
        if (Wire.getWireTimeoutFlag()) {
            Serial.println(F("Timeout accessing I2C bus. Wait for bus becoming available"));
            Wire.clearWireTimeoutFlag();
            delay(100);
        } else {
            break;
        }
    } while (true);

    uint8_t tRetCode = Wire.endTransmission();
    if (tRetCode == 0) {
        Serial.print(F("Found attached I2C device at 0x"));
        Serial.println(aStandardDeviceAddress, HEX);
        Serial.flush();
        return true;
    } else {
        Serial.print(F("Transmission error code="));
        if (tRetCode == 2) {
            Serial.print(F("\"address send, NACK received. Device not connected?\""));
        } else {
            Serial.print(tRetCode);
        }
        Serial.print(F(" while looking for device at default address 0x"));
        Serial.println(aStandardDeviceAddress, HEX);
        return false;
    }
}

uint8_t scanForAttachedI2CDevice(void) {
    static unsigned int sScanCount = 0;
    int tFoundAdress = SBM_INVALID_ADDRESS;
    for (uint8_t tI2CAddress = 0; tI2CAddress < 127; tI2CAddress++) {
        Wire.beginTransmission(tI2CAddress);
        uint8_t tOK = Wire.endTransmission(true);
        if (tOK == 0) {
            Serial.print(F("Found I2C device attached at address: 0x"));
            Serial.println(tI2CAddress, HEX);
            tFoundAdress = tI2CAddress;
        }
    }
    if (tFoundAdress == SBM_INVALID_ADDRESS) {
        Serial.print(F("Scan found no attached I2C device - "));
        Serial.println(sScanCount);
        myLCD.setCursor(0, 2);
        myLCD.print("Scan for device ");
        char tString[4];
        sprintf_P(tString, PSTR("%3u"), sScanCount);
        myLCD.print(tString);
        sScanCount++;
    } else {
        myLCD.setCursor(0, 2);
        myLCD.print(F("Found device at 0x"));
        myLCD.print(tFoundAdress, HEX);
        delay(1000);
        // clear LCD line
        myLCD.setCursor(0, 3);
        myLCD.print("                    ");
    }
    return tFoundAdress;
}

void printInitialInfo() {
    Serial.println(F("\r\n*** STATIC INFO ***"));
    /*
     * First read battery mode to set the sCapacityModePower flag to display the static values with the right unit
     */
    readWordAndPrint(&sBatteryModeFuctionDescription, false);

    Serial.flush(); // in order not to interfere with i2c timing
    printSBMStaticInfo();

    Serial.println(F("\r\n*** MANUFACTURER INFO ***"));
    Serial.flush();
    printSBMManufacturerInfo();

    Serial.println(F("\r\n*** RATE TEST INFO ***"));
    Serial.flush();
    printSBMATRateInfo();

    Serial.println(F("\r\n*** DYNAMIC INFO ***"));
    Serial.flush();
    printFunctionDescriptionArray(sSBMDynamicFunctionDescriptionArray,
            (sizeof(sSBMDynamicFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), false);

    Serial.println(F("\r\n*** DYNAMIC NON STANDARD INFO ***"));
    Serial.flush();
    printSBMNonStandardInfo(false);

    Serial.println(F("\r\n*** CHANGED VALUES ***"));
    Serial.flush();
}

/*
 * First write the command/function address byte, then read the word value for this function
 * From the BQ spec: The processor then sends the bq2060 device address of 0001011 (bits 7–1)
 *                   plus a R/W bit (bit 0) followed by an SMBus command code.
 */
uint16_t readWord(uint8_t aCommand) {
    Wire.beginTransmission(sI2CDeviceAddress);
    Wire.write(aCommand);
    sGlobalReadError = Wire.endTransmission(false); // do not send stop, is required for some packs
    /*
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     *          5 .. timeout
     */
    if (sGlobalReadError != 0) {
#ifdef DEBUG
        Serial.print(F("Error at I2C access: "));
        Serial.println(sGlobalReadError);
#endif
//        Wire.endTransmission(true);
        return 0xFFFF;
    } else {
        Wire.requestFrom(sI2CDeviceAddress, (uint8_t) 2);
        uint8_t tLSB = Wire.read();
        uint8_t tMSB = Wire.read();
        return (int) tLSB | (((int) tMSB) << 8);
    }
}

/*
 *  Write the command/function address byte and the word value for this function LSB first, because the BQ reads and send LSByte first!
 */
void writeWord(uint8_t aCommand, uint16_t aValue) {
    Wire.beginTransmission(sI2CDeviceAddress);
    Wire.write((aValue >> 8) & 0xFF);
    Wire.write(aCommand);
    Wire.write(aValue & 0xFF);
    Wire.endTransmission();
}

/*
 * Write manufacturer command to manufacturer access function/register and read the result for that manufacturer command
 */
int readWordFromManufacturerAccess(uint16_t aManufacturerCommand) {
    writeWord(MANUFACTURER_ACCESS, aManufacturerCommand);
    return readWord(MANUFACTURER_ACCESS);
}

uint8_t readBlock(uint8_t aCommand, uint8_t *aDataBufferPtr, uint8_t aDataBufferLength) {
    Wire.beginTransmission(sI2CDeviceAddress);
    Wire.write(aCommand);
    Wire.endTransmission(false);
    Wire.requestFrom(sI2CDeviceAddress, (uint8_t) 1);

// First read length of data
    uint8_t tLengthOfData = Wire.read();

#ifdef DEBUG
    Serial.println();
    Serial.print(F("tLengthOfData="));
    Serial.println(tLengthOfData);
#endif

    if (tLengthOfData > aDataBufferLength) {
        Serial.println();
        Serial.print(F("Error: received invalid block length of: "));
        Serial.print(tLengthOfData);
        Serial.print(F(" -> try "));
        Serial.println(aDataBufferLength);
        tLengthOfData = aDataBufferLength;
    }

    if (tLengthOfData > 0) {
        /*
         * It is foolproof to start a new transmission here
         */
        Wire.beginTransmission(sI2CDeviceAddress);
        Wire.write(aCommand);
        Wire.endTransmission(false);
#ifdef DEBUG
        uint8_t tNumberOfDataReceived = Wire.requestFrom(sI2CDeviceAddress, (uint8_t) (tLengthOfData + 1)); // +1 since the length is read again
        Serial.print(F("tNumberOfDataReceived="));
        Serial.println(tNumberOfDataReceived);
        Serial.flush(); // required to see complete output in case of crash
#else
        tLengthOfData = Wire.requestFrom(sI2CDeviceAddress, (uint8_t) (tLengthOfData + 1)) - 1; // +1 -1 since the length is read again
#endif

        Wire.read();
        Wire.readBytes(aDataBufferPtr, tLengthOfData);
    }
    return tLengthOfData;
}

/*
 * checks if description string is in progmen
 */
void printDescriptionPGM(const char *aDescription) {
    Serial.print((const __FlashStringHelper*) aDescription);
}

void printValue(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t tCurrentValue) {
    {
        bool tSomethingWasPrinted = true;
        if (aSBMFunctionDescription->ValueFormatter == NULL) {
            Serial.print((const __FlashStringHelper*) aSBMFunctionDescription->Description);
            Serial.print(tCurrentValue);
            Serial.print(F(" | 0x"));
            Serial.print(tCurrentValue, HEX);
            aSBMFunctionDescription->lastValue = tCurrentValue;
        } else {
            tSomethingWasPrinted = aSBMFunctionDescription->ValueFormatter(aSBMFunctionDescription, tCurrentValue);
        }
        aSBMFunctionDescription->lastValue = tCurrentValue;

        // print Hex value if value is not really plausible. We may get negative values here!
        if ((tCurrentValue & 0xFF) == 0xFF) {
            Serial.print(F(" - received 0x"));
            Serial.print(tCurrentValue, HEX);
        }
        if (tSomethingWasPrinted) {
            Serial.println();
        }

        Serial.flush();
    }
}

/*
 * Read word and print if value has changed.
 * To avoid spurious outputs check changed values 3 times.
 */
void readWordAndPrint(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, bool aOnlyPrintIfValueChanged) {
    uint16_t tCurrentValue = readWord(aSBMFunctionDescription->FunctionCode);

    if (sGlobalReadError == 0) {
        if (aOnlyPrintIfValueChanged) {
            if (tCurrentValue != aSBMFunctionDescription->lastValue) {
                // check value again, maybe it was a transmit error
                delay(33); // just guessed the value of 33
                uint16_t tCurrentValue2 = readWord(aSBMFunctionDescription->FunctionCode);
                if (tCurrentValue2 != aSBMFunctionDescription->lastValue) {
                    delay(17); // just guessed the value
                    uint16_t tCurrentValue3 = readWord(aSBMFunctionDescription->FunctionCode);
                    if (tCurrentValue3 != aSBMFunctionDescription->lastValue) {
                        printValue(aSBMFunctionDescription, tCurrentValue);
                    }
                }
            }

        } else {
            printValue(aSBMFunctionDescription, tCurrentValue);
        }
    }
}

bool printBinary(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aValue) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);
    Serial.print("0b");
    Serial.print(aValue, BIN);
    return true;
}

bool printSigned(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aValue) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);
    Serial.println((int) aValue);
    return true;
}

const char* getCapacityModeUnit() {
    if (sCapacityModePower) {
        return StringCapacityModePower;
    }
    return StringCapacityModeCurrent;

}

bool printCapacity(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aCapacity) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);
    Serial.print(aCapacity);
    Serial.print(getCapacityModeUnit());
    Serial.print('h');
    if (sCapacityModePower) {
        // print also mA since changing capacity mode did not work
        Serial.print(" | ");
        aCapacity = (aCapacity * 10000L) / sDesignVoltage;
        Serial.print(aCapacity);
        Serial.print(StringCapacityModeCurrent);
        Serial.print('h');
    }

    if (aDescription->FunctionCode == FULL_CHARGE_CAPACITY) {
        /*
         * print design capacity -> full charge capacity and percent of design capacity
         */
        myLCD.setCursor(0, 1);
        uint16_t DesignCapacity = sSBMStaticFunctionDescriptionArray[INDEX_OF_DESIGN_CAPACITY].lastValue;
        uint8_t tPercent = (aCapacity * 100L) / DesignCapacity;
        myLCD.print(tPercent);
        myLCD.print("% ");
        myLCD.print(DesignCapacity);
        if (sCapacityModePower) {
            myLCD.print('0'); // here we have units of 10 mWh
        }
        myLCD.print("->");
        myLCD.print(aCapacity);
        myLCD.print((getCapacityModeUnit() + 1));
        myLCD.print('h');

        Serial.print(" = ");
        Serial.print(tPercent);
        Serial.print('%');
    }

    if (aDescription->DescriptionLCD != NULL) {
        // always print as mAh with trailing space
        myLCD.setCursor(0, 3);
        myLCD.print(aCapacity);
        myLCD.print(getCapacityModeUnit());
        myLCD.print('h');
        myLCD.print(aDescription->DescriptionLCD);
    }
    return true;
}

/*
 * Only used for Relative_Charge
 */
bool printPercentage(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aPercentage) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);
    Serial.print(aPercentage);
    Serial.print('%');
    if (aDescription->DescriptionLCD != NULL) {
        myLCD.setCursor(0, 2);
        myLCD.print(aPercentage);
        myLCD.print('%');
        myLCD.print(aDescription->DescriptionLCD);
    }
    return true;
}

/*
 *  display 0 for values > 99h59min
 */
bool printTime(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aMinutes) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);

    myLCD.setCursor(0, 1);
// I have seen FFFE as value!
    if (aMinutes >= 0xFFFE) {
        Serial.print(F("Battery not being (dis)charged"));
    } else {
        uint16_t tHour;
        if (aMinutes >= 60) {
            tHour = aMinutes / 60;
            Serial.print(tHour);
            Serial.print(" h ");
            if (aDescription->DescriptionLCD != NULL && sCurrent != 0) {
                // clip LCD display at 99h59min
                if (aMinutes > ((100 * 60) - 1)) {
                    tHour = 99;
                }
                // clear LCD line
                myLCD.print("                    ");
                myLCD.setCursor(0, 1);
                myLCD.print(tHour);
                myLCD.print(" h ");
            }
            aMinutes = aMinutes % 60;
        }
        Serial.print(aMinutes);
        Serial.print(" min");
        if (aDescription->DescriptionLCD != NULL && sCurrent != 0) {
            if (tHour == 0) {
                // clear LCD line
                myLCD.print("                    ");
                myLCD.setCursor(0, 1);
            }

            myLCD.print(aMinutes);
            myLCD.print(aDescription->DescriptionLCD);
        }
    }
    return true;
}

/*
 * Print only if changed by two ore more mV
 */
bool printVoltage(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aVoltage) {
    if (aVoltage < aDescription->lastValue - 1 || aDescription->lastValue + 1 < aVoltage) {
        Serial.print((const __FlashStringHelper*) aDescription->Description);
        Serial.print((float) aVoltage / 1000, 3);
        Serial.print(" V");
        if (aDescription->DescriptionLCD != NULL) {
            // print 8 character from 0 to 7
            myLCD.setCursor(0, 0);
            myLCD.print("         "); // clear old value from 0 to 8 incl. trailing space
            myLCD.setCursor(0, 0);
            myLCD.print((float) aVoltage / 1000, 3);
            myLCD.print(" V");
        }
        // store for global use
        if (aDescription->FunctionCode == DESIGN_VOLTAGE) {
            sDesignVoltage = aVoltage;
        }
        return true;
    }
    return false;
}

/*
 * Print only if changed by two ore more mA
 */
bool printCurrent(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aCurrent) {
    sCurrent = aCurrent;

    if (aCurrent < aDescription->lastValue - 1 || aDescription->lastValue + 1 < aCurrent) {
        Serial.print((const __FlashStringHelper*) aDescription->Description);
        Serial.print((int) aCurrent);
        Serial.print(" mA");
        if (aDescription->DescriptionLCD != NULL) {
            // print 7 character from 12 to 18
            myLCD.setCursor(9, 0);
            myLCD.print("           "); // clear old value from 9 to 19 incl. leading and trailing spaces
            myLCD.setCursor(12, 0);
            myLCD.print((int) aCurrent);
            myLCD.print(" mA");
        }
        return true;
    }
    return false;
}

/*
 * Print only if changed by more than 0.1 C
 */
bool printTemperature(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aTemperature) {
    if (aTemperature < aDescription->lastValue - 100 || aDescription->lastValue + 100 < aTemperature) {
        Serial.print((const __FlashStringHelper*) aDescription->Description);
        Serial.print((float) (aTemperature / 10.0) - 273.15);
        Serial.print(" C");
        return true;
    }
    return false;
}

/*
 * Format as ISO date
 */
bool printManufacturerDate(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aDate) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);

    int tDay = aDate & 0x1F;
    int tMonth = (aDate >> 5) & 0x0F;
    int tYear = 1980 + ((aDate >> 9) & 0x7F);
    String tDateAsString = " ";
    tDateAsString += tYear;
    tDateAsString += "-";
    tDateAsString += tMonth;
    tDateAsString += "-";
    tDateAsString += tDay;
    Serial.print(tDateAsString);
    return true;
}

bool printBatteryMode(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aMode) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);

    Serial.println(aMode, BIN);

    if (aMode & INTERNAL_CHARGE_CONTROLLER) {
        Serial.println(F("- Internal Charge Controller Supported"));
    }
    if (aMode & CONDITION_FLAG) {
        Serial.println(F("- Conditioning Cycle Requested"));
    } else {
        Serial.println(F("- Battery OK"));
    }
    if (aMode & CHARGE_CONTROLLER) {
        Serial.println(F("- Charge Controller Enabled"));
    }

    if (aMode & ALARM_MODE) {
        // means the battery will not be I2C master and send alarms
        Serial.println(F("- Disable AlarmWarning broadcast to Host and Smart Battery Charger"));
    }

    if (aMode & CHARGER_MODE) {
        // means the battery will not be I2C master and not send charging info (to the charger)
        Serial.println(F("- Disable broadcasts of ChargingVoltage and ChargingCurrent to Smart Battery Charger"));
    }

    if (aMode & CAPACITY_MODE) {
        sCapacityModePower = true; // store for global use
        Serial.println(F("- Using power (10mWh) instead of current (mAh)"));
    }
    return true;
}

bool printBatteryStatus(struct SBMFunctionDescriptionStruct *aDescription, uint16_t aStatus) {
    Serial.print((const __FlashStringHelper*) aDescription->Description);
    Serial.println(aStatus, BIN);
    /*
     * Error Bits
     */
    if (aStatus & OVER_CHARGED_ALARM__FLAG) {
        Serial.println(F("- OVER_CHARGED_ALARM"));
    }
    if (aStatus & TERMINATE_CHARGE_ALARM_FLAG) {
        Serial.println(F("- TERMINATE_CHARGE_ALARM"));
    }
    if (aStatus & OVER_TEMP_ALARM_FLAG) {
        Serial.println(F("- OVER_TEMP_ALARM"));
    }
    if (aStatus & TERMINATE_DISCHARGE_ALARM_FLAG) {
        Serial.println(F("- TERMINATE_DISCHARGE_ALARM"));
    }
    if (aStatus & REMAINING_CAPACITY_ALARM_FLAG) {
        Serial.println(F("- REMAINING_CAPACITY_ALARM"));
    }
    if (aStatus & REMAINING_TIME_ALARM_FLAG) {
        Serial.println(F("- REMAINING_TIME_ALARM_FLAG"));
    }

    /*
     * Status Bits
     */
    if (aStatus & INITIALIZED) {
        Serial.println(F("- Initialized"));
    }
    if (aStatus & DISCHARGING) {
        Serial.println(F("- Discharging"));
    }
    if (aStatus & FULLY_CHARGED) {
        Serial.println(F("- Fully Charged"));
    }
    if (aStatus & FULLY_DISCHARGED) {
        Serial.println(F("- Fully Discharged"));
    }
    return true;
}

void printFunctionDescriptionArray(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint8_t aLengthOfArray,
        bool aOnlyPrintIfValueChanged) {
    for (uint8_t i = 0; i < aLengthOfArray && sGlobalReadError == 0; ++i) {
        readWordAndPrint(aSBMFunctionDescription, aOnlyPrintIfValueChanged);
        aSBMFunctionDescription++;
    }
}

/*
 * Sometimes charging current and charging voltage are only available if external enable connector is present (or enable pin is tight low)
 */
void printSBMStaticInfo(void) {
    uint8_t tReceivedLength = 0;

    Serial.print(F("Manufacturer Name: "));
    tReceivedLength = readBlock(MFG_NAME, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    Serial.write(sI2CDataBuffer, tReceivedLength);
    Serial.println("");

    Serial.print(F("Chemistry: "));
    tReceivedLength = readBlock(CELL_CHEM, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    Serial.write(sI2CDataBuffer, tReceivedLength);
    Serial.println("");

    Serial.print(F("Manufacturer Data: "));
    tReceivedLength = readBlock(MANUFACTURER_DATA, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    Serial.write(sI2CDataBuffer, tReceivedLength);
    Serial.write(" | 0x");
    for (int i = 0; i < tReceivedLength; ++i) {
        Serial.print(sI2CDataBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");

    Serial.print(F("Device Name: "));
    tReceivedLength = readBlock(DEV_NAME, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    Serial.write(sI2CDataBuffer, tReceivedLength);
    Serial.println("");

    printFunctionDescriptionArray(sSBMStaticFunctionDescriptionArray,
            (sizeof(sSBMStaticFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), false);
}

void printSBMManufacturerInfo(void) {

    uint16_t tType = readWordFromManufacturerAccess(TI_Device_Type);
    Serial.print(F("Device Type: "));
    Serial.print(tType);
    Serial.print(F(" | 0x"));
    Serial.println(tType, HEX);

    uint16_t tVersion = readWordFromManufacturerAccess(TI_Firmware_Version);
// check if read valid data
    if (tType != tVersion) {

        Serial.print(F("Firmware Version: "));
        Serial.print((uint8_t) (tVersion >> 8), HEX);
        Serial.print(".");
        Serial.println((uint8_t) tVersion, HEX);

        if (tType == 2083) {
            Serial.print(F("Controller IC identified by device type: "));
            Serial.println(F("bq2085"));
            Serial.print(F("End of Discharge Voltage Level: "));
            uint16_t tLevel = readWordFromManufacturerAccess(BQ2084_EDV_level);
            Serial.print(((float) tLevel) / 1000, 3);
            Serial.println(" V");
            Serial.println();

        } else if (tType == 2084) {
            Serial.print(F("Controller IC identified by device type: "));
            Serial.println(F("bq2084"));
            Serial.print(F("End of Discharge Voltage Level: "));
            uint16_t tLevel = readWordFromManufacturerAccess(BQ2084_EDV_level);
            Serial.print(((float) tLevel) / 1000, 3);
            Serial.println(" V");
            Serial.println();

        } else {
            if (tType == 0x700) {
                Serial.print(F("Controller IC identified by device type: "));
                Serial.println(F("bq20z70, bq20z75, bq29330"));
                printFunctionDescriptionArray(sSBMbq20z70FunctionDescriptionArray,
                        (sizeof(sSBMbq20z70FunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), false);
            } else if (tType == 0x451) {
                Serial.print(F("Controller IC identified by device type: "));
                Serial.println(F("bq20z45-R1"));
                printFunctionDescriptionArray(sSBMbq20z70FunctionDescriptionArray,
                        (sizeof(sSBMbq20z70FunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), false);
            }

            Serial.print(F("Hardware Version: 0x"));
            Serial.println(readWordFromManufacturerAccess(BQ20Z70_Hardware_Version), HEX);

            Serial.println();
        }

        /*
         * Status
         */
        Serial.print(F("Manufacturer Status (BIN): 0b"));
        uint8_t tStatus = readWordFromManufacturerAccess(BQ20Z70_Manufacturer_Status) >> 8;
        Serial.println(tStatus, BIN);
        Serial.print(F("- FET Status "));
        Serial.println(tStatus >> 6);
        Serial.print(F("- State: 0b"));
        tStatus = tStatus & 0x0F;
        Serial.println(tStatus, BIN);
        if (tStatus == 0x01) {
            Serial.println(F(" - Normal Discharge"));
        } else if (tStatus == 0x05) {
            Serial.println(F(" - Charge"));
        } else if (tStatus == 0x07) {
            Serial.println(F(" - Charge Termination"));
        } else if (tStatus == 0x0C) {
            Serial.println(F(" - Battery Failure"));
        } else if (tStatus == 0x09) {
            Serial.println(F(" - Permanent Failure"));
            uint8_t tPFStatus = readWordFromManufacturerAccess(BQ20Z70_PFStatus) >> 8;
            Serial.print(F(" - PFStatus: 0b"));
            Serial.println(tPFStatus, BIN);
        } else if (tStatus == 0x0F) {
            Serial.println(F(" - Battery Pack removed"));
        }
    }
}

void printSBMNonStandardInfo(bool aOnlyPrintIfValueChanged) {
    if (sNonStandardInfoSupportedByPack != NON_STANDARD_INFO_NOT_SUPPORTED && sGlobalReadError == 0) {
        if (sNonStandardInfoSupportedByPack == NON_STANDARD_INFO_NOT_INTIALIZED) {
            /*
             * very simple check if non standard info supported by pack
             * compare value of CELL1_VOLTAGE and CELL2_VOLTAGE
             */
            uint16_t tCurrentValue = readWord(sSBMNonStandardFunctionDescriptionArray[0].FunctionCode);
            uint16_t tCurrentValue1 = readWord(sSBMNonStandardFunctionDescriptionArray[1].FunctionCode);
            if (tCurrentValue == tCurrentValue1) {
                sNonStandardInfoSupportedByPack = NON_STANDARD_INFO_NOT_SUPPORTED;
                return;
            } else {
                sNonStandardInfoSupportedByPack = NON_STANDARD_INFO_SUPPORTED;
            }
        }

        printFunctionDescriptionArray(sSBMNonStandardFunctionDescriptionArray,
                (sizeof(sSBMNonStandardFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), aOnlyPrintIfValueChanged);
        sGlobalReadError = 0; // I have seen some read errors here
    }
}

void printSBMATRateInfo(void) {
    writeWord(AtRate, 100);
    Serial.print(F("Setting AT rate to 100"));
    Serial.print(getCapacityModeUnit());
    long tmA;
    if (sCapacityModePower) {
        // print also mA since changing capacity mode did not work
        Serial.print(" | ");
        tmA = (100 * 10000L) / sDesignVoltage;
        Serial.print(tmA);
        Serial.print(StringCapacityModeCurrent);
    }
    Serial.println();
    delay(20); // > 5 ms for bq2085-V1P3
    readWordAndPrint(&sSBMATRateFunctionDescriptionArray[0], false);

    writeWord(AtRate, -100);
    Serial.print(F("Setting AT rate to -100"));
    Serial.print(getCapacityModeUnit());
    if (sCapacityModePower) {
        // print also mA since changing capacity mode did not work
        Serial.print(" | ");
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        Serial.print(-tmA);
        Serial.print(StringCapacityModeCurrent);
    }
    Serial.println();

    delay(20); // > 5 ms for bq2085-V1P3
    for (uint8_t i = 1; i < (sizeof(sSBMATRateFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)); ++i) {
        readWordAndPrint(&sSBMATRateFunctionDescriptionArray[i], false);
    }
}

/*
 * Not used yet
 * Read battery mode as test value and wait for plausible result
 * @return true if read was successful i.e. != 0xFFFF
 */
bool testReadAndPrint() {
    static uint8_t sRegisterAddress = MANUFACTURER_ACCESS; // 0x00
    static uint8_t sTestCounter = 0;

    uint16_t tTestResult = readWord(sRegisterAddress);

    if (tTestResult == 0xFFFF) {
        Serial.print(F("Test read address 0x"));
        Serial.print(sRegisterAddress, HEX);
        Serial.print(F("=0x"));
        Serial.println(tTestResult, HEX);

        myLCD.setCursor(0, 2);
        myLCD.print(F("Test read adr. 0x"));
        myLCD.print(sRegisterAddress, HEX);
        myLCD.setCursor(0, 3);
        myLCD.print(F("0x"));
        myLCD.print(tTestResult, HEX);
        char tString[5];
        sprintf_P(tString, PSTR("%4u"), sTestCounter);
        myLCD.print(tString);

        sTestCounter++;
        sRegisterAddress = (sRegisterAddress + 1) & 0x0F;

        return false;
    }
    return true;
}
