/*
 *  SBMInfo.ino
 *  Shows Smart Battery Info
 *
 *  If a Li-ion supply is detected, the LCD display timing for standalone usage (without serial connection) is activated.
 *
 *  Copyright (C) 2016-2023  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "SBMInfo.h"
#include <Wire.h>
#include "ADCUtils.hpp"

#define VERSION_EXAMPLE "4.3"

#if defined(__AVR__)
/*
 * Available only for some AVR CPU's like ATmega328
 * This requires 4 resistors at Pins A0 to A3, see documentation in file MeasureVoltageAndResistance.hpp
 */
#define USE_VOLTAGE_AND_RESISTANCE_MEASUREMENT
#endif // defined(__AVR__)

/*
 * The charge control pin is high as long as relative charge is below 95%.
 * It can be used to control a NPN transistor, which collector controls a high side P FET
 */
#define CHARGE_CONTROL_PIN                  9
#define CHARGE_SWITCH_OFF_PERCENTAGE       95
/*
 * The discharge control pin is high as long as relative charge is above 5% AND 3300 mV. It can be used to control a logic level FET directly.
 */
#define DISCHARGE_CONTROL_PIN              10
#define DISCHARGE_SWITCH_OFF_PERCENTAGE     5
#define DISCHARGE_SWITCH_OFF_MILLIVOLT   3300 // to be below the guessed EDV2 value
bool sCellVoltageIsBelowSwitchOffThreshold;
void checkChargeAndDischargeLimits();

#define FORCE_LCD_DISPLAY_TIMING_PIN              11 // If pulled to ground and VCC is > 4300 mV, forces slow display timing as used for standalone mode (with Li-ion supply)

#define MILLIS_BETWEEN_READING_FOR_CHANGED_VALUES   3000

/*
 * Activate the type of LCD connection you use.
 * If no element selected, only serial output is generated.
 */
#define USE_PARALLEL_LCD // Uses pin 3 to 8
//#define USE_SERIAL_LCD // Currently not available, since LiquidCrystal_I2C does not support write of byte arrays

#if defined(USE_SERIAL_LCD) && defined(USE_PARALLEL_LCD)
#error Cannot use parallel and serial LCD simultaneously
#endif
#if defined(USE_SERIAL_LCD) || defined(USE_PARALLEL_LCD)
#define USE_LCD
#endif

/*
 * LCD Display before device connected
 * 1. line: "SBMInfo" | Version |  VCC voltage
 * 2. line: Date of program compilation
 * 3. line: "Scan for device" | Scan counter
 * 4. line: Resistance or Voltage
 *
 * LCD Display after device connected
 * 1. line: "SBMInfo" | Version
 * 2. line: Date of program compilation | Manufacturer name
 * 3. line: Manufacturer date (YYYY-MM-DD) | Battery cycle count
 * 4. line: Design voltage | Design capacity
 *
 * LCD Display dynamic data
 * 1. line: Voltage | Current (negative for discharging) | optional 'H' for read error (hold)
 * 2. line: Percent of designed full charge capacity | Design capacity -> Full charge capacity
 * 3. line: Percent of relative charge
 * 3. line: Time to empty of full at current current (of line 1)
 * 4. line: Currently available (remaining) capacity
 */
/*
 * Imports and definitions for LCD
 */
#if defined(USE_SERIAL_LCD)
#include <LiquidCrystal_I2C.h> // Use an up to date library version which has the init method
#endif
#if defined(USE_PARALLEL_LCD)
#include "LiquidCrystal.h"
#endif

// Definitions required for a 2004 LCD
#define LCD_COLUMNS 20
#define LCD_ROWS 4
#define USE_2004_LCD

#if defined(USE_SERIAL_LCD)
LiquidCrystal_I2C myLCD(0x27, LCD_COLUMNS, LCD_ROWS);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif
#if defined(USE_PARALLEL_LCD)
//LiquidCrystal myLCD(2, 3, 4, 5, 6, 7);
//LiquidCrystal myLCD(7, 8, A0, A1, A2, A3);
LiquidCrystal myLCD(7, 8, 3, 4, 5, 6); // This also clears display
#endif

#if defined(USE_VOLTAGE_AND_RESISTANCE_MEASUREMENT) // Available only for some AVR CPU's like ATmega328
#define NO_PRINT_OF_RESISTOR_MEASURMENT_VOLTAGE
// Include it after LCD settings, it requires the macros USE_LCD and USE_2004_LCD to be set
#include "MeasureVoltageAndResistance.hpp"
#endif

#include "WireUtils.hpp"

/*
 * Version 4.3 - 11/2023
 * - Fixed no voltage measurement bug.
 * - Improved print and LCD display after I2C reconnection.
 *
 * Version 4.2 - 8/2023
 * - Removed compile time warnings.
 *
 * Version 4.1 - 3/2022
 * - Support for automatic discharge and charge.
 * - Improved output.
 *
 * Version 4.0 - 10/2021
 * - Integrated voltage and resistance measurement.
 * - Major improvements in I2C communication and output.
 * - Detection of disconnect.
 *
 * Version 3.3 - 3/2021
 * - Improved standalone output.
 *
 * Version 3.2 - 3/2020
 * - Improved error handling.
 * - Better prints at scanning.
 */

//#define DEBUG
/*
 *  Uses A4/A5 - the hardware I2C pins on Arduino
 */

#define DATA_BUFFER_LENGTH 32 // For Block Read
uint8_t sI2CDataBuffer[DATA_BUFFER_LENGTH];

uint8_t sI2CDeviceAddress = SBM_DEVICE_ADDRESS; // >= 128 means invalid

void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint);
void LCDClearLine(uint8_t aLineNumber);

void prettyPrintDescription(const char *aDescription);
void prettyPrintDescription(const __FlashStringHelper *aDescription);
void prettyPrintlnValueDescription(const __FlashStringHelper *aDescription);

void printHexAndBinary(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aValue);
void printlnHex(uint16_t aValue);
void printCycleCount(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aValue);
void printCapacity(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aCapacity);
void printPercentage(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aPercentage);
void printRelativeCharge(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aPercentage);

void printTime(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aMinutes);
void printBatteryMode(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aMode);
void printPackStatus(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aStatus);
void printBatteryStatus(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aStatus);
void printSpecificationInfo(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aSpecificationInfo);
void printManufacturerDate(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aDate);
void printVoltage(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aVoltageMillivolt);
void printCellVoltage(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aVoltage);
void printCurrent(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aCurrent);
void printTemperature(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aTemperature);

void printFunctionDescriptionArray(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint8_t aLengthOfArray);
void readWordAndPrint(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription);
void printSBMStaticInfo(void);
void printSBMManufacturerInfo(void);
void checkForSBMNonStandardInfoSupported();
void printSBMNonStandardInfo();
void printSBMATRateInfo(void);

void printInitialInfo();

bool testReadAndPrint();

void TogglePin(uint8_t aPinNr);

#define I2C_RETRY_DELAY_MILLIS 10 // 5 was successful for me
uint16_t readWord(uint8_t aCommand);
void writeWord(uint8_t aCommand, uint16_t aValue);
int readWordFromManufacturerAccess(uint16_t aManufacturerCommand);
uint8_t readBlock(uint8_t aCommand, uint8_t *aDataBufferPtr, uint8_t aDataBufferLength);

void MeasureVoltageAndResistance();

#define FULL_CHARGE_CAPACITY_LCD_ROW_NON_STANDARD_SUPPORTED         2 // In row 1 we have the non standard info / cell voltages
#define FULL_CHARGE_CAPACITY_LCD_ROW_NON_STANDARD_NOT_SUPPORTED     1 // Starting with row 0
#define DESIGN_CAPACITY_LCD_ROW         3 // Starting with row 0
#define REMAINING_CAPACITY_LCD_ROW      3 // Starting with row 0

#define CURRENT_LCD_ROW                 0 // Starting with row 0
#define VOLTAGE_LCD_ROW                 0 // Starting with row 0
#define DESIGN_VOLTAGE_LCD_ROW          3 // Starting with row 0
#define CELL_VOLTAGE_LCD_ROW            1 // Starting with row 0
#define ESR_LCD_ROW                     2 // Starting with row 0
#define REMAINING_TIME_LCD_ROW          2 // Starting with row 0

/*
 * Command definitions
 */
#define PRINT_VALUE_START_COLUMN 36
const char Serial_Number[] PROGMEM = "Serial number";
const char Manufacture_Date[] PROGMEM = "Manufacture date (YYYY-MM-DD)";
const char Design_Capacity[] PROGMEM = "Design capacity";
const char Design_Voltage[] PROGMEM = "Design voltage";
const char Charging_Current[] PROGMEM = "Charging current";
const char Charging_Voltage[] PROGMEM = "Charging voltage";
const char Remaining_Capacity_Alarm[] PROGMEM = "Remaining capacity alarm";
const char Specification_Info[] PROGMEM = "SBM protocol (Version / Revision)";
const char Cycle_Count[] PROGMEM = "Cycle count";
const char Max_Error_of_charge_calculation[] PROGMEM = "Max error of charge calculation";
const char RemainingTimeAlarm[] PROGMEM = "Remaining time alarm";
const char Battery_Mode[] PROGMEM = "Battery mode";
const char Pack_Status[] PROGMEM = "Pack config and status";

struct SBMFunctionDescriptionStruct sBatteryModeFuctionDescription = { BATTERY_MODE, Battery_Mode, &printBatteryMode, NULL, 0, 0 };
/*
 * Design voltage must be read before reading other capacity values for conversion of mWh to mAh
 */
struct SBMFunctionDescriptionStruct sSBMStaticFunctionDescriptionArray[] = { {
SERIAL_NUM, Serial_Number, NULL, NULL, 0, 0 }, {
MFG_DATE, Manufacture_Date, &printManufacturerDate, NULL, 0, 0 }, {
DESIGN_VOLTAGE, Design_Voltage, &printVoltage, "", 0, 0 }, {
DESIGN_CAPACITY, Design_Capacity, &printCapacity, "", 0, 0 }/* DescriptionLCD must be not NULL */, {
CHARGING_CURRENT, Charging_Current, &printCurrent, NULL, 0, 0 }, {
CHARGING_VOLTAGE, Charging_Voltage, &printVoltage, NULL, 0, 0 }, {
SPEC_INFO, Specification_Info, &printSpecificationInfo, NULL, 0, 0 }, {
CYCLE_COUNT, Cycle_Count, &printCycleCount, " cycl.", 0, 0 }, {
MAX_ERROR, Max_Error_of_charge_calculation, &printPercentage, NULL, 0, 0 }, {
REMAINING_TIME_ALARM, RemainingTimeAlarm, &printTime, NULL, 0, 0 }, {
REMAINING_CAPACITY_ALARM, Remaining_Capacity_Alarm, &printCapacity, NULL, 0, 0 } };

const char Full_Charge_Capacity[] PROGMEM = "Full charge capacity";
const char Remaining_Capacity[] PROGMEM = "Remaining capacity";
const char Relative_Charge[] PROGMEM = "Relative charge";
const char Absolute_Charge[] PROGMEM = "Absolute charge";
const char Minutes_remaining_until_empty[] PROGMEM = "Minutes remaining until empty";
const char Average_minutes_remaining_until_empty[] PROGMEM = "Average minutes remaining until empty ";
const char Minutes_remaining_for_full_charge[] PROGMEM = "Minutes remaining for full charge";
const char Battery_Status[] PROGMEM = "Battery status (BIN)";
const char Voltage[] PROGMEM = "Voltage";
const char Current[] PROGMEM = "Current";
const char Average_Current_of_last_minute[] PROGMEM = "Average current of last minute";
const char Temperature[] PROGMEM = "Temperature";

#define VOLTAGE_PRINT_DELTA_MILLIVOLT 5     // Print only if changed by two ore more mV
#define VOLTAGE_PRINT_DELTA_MILLIAMPERE 2   // Print only if changed by two ore more mA
#define VOLTAGE_PRINT_DELTA_MILLIDEGREE 100   // Print only if changed by 0.1 ore more degree

struct SBMFunctionDescriptionStruct sSBMDynamicFunctionDescriptionArray[] = { {
RELATIVE_SOC, Relative_Charge, &printRelativeCharge, NULL, 0, 0 }, { /* Must be first, because value is printed in Remaining_Capacity */
ABSOLUTE_SOC, Absolute_Charge, &printPercentage, NULL, 0, 0 }, {
FULL_CHARGE_CAPACITY, Full_Charge_Capacity, &printCapacity, "", 0, 0 }/* DescriptionLCD must be not NULL */, {
REMAINING_CAPACITY, Remaining_Capacity, &printCapacity, " remCap", 0, 0 }, {
VOLTAGE, Voltage, &printVoltage, "", VOLTAGE_PRINT_DELTA_MILLIVOLT, 0 } /* DescriptionLCD must be not NULL */, {
CURRENT, Current, &printCurrent, "", VOLTAGE_PRINT_DELTA_MILLIAMPERE, 0 } /* DescriptionLCD must be not NULL */, {
AVERAGE_CURRENT, Average_Current_of_last_minute, &printCurrent, NULL, 5, 0 } /* Print only changes of 5 mA or more */, {
TEMPERATURE, Temperature, &printTemperature, NULL, VOLTAGE_PRINT_DELTA_MILLIDEGREE, 0 }, {
RUN_TIME_TO_EMPTY, Minutes_remaining_until_empty, &printTime, " min ", 0, 0 }, {
AVERAGE_TIME_TO_EMPTY, Average_minutes_remaining_until_empty, &printTime, NULL, 0, 0 }, {
TIME_TO_FULL, Minutes_remaining_for_full_charge, &printTime, " min ", 0, 0 }, {
BATTERY_STATUS, Battery_Status, &printBatteryStatus, NULL, 0, 0 }, {
PACK_STATUS, Pack_Status, &printPackStatus, NULL, 0, 0 } };

/*
 * SBM non standard info
 * These aren't part of the standard, but work with some packs.
 */
const char Cell_1_Voltage[] PROGMEM = "Cell 1 Voltage";
const char Cell_2_Voltage[] PROGMEM = "Cell 2 Voltage";
const char Cell_3_Voltage[] PROGMEM = "Cell 3 Voltage";
const char Cell_4_Voltage[] PROGMEM = "Cell 4 Voltage";
const char State_of_Health[] PROGMEM = "State of Health";

#define NON_STANDARD_INFO_NOT_SUPPORTED     0
#define NON_STANDARD_INFO_SUPPORTED         1
#define NON_STANDARD_INFO_UNKNOWN_SUPPORTED 2   // Non standard info is NO cell voltage
// for plausi check
#define NON_STANDARD_INFO_CELL_MILLIVOLT_HIGH   5000
#define NON_STANDARD_INFO_CELL_MILLIVOLT_LOW    3000
int sNonStandardInfoSupportedByPack;
struct SBMFunctionDescriptionStruct sSBMNonStandardFunctionDescriptionArray[] = { {
CELL1_VOLTAGE, Cell_1_Voltage, &printCellVoltage, NULL, VOLTAGE_PRINT_DELTA_MILLIVOLT, 0 }, {
CELL2_VOLTAGE, Cell_2_Voltage, &printCellVoltage, NULL, VOLTAGE_PRINT_DELTA_MILLIVOLT, 0 }, {
CELL3_VOLTAGE, Cell_3_Voltage, &printCellVoltage, NULL, VOLTAGE_PRINT_DELTA_MILLIVOLT, 0 }, {
CELL4_VOLTAGE, Cell_4_Voltage, &printCellVoltage, NULL, VOLTAGE_PRINT_DELTA_MILLIVOLT, 0 }, {
STATE_OF_HEALTH, State_of_Health, NULL, NULL, 0, 0 } };

bool sCapacityModePower;                // false = current, true = power
uint16_t sDesignVoltageMillivolt;       // to retrieve last value for mWh to mA conversion
uint16_t sDesignCapacity;               // to compute relative capacity percent
uint16_t sDesignCapacityMilliAmpereHour; // for LCD output
uint16_t sRelativeChargePercent;        // for LCD output of cell voltage instead of time to full or empty
unsigned long sLastLCDTimePrintMillis;  // for LCD output of cell voltage instead of time to full or empty
int16_t sCurrentMilliampere;            // to decide if print "time to" values
uint8_t sGlobalI2CReadError;
uint8_t sLastGlobalReadError = 0;
bool sPrintOnlyChanges;                 // Is set to true after the setup / initial print
uint16_t sLastNoLoadVoltageMillivolt;   // to compute ESR
uint8_t sNoLoadVoltageMillivoltNotJustWrittenDelay; // 1 => compute ESR, 0 wait for current to be 0. To compute ESR only once but with 1 measurement delay

/*
 * This changes the display behavior to the standalone version.
 */
bool sVCCisLIION = false;

/*
 * Value depends on capacity mode
 */
const char TimeToFull_at_rate[] PROGMEM = "TimeToFull at rate";
const char TimeToEmpty_at_rate[] PROGMEM = "TimeToEmpty at rate";
const char Can_be_delivered_for_10_seconds_at_rate[] PROGMEM = "Can be delivered for 10 seconds at rate ";

struct SBMFunctionDescriptionStruct sSBMATRateFunctionDescriptionArray[] = { {
AtRateTimeToFull, TimeToFull_at_rate, &printTime, NULL, 0, 0 }, {
AtRateTimeToEmpty, TimeToEmpty_at_rate, &printTime, NULL, 0, 0 }, {
AtRateOK, Can_be_delivered_for_10_seconds_at_rate, NULL, NULL, 0, 0 } };

const char Charging_Status[] PROGMEM = "Charging Status";
const char Operation_Status[] PROGMEM = "Operation Status";
const char Pack_Voltage[] PROGMEM = "Pack Voltage";
struct SBMFunctionDescriptionStruct sSBMbq20z70FunctionDescriptionArray[] = { {
BQ20Z70_ChargingStatus, Charging_Status, &printHexAndBinary, NULL, 0, 0 }, {
BQ20Z70_OperationStatus, Operation_Status, &printHexAndBinary, NULL, 0, 0 }, {
BQ20Z70_PackVoltage, Pack_Voltage, &printVoltage, NULL, 0, 0 } };

/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/*
 * From the specs:
 * Its clock frequency range is 10 kHz to 100 kHz.
 * The charger must NOT charge a battery when it senses the resistance between the Safety Signal pin and ground to be in the range between 425 and 3150 ohm.
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

    pinMode(CHARGE_CONTROL_PIN, OUTPUT);
    pinMode(DISCHARGE_CONTROL_PIN, OUTPUT);
    digitalWrite(CHARGE_CONTROL_PIN, LOW);
    digitalWrite(DISCHARGE_CONTROL_PIN, LOW);

    pinMode(FORCE_LCD_DISPLAY_TIMING_PIN, INPUT_PULLUP);

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

#if defined(DIDR0)
    // Disable  digital input on all unused ADC channel pins to reduce power consumption
    DIDR0 = ADC0D | ADC1D | ADC2D | ADC3D;
#endif

    // set up the LCD's number of columns and rows:
    myLCD.begin(LCD_COLUMNS, LCD_ROWS); // This also clears display
    myLCD.print(F("SBMInfo " VERSION_EXAMPLE "   "));
#if defined(__AVR__)
    myLCD.print(((float) getVCCVoltageMillivolt()) / 1000, 2);
    myLCD.print(F(" V"));
#endif

    myLCD.setCursor(0, 1);
    myLCD.print(F(__DATE__));

    Wire.begin();
    Wire.setWireTimeout(); // Sets default timeout of 25 ms.
    Wire.setClock(32000); // lowest rate available is 31000
//    Wire.setClock(50000); // seen this for sony packs

    Serial.println(
            F(
                    "Configured to set charge control pin " STR(CHARGE_CONTROL_PIN) " to low above " STR(CHARGE_SWITCH_OFF_PERCENTAGE) " %"));
    Serial.println(
            F(
                    "Configured to set discharge control pin " STR(DISCHARGE_CONTROL_PIN) " to low below " STR(DISCHARGE_SWITCH_OFF_PERCENTAGE) " % or " STR(DISCHARGE_SWITCH_OFF_MILLIVOLT) " mV"));

#if defined(__AVR__)
    if (!isVCCUSBPowered() || digitalRead(FORCE_LCD_DISPLAY_TIMING_PIN) == LOW) {
        sVCCisLIION = true;
    } else {
        Serial.println(F("No Li-ion supply detected -> fast display timing"));
    }
#endif

    /*
     * Check for I2C device attached.
     * Devices which require some time to wakeup, will be found later with scanForAttachedI2CDevice()
     */
    if (checkForAttachedI2CDevice(&Serial, SBM_DEVICE_ADDRESS)) {
        sI2CDeviceAddress = SBM_DEVICE_ADDRESS;
    } else {
        Serial.println(F("Start scanning for device at I2C bus"));
        Serial.flush();
        do {
            /*
             * Check for I2C device and blink until device attached
             * This sets the I2C stop condition for the next commands
             */
            int8_t tI2CDeviceAddress = scanForAttachedI2CDevice(&Serial);
            /*
             * Handle LCD output
             */
            if (tI2CDeviceAddress == I2C_SCAN_TIMEOUT) {
                myLCD.setCursor(0, 2);
                myLCD.print(F("SDA or SCL at ground"));
            } else if (tI2CDeviceAddress == I2C_SCAN_NO_DEVICE) {
                myLCD.setCursor(0, 2);
                myLCD.print("Scan for device ");
                char tString[5];
                sprintf_P(tString, PSTR("%4u"), sScanCount);
                myLCD.print(tString);
            } else if (tI2CDeviceAddress >= 0) {
                sI2CDeviceAddress = tI2CDeviceAddress;
                myLCD.setCursor(0, 2);
                myLCD.print(F("Found device at 0x"));
                myLCD.print(sI2CDeviceAddress, HEX);
                delay(2000);
                // clear LCD line
                LCDClearLine(2);
                break;
            }

            /*
             * If no device found, do voltage and resistance measurement
             */
#if defined(USE_VOLTAGE_AND_RESISTANCE_MEASUREMENT)
            MeasureVoltageAndResistance();
#endif
            delay(500);
            TogglePin(LED_BUILTIN);

        } while (true);
    }
    LCDClearLine(3); // Clear data from MeasureVoltageAndResistance()

//    writeWord(MANUFACTURER_ACCESS, 0x0A00); // plus a read. Seen it for old (2005) Dell/Panasonic batteries
    printInitialInfo();
    sPrintOnlyChanges = true;
    digitalWrite(DISCHARGE_CONTROL_PIN, HIGH);
}

const char *sTWIErrorStrings[] = { "OK", "length to long for buffer", "address send, NACK received", "data send, NACK received",
        "other error", "timeout" };

void loop() {
//    Serial.print(F("sGlobalI2CReadError="));
//    Serial.println(sGlobalI2CReadError);
    if (sGlobalI2CReadError == 0) {
        printFunctionDescriptionArray(sSBMDynamicFunctionDescriptionArray,
                (sizeof(sSBMDynamicFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)));
        printSBMNonStandardInfo();

        // Here, all values for checking are already read in
        checkChargeAndDischargeLimits();
    } else {
        // Test connection with readWord(). This sets the sGlobalI2CReadError flag accordingly.
        readWord(CYCLE_COUNT);
    }

    /*
     * Manage display of sGlobalI2CReadError
     */
    if (sLastGlobalReadError != sGlobalI2CReadError) {
        Serial.print(F("\r\nI2C read error changed from "));
        Serial.print(sTWIErrorStrings[sLastGlobalReadError]);
        Serial.print('|');
        Serial.print(sLastGlobalReadError);
        Serial.print(F(" to "));
        Serial.print(sTWIErrorStrings[sGlobalI2CReadError]);
        Serial.print('|');
        Serial.println(sGlobalI2CReadError);
        Serial.flush();

        sLastGlobalReadError = sGlobalI2CReadError;

        if (sGlobalI2CReadError == 0) {
            // clear the display of 'H' for sGlobalI2CReadError
            myLCD.setCursor(19, 0);
            myLCD.print(' ');

            // print info again
            printInitialInfo();
            sPrintOnlyChanges = true;
        } else {
            // display 'H' for sGlobalI2CReadError
            myLCD.setCursor(19, 0);
            myLCD.print('H');
        }
    }

    delay(MILLIS_BETWEEN_READING_FOR_CHANGED_VALUES);
    TogglePin(LED_BUILTIN);
}

void TogglePin(uint8_t aPinNr) {
    if (digitalRead(aPinNr) == HIGH) {
        digitalWrite(aPinNr, LOW);
    } else {
        digitalWrite(aPinNr, HIGH);
    }
}

/*
 * Uses LCD Line 2 for display
 */
uint8_t scanForAttachedI2CDevice(void) {
    static unsigned int sScanCount = 0;
    // the next 2 statements disable TWI hangup, if SDA and SCL are connected and disconnected from ground.
#if defined(TWCR)
    TWCR = 0;
#endif
    Wire.begin();

    auto tStartMillis = millis();
    int tFoundAdress = SBM_INVALID_ADDRESS;
    // We cannot use uint_fast8_t here, since it is ambiguous parameter for beginTransmission()  on 16/32 bit CPU
    for (uint8_t tI2CAddress = 0; tI2CAddress < 127; tI2CAddress++) {
        Wire.beginTransmission(tI2CAddress);
        uint8_t tOK = Wire.endTransmission(true);
        if (tOK == 0) {
            Serial.print(F("Found I2C device attached at address: "));
            printlnHex(tI2CAddress);
            tFoundAdress = tI2CAddress;
        }
    }
    if (millis() - tStartMillis > 2000) {
        Serial.print(F("I2C Scan timeout. It seems that at least one of SCA or SCL is connected to ground."));
        myLCD.setCursor(0, 2);
        myLCD.print(F("SDA or SCL at ground"));
        tFoundAdress = SBM_INVALID_ADDRESS;
    } else if (tFoundAdress == SBM_INVALID_ADDRESS) {
        Serial.print(F("Scan found no attached I2C device - "));
        Serial.println(sScanCount);
        myLCD.setCursor(0, 2);
        myLCD.print("Scan for device ");
        char tString[5];
        sprintf_P(tString, PSTR("%4u"), sScanCount);
        myLCD.print(tString);
        sScanCount++;
    } else {
        myLCD.setCursor(0, 2);
        myLCD.print(F("Found device at 0x"));
        myLCD.print(tFoundAdress, HEX);
        delay(2000);
        // clear LCD line
        LCDClearLine(2);
    }
    return tFoundAdress;
}

void printInitialInfo() {
    sPrintOnlyChanges = false;

    /*
     * The workaround to set __FILE__ with #line __LINE__ "LightToServo.cpp" disables source output including in .lss file (-S option)
     */
    Serial.println(F("\r\n*** STATIC INFO ***"));
    /*
     * First read battery mode to set the sCapacityModePower flag to display the static values with the right unit
     */
    readWordAndPrint(&sBatteryModeFuctionDescription);

    checkForSBMNonStandardInfoSupported(); // result affects the LCD row of capacity

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
            (sizeof(sSBMDynamicFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)));

    Serial.println(F("\r\n*** DYNAMIC NON STANDARD INFO / Cell Voltages + SOH ***"));
    Serial.flush();
    printSBMNonStandardInfo();

    Serial.println(F("\r\n*** CHANGED VALUES ***"));
    Serial.flush();
}

void writeCommandWithRetry(uint8_t aCommand) {
    Wire.beginTransmission(sI2CDeviceAddress);
    Wire.write(aCommand);
    sGlobalI2CReadError = Wire.endTransmission(false); // do not send stop, is required for some packs
    /*
     * Output   0 .. success
     *          1 .. length to long for buffer
     *          2 .. address send, NACK received
     *          3 .. data send, NACK received
     *          4 .. other twi error (lost bus arbitration, bus error, ..)
     *          5 .. timeout
     */
    if (sGlobalI2CReadError == 2) {
        delay(I2C_RETRY_DELAY_MILLIS);
        // Try again
        Wire.beginTransmission(sI2CDeviceAddress);
        Wire.write(aCommand);
        sGlobalI2CReadError = Wire.endTransmission(false); // do not send stop, is required for some packs
    }
#if defined(DEBUG)
    if(sGlobalI2CReadError) {
        Serial.print(F("sGlobalI2CReadError="));
        Serial.print(sGlobalI2CReadError);
        Serial.print(F(" Command=0x"));
        Serial.println(aCommand,HEX);
    }
#endif
}

/*
 * First write the command/function address byte, then read the word value for this function
 * From the BQ spec: The processor then sends the bq2060 device address of 0001011 (bits 7Ã¢â‚¬â€œ1)
 *                   plus a R/W bit (bit 0) followed by an SMBus command code.
 */
uint16_t readWord(uint8_t aCommand) {
    writeCommandWithRetry(aCommand);
    if (sGlobalI2CReadError != 0) {
#if defined(DEBUG)
        Serial.print(F("Error at I2C access "));
        Serial.println(sGlobalI2CReadError);
#endif
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
    writeCommandWithRetry(aCommand);
    Wire.requestFrom(sI2CDeviceAddress, (uint8_t) 1);

// First read length of data
    uint8_t tLengthOfData = Wire.read();

#if defined(DEBUG)
    Serial.println();
    Serial.print(F("tLengthOfData="));
    Serial.println(tLengthOfData);
#endif
    aDataBufferLength = aDataBufferLength - 1; // we read later with tLengthOfData + 1

    if (tLengthOfData > aDataBufferLength) {
        Serial.println();
        Serial.print(F("Error: received invalid block length of "));
        Serial.print(tLengthOfData);
        Serial.print(F(" -> try "));
        Serial.println(aDataBufferLength);
        tLengthOfData = aDataBufferLength;
    }

    if (tLengthOfData > 0) {
        /*
         * It is foolproof to start a new transmission here
         */
        writeCommandWithRetry(aCommand);

#if defined(DEBUG)
        uint8_t tNumberOfDataReceived = Wire.requestFrom(sI2CDeviceAddress, (uint8_t) (tLengthOfData + 1)); // +1 since the length is read again
        Serial.print(F("tNumberOfDataReceived="));
        Serial.println(tNumberOfDataReceived);
        Serial.flush();// required to see complete output in case of crash
#else
        // request maximum aDataBufferLength + 1 (for length) bytes from device
        Wire.requestFrom(sI2CDeviceAddress, (uint8_t) (tLengthOfData + 1)); // +1 since the length is read again
#endif

        Wire.read(); // read length byte
        Wire.readBytes(aDataBufferPtr, tLengthOfData); // read at maximum aDataBufferLength bytes
    }
    return tLengthOfData;
}

void prettyPrintDescription(const __FlashStringHelper *aDescription) {
    Serial.print(aDescription);
    uint8_t tStringLength = strlen_P((const char*) aDescription);
    Serial.print(' '); // print at least one space
    for (int8_t i = 0; i < PRINT_VALUE_START_COLUMN - (tStringLength + 1); ++i) {
        Serial.print(' ');
    }
}

void prettyPrintlnValueDescription(const __FlashStringHelper *aValueDescription) {
    prettyPrintDescription(F(""));
    Serial.print(aValueDescription);
    Serial.println();
}

void prettyPrintDescription(const char *aDescription) {
    prettyPrintDescription((const __FlashStringHelper*) aDescription);
}
//
//void printPaddingForStartColumn() {
//    for (int8_t i = 0; i < PRINT_VALUE_START_COLUMN; ++i) {
//        Serial.print(' ');
//    }
//}

void printValue(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t tCurrentValue) {
    // print always 0 value
    if (!sPrintOnlyChanges || (tCurrentValue == 0 && aSBMFunctionDescription->lastPrintedValue != 0)
            || (abs(tCurrentValue - aSBMFunctionDescription->lastPrintedValue) > aSBMFunctionDescription->minDeltaValueToPrint)) {
        aSBMFunctionDescription->lastPrintedValue = tCurrentValue;

        prettyPrintDescription(aSBMFunctionDescription->Description);

        if (aSBMFunctionDescription->ValueFormatter == NULL) {
            /*
             * Default formatting, print decimal and hex value
             */
            Serial.print(tCurrentValue);
            Serial.print(F(" | 0x"));
            Serial.print(tCurrentValue, HEX);
        } else {
            aSBMFunctionDescription->ValueFormatter(aSBMFunctionDescription, tCurrentValue);
        }

        // print Hex value if value is not really plausible. We may get negative values here!
        if ((tCurrentValue & 0xFF) == 0xFF) {
            Serial.print(F(" - received 0x"));
            Serial.print(tCurrentValue, HEX);
        }
        Serial.println();
        Serial.flush();
    }
}

/*
 * Read word and print if value has changed.
 * To avoid spurious outputs check changed values 3 times.
 */
void readWordAndPrint(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription) {
    uint16_t tCurrentValue = readWord(aSBMFunctionDescription->FunctionCode);
    if (sGlobalI2CReadError == 0) {
        printValue(aSBMFunctionDescription, tCurrentValue);
    }

//    if (sGlobalI2CReadError == 0) {
//        if (sPrintOnlyChanges) {
//            if (tCurrentValue != aSBMFunctionDescription->lastPrintedValue) {
//                // check value again, maybe it was a transmit error
//                delay(33); // just guessed the value of 33
//                uint16_t tCurrentValue2 = readWord(aSBMFunctionDescription->FunctionCode);
//                if (tCurrentValue2 != aSBMFunctionDescription->lastPrintedValue) {
//                    delay(17); // just guessed the value
//                    uint16_t tCurrentValue3 = readWord(aSBMFunctionDescription->FunctionCode);
//                    if (tCurrentValue3 != aSBMFunctionDescription->lastPrintedValue) {
//                        printValue(aSBMFunctionDescription, tCurrentValue);
//                    }
//                }
//            }
//
//        } else {
//            printValue(aSBMFunctionDescription, tCurrentValue);
//        }
//    }
}

void printByteHex(uint16_t aValue) {
    Serial.print(F("0x"));
    Serial.print(aValue, HEX);
}

void printlnHex(uint16_t aValue) {
    printByteHex(aValue);
    Serial.println();
}

void printHexAndBinary(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription __attribute__((unused)), uint16_t aValue) {
    printByteHex(aValue);
    Serial.print(" | 0b");
    Serial.print(aValue, BIN);
}

void printCycleCount(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aValue) {
    Serial.println((int) aValue);
    if (aSBMFunctionDescription->DescriptionLCD != NULL) {
        myLCD.setCursor(11, 2);
        myLCD.print(aValue);
        myLCD.print(aSBMFunctionDescription->DescriptionLCD);
    }
}

void printPercentage(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aPercentage) {
    (void) aSBMFunctionDescription;
    Serial.print(aPercentage);
    Serial.print('%');
}
void printRelativeCharge(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aPercentage) {
    sRelativeChargePercent = aPercentage;
    printPercentage(aSBMFunctionDescription, aPercentage);
}

/*
 * Handles the charge and discharge pin
 * Requires preceding call to printRelativeCharge() by call to printFunctionDescriptionArray(sSBMDynamicFunctionDescriptionArray,...)
 */
void checkChargeAndDischargeLimits() {

    if (sRelativeChargePercent > CHARGE_SWITCH_OFF_PERCENTAGE) {
        digitalWrite(CHARGE_CONTROL_PIN, LOW);
    } else {
        digitalWrite(CHARGE_CONTROL_PIN, HIGH);
    }

    if (sRelativeChargePercent < DISCHARGE_SWITCH_OFF_PERCENTAGE) {
        digitalWrite(DISCHARGE_CONTROL_PIN, LOW);
    } else if (!sCellVoltageIsBelowSwitchOffThreshold) {
        digitalWrite(DISCHARGE_CONTROL_PIN, HIGH);
    }

}

const char* getCapacityModeUnit() {
    if (sCapacityModePower) {
        return StringCapacityModePower;
    }
    return StringCapacityModeCurrent;
}

/*
 * Prints only mAh on LCD
 * "28% 4400 -> 1247 mAh" on row 1 or 2 depending on sNonStandardInfoSupportedByPack
 *
 * @param aCapacity as mAh or if sCapacityModePower == true, then as mWh
 * !!! Sets sDesignCapacityMilliAmpereHour and sDesignCapacity if called for DESIGN_CAPACITY
 */
void printCapacity(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aCapacity) {
    /*
     * Print mWh and mAh
     */
    Serial.print(aCapacity);
    Serial.print(getCapacityModeUnit());
    Serial.print('h');
    uint16_t tCapacityMilliAmpereHour = aCapacity;
    if (sCapacityModePower) {
        // print also mA since changing capacity mode did not work
        Serial.print(" | ");
        tCapacityMilliAmpereHour = (aCapacity * 10000L) / sDesignVoltageMillivolt;
        Serial.print(tCapacityMilliAmpereHour);
        Serial.print(StringCapacityModeCurrent);
        Serial.print('h');
    }

    /*
     * Now print capacity only as mAh, not as mWh
     */

    if (aSBMFunctionDescription->FunctionCode == FULL_CHARGE_CAPACITY) {
        /*
         * print design capacity -> full charge capacity and percent of design capacity
         */
        if (sNonStandardInfoSupportedByPack != NON_STANDARD_INFO_SUPPORTED) {
            myLCD.setCursor(0, FULL_CHARGE_CAPACITY_LCD_ROW_NON_STANDARD_NOT_SUPPORTED);
        } else {
            myLCD.setCursor(0, FULL_CHARGE_CAPACITY_LCD_ROW_NON_STANDARD_SUPPORTED);
        }

        // "28% 4400 -> 1247 mAh" on row 1 or 2 depending on sNonStandardInfoSupportedByPack
        uint8_t tPercent = (aCapacity * 100L) / sDesignCapacity;
        myLCD.print(tPercent);
        myLCD.print("% ");
        myLCD.print(sDesignCapacityMilliAmpereHour);
        myLCD.print(" -> ");
//        if (tPercent < 100) {
//            myLCD.print(' ');
//        }
        myLCD.print(tCapacityMilliAmpereHour);
        myLCD.print(StringCapacityModeCurrent);
        myLCD.print('h');

        Serial.print(" = ");
        Serial.print(tPercent);
        Serial.print('%');
    }

    else if (aSBMFunctionDescription->DescriptionLCD != NULL) {
        /*
         * Only called here with DESIGN_CAPACITY or REMAINING_CAPACITY
         * "0 mAh 0% remCap     " on row 3
         * "4400 mAh " on row 3 for first page
         */
        // always print as mAh with trailing space
        if (aSBMFunctionDescription->FunctionCode == REMAINING_CAPACITY) {
            myLCD.setCursor(0, REMAINING_CAPACITY_LCD_ROW);
        } else {
            sDesignCapacityMilliAmpereHour = tCapacityMilliAmpereHour;
            sDesignCapacity = aCapacity;
            myLCD.setCursor(11, DESIGN_CAPACITY_LCD_ROW);
        }

        uint8_t tCharacterPrinted = 0;
        tCharacterPrinted += myLCD.print(tCapacityMilliAmpereHour);
        tCharacterPrinted += myLCD.print(StringCapacityModeCurrent) + 1;
        myLCD.print('h');
        if (aSBMFunctionDescription->FunctionCode == REMAINING_CAPACITY) {
            myLCD.print(' ');
            tCharacterPrinted += myLCD.print(sRelativeChargePercent) + 2;
            myLCD.print('%');

            Serial.print(" = ");
            Serial.print(sRelativeChargePercent);
            Serial.print('%');
        }
        tCharacterPrinted += myLCD.print(aSBMFunctionDescription->DescriptionLCD);
        if (aSBMFunctionDescription->FunctionCode == REMAINING_CAPACITY) {
            // clear rest of line
            LCDPrintSpaces(LCD_COLUMNS - tCharacterPrinted);
        }
    }
}

void printVoltage(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aVoltageMillivolt) {
    Serial.print((float) aVoltageMillivolt / 1000, 3);
    Serial.print(" V");

    if (aSBMFunctionDescription->DescriptionLCD != NULL) {
        if (aSBMFunctionDescription->FunctionCode == DESIGN_VOLTAGE) {
            // store for global use
            sDesignVoltageMillivolt = aVoltageMillivolt;
            myLCD.setCursor(0, DESIGN_VOLTAGE_LCD_ROW);
        } else /*if (aSBMFunctionDescription->FunctionCode == VOLTAGE)*/{
//            // Print 8 spaces from 0 to 7
//            myLCD.setCursor(0, 0);
//            LCDPrintSpaces(9); // clear old value from 0 to 8 incl. trailing space
            if (sCurrentMilliampere == 0) {
                sLastNoLoadVoltageMillivolt = aVoltageMillivolt;
                sNoLoadVoltageMillivoltNotJustWrittenDelay = 3;
            } else {
                // do not count below 0
                if (sNoLoadVoltageMillivoltNotJustWrittenDelay != 0) {
                    sNoLoadVoltageMillivoltNotJustWrittenDelay--;
                }
                if (sNoLoadVoltageMillivoltNotJustWrittenDelay == 1) {
                    /*
                     * Compute and print ESR only once, resolution is 0.008 ohm
                     */
                    // first compute integer value
                    int16_t tVoltageDifference = (int16_t) aVoltageMillivolt - (int16_t) sLastNoLoadVoltageMillivolt;
                    float tESROhm = (float) tVoltageDifference / sCurrentMilliampere;
                    // We read voltage before current, so it may become negative if load was detached
                    if (tESROhm > 0) {
//                Serial.print(F(" | (aVoltageMillivolt="));
//                Serial.print(aVoltageMillivolt);
//                Serial.print(F(" - sLastNoLoadVoltageMillivolt="));
//                Serial.print(sLastNoLoadVoltageMillivolt);
//                Serial.print(F(") / sCurrentMilliampere = "));
//                Serial.print(sCurrentMilliampere);
                        Serial.print(F(" | ESR = "));
                        Serial.print(tESROhm, 3);
                        Serial.print(F(" ohm"));
                        myLCD.setCursor(12, ESR_LCD_ROW);
                        myLCD.print(' ');
                        myLCD.print(tESROhm, 3);
                        myLCD.print(" \xF4"); // ohm
                    }
                }
            }

            myLCD.setCursor(0, VOLTAGE_LCD_ROW);
        }
        myLCD.print((float) aVoltageMillivolt / 1000, 3);
        myLCD.print(" V");
    }
}

/*
 * Print Cell voltages at CELL_VOLTAGE_LCD_ROW (1)
 */
void printCellVoltage(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aVoltage) {
// test for sensible value
    if (NON_STANDARD_INFO_CELL_MILLIVOLT_LOW < aVoltage && aVoltage < NON_STANDARD_INFO_CELL_MILLIVOLT_HIGH) {
        /*
         * Check for discharge switch off. We are called for more than one cell voltage here.
         */
        if (aVoltage < DISCHARGE_SWITCH_OFF_MILLIVOLT) {
            sCellVoltageIsBelowSwitchOffThreshold = true;
            digitalWrite(DISCHARGE_CONTROL_PIN, LOW);
            Serial.println(F("Stop voltage reached -> stop discharge"));
        }

        // cell voltages in row 1. 100 was not reached for a bq2084. Print if time (minutes) is not updated for more than 2 minutes.
        if (!sPrintOnlyChanges || sRelativeChargePercent == 0 || sRelativeChargePercent > 99
                || millis() - sLastLCDTimePrintMillis > 120000) {
            uint8_t tCellNumber = (aSBMFunctionDescription->FunctionCode - CELL4_VOLTAGE); // 0 to 3 for cell 4 to 1
            uint8_t tCellIndex = 3 - tCellNumber; // 0 to 3 for cell 1 to 4
            if (sDesignVoltageMillivolt < 13000 && tCellIndex < 3) { // I have seen one Sony 11V pack where only one voltage was at position CELL4_VOLTAGE
            // 3 cell voltages, each 6 characters, "3.844V"
                myLCD.setCursor(tCellIndex * 7, CELL_VOLTAGE_LCD_ROW);
                myLCD.print((float) aVoltage / 1000, 3);
                myLCD.print('V');
                // do not overwrite first character of next row 2
                if (tCellIndex < 2) {
                    myLCD.print(' ');
                }
            } else {
                // 4 cell voltages, each 4 characters
                myLCD.setCursor(tCellIndex * 5, CELL_VOLTAGE_LCD_ROW);
                myLCD.print((float) aVoltage / 1000, 2);
                myLCD.print(' ');
            }
        }

        printVoltage(aSBMFunctionDescription, aVoltage);
    } else {
        printByteHex(aVoltage);
    }
}

void printCurrent(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aCurrent) {
    char tString[6];

    int tCurrent = (int) aCurrent; // current can be negative
    if (aSBMFunctionDescription->FunctionCode == CURRENT) {
        sCurrentMilliampere = tCurrent;
    }

    Serial.print(tCurrent);
    Serial.print(" mA");
    if (aSBMFunctionDescription->DescriptionLCD != NULL) {
        // print 7 character from 12 to 18
        myLCD.setCursor(9, CURRENT_LCD_ROW);
        LCDPrintSpaces(11);
        ; // clear old value from 9 to 19 incl. leading and trailing spaces
        myLCD.setCursor(11, CURRENT_LCD_ROW);
        sprintf_P(tString, PSTR("%4d"), tCurrent);
        myLCD.print(tString);
        myLCD.print(" mA");
    }
}

void printTemperature(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription __attribute__((unused)), uint16_t aTemperature) {
    Serial.print((float) (aTemperature / 10.0) - 273.15);
    Serial.print(" C");
}

/*
 *  display 0 for values > 99h59min
 */
void printTime(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aMinutes) {

// I have seen FFFE as value, like it is described in some TI datasheet :-)
    if (aMinutes >= 0xFFFE) {
        Serial.print(F("Battery not being (dis)charged"));
    } else {

        // Hours
        uint16_t tHour = aMinutes / 60;
        if (tHour > 0) {
            Serial.print(tHour);
            Serial.print(" h ");
            if (aSBMFunctionDescription->DescriptionLCD != NULL && sCurrentMilliampere != 0) {
                // clip LCD display at 99h59min
                if (aMinutes > ((100 * 60) - 1)) {
                    tHour = 99;
                }
                myLCD.setCursor(0, REMAINING_TIME_LCD_ROW);
                LCDPrintSpaces(11);
                myLCD.setCursor(0, REMAINING_TIME_LCD_ROW);
                myLCD.print(tHour);
                myLCD.print(" h ");
            }
            aMinutes = aMinutes % 60;
        }

        // Minutes
        Serial.print(aMinutes);
        Serial.print(" min");
        if (aSBMFunctionDescription->DescriptionLCD != NULL && sCurrentMilliampere != 0) {
            if (tHour == 0) {
                myLCD.setCursor(0, REMAINING_TIME_LCD_ROW);
                LCDPrintSpaces(11);
                myLCD.setCursor(0, REMAINING_TIME_LCD_ROW);
            }

            myLCD.print(aMinutes);
            myLCD.print(aSBMFunctionDescription->DescriptionLCD);
            sLastLCDTimePrintMillis = millis();
        }
    }
}

/*
 * Format as ISO date
 */
void printManufacturerDate(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription __attribute__((unused)), uint16_t aDate) {

    int tDay = aDate & 0x1F;
    int tMonth = (aDate >> 5) & 0x0F;
    int tYear = 1980 + ((aDate >> 9) & 0x7F);
    String tDateAsString = "";
    tDateAsString = tYear;
    tDateAsString += "-";
    tDateAsString += tMonth;
    tDateAsString += "-";
    tDateAsString += tDay;
    Serial.print(tDateAsString);

    myLCD.setCursor(0, 2);
    myLCD.print(tDateAsString);
}

void printSpecificationInfo(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription __attribute__((unused)),
        uint16_t aSpecificationInfo) {
    if (aSpecificationInfo >= 0x40) {
        printByteHex(aSpecificationInfo);
        Serial.print(F(" | "));
    }
    uint8_t tSpecificationInfo = aSpecificationInfo;
    Serial.print(F("1."));

    uint8_t tVersion = tSpecificationInfo & 0xF0;
    if (tVersion == 0x10) {
        Serial.print('0');
    } else {
        Serial.print('1');
        if (tVersion > 0x20) {
            Serial.print(F(" with optional PEC support"));
        }
    }
    Serial.print(F(" / "));
    Serial.print(tSpecificationInfo & 0x0F);

}

void printPackStatus(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aStatus) {
    printHexAndBinary(aSBMFunctionDescription, aStatus);
    Serial.println();

    if (aStatus != 0) {
        // Seems to be 0 on packs not supporting this
        if (aStatus & PRESENCE) {
            prettyPrintlnValueDescription(F("- Pack inserted"));
        } else {
            prettyPrintlnValueDescription(F("- Pack not inserted"));
        }
        if (aStatus & EDV2_THRESHOLD) {
            prettyPrintlnValueDescription(F("- Voltage = EDV2"));
        } else {
            prettyPrintlnValueDescription(F("- Voltage > EDV2"));
        }
        if (aStatus & SEALED_STATE) {
            prettyPrintlnValueDescription(F("- Pack sealed"));
        } else {
            prettyPrintlnValueDescription(F("- Pack unsealed"));
        }
        if (aStatus & VDQ_DISCHARGE_QUALIFIED_FOR_CAPACITY_LEARNING) {
            prettyPrintlnValueDescription(F("- Discharge is qualified for capacity learning"));
        }
    }
}

void printBatteryMode(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aMode) {
    printHexAndBinary(aSBMFunctionDescription, aMode);
    Serial.println();

    if (aMode & INTERNAL_CHARGE_CONTROLLER) {
        prettyPrintlnValueDescription(F("- Internal Charge Controller Supported"));
    }
    if (aMode & CONDITION_FLAG) {
        prettyPrintlnValueDescription(F("- Conditioning Cycle Requested"));
    } else {
        prettyPrintlnValueDescription(F("- Battery OK"));
    }

    if (aMode & CHARGE_CONTROLLER) {
        prettyPrintlnValueDescription(F("- Charge Controller Enabled"));
    }

    if (aMode & ALARM_MODE) {
        // means the battery will not be I2C master and send alarms
        prettyPrintlnValueDescription(F("- Disable AlarmWarning broadcast to Host and Smart Battery Charger"));
    }

    if (aMode & CHARGER_MODE) {
        // means the battery will not be I2C master and not send charging info (to the charger)
        prettyPrintlnValueDescription(F("- Disable broadcasts of ChargingVoltage and ChargingCurrent to Smart Battery Charger"));
    }

    if (aMode & CAPACITY_MODE) {
        sCapacityModePower = true; // store for global use
        prettyPrintlnValueDescription(F("- Using power (10mWh) instead of current (mAh)"));
    } else {
        sCapacityModePower = false; // store for global use
    }
}

void printBatteryStatus(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint16_t aStatus) {
    printHexAndBinary(aSBMFunctionDescription, aStatus);
    Serial.println();
    /*
     * Error Bits
     */
    if (aStatus & OVER_CHARGED_ALARM__FLAG) {
        prettyPrintlnValueDescription(F("- OVER_CHARGED_ALARM"));
    }
    if (aStatus & TERMINATE_CHARGE_ALARM_FLAG) {
        prettyPrintlnValueDescription(F("- TERMINATE_CHARGE_ALARM"));
    }
    if (aStatus & OVER_TEMP_ALARM_FLAG) {
        prettyPrintlnValueDescription(F("- OVER_TEMP_ALARM"));
    }
    if (aStatus & TERMINATE_DISCHARGE_ALARM_FLAG) {
        prettyPrintlnValueDescription(F("- TERMINATE_DISCHARGE_ALARM"));
    }
    if (aStatus & REMAINING_CAPACITY_ALARM_FLAG) {
        prettyPrintlnValueDescription(F("- REMAINING_CAPACITY_ALARM"));
    }
    if (aStatus & REMAINING_TIME_ALARM_FLAG) {
        prettyPrintlnValueDescription(F("- REMAINING_TIME_ALARM_FLAG"));
    }

    /*
     * Status Bits
     */
    if (aStatus & INITIALIZED) {
        prettyPrintlnValueDescription(F("80 Initialized"));
    }
    if (aStatus & DISCHARGING) {
        prettyPrintlnValueDescription(F("40 Discharging"));
    }
    if (aStatus & FULLY_CHARGED) {
        prettyPrintlnValueDescription(F("20 Fully Charged"));
    }
    if (aStatus & FULLY_DISCHARGED) {
        prettyPrintlnValueDescription(F("10 Fully Discharged"));
    }
}

void printFunctionDescriptionArray(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, uint8_t aLengthOfArray) {
    for (uint_fast8_t i = 0; i < aLengthOfArray && sGlobalI2CReadError == 0; ++i) {
        readWordAndPrint(aSBMFunctionDescription);
        aSBMFunctionDescription++;
    }
}

void printI2CDataBuffer(uint8_t aReceivedLength) {
    Serial.write(sI2CDataBuffer, aReceivedLength);
    Serial.println();
    Serial.print(F("0x"));
    for (int i = 0; i < aReceivedLength; ++i) {
        Serial.print(sI2CDataBuffer[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}
/*
 * Sometimes charging current and charging voltage are only available if external enable connector is present (or enable pin is tight low)
 */
void printSBMStaticInfo(void) {
    uint8_t tReceivedLength = 0;

    prettyPrintDescription(F("Manufacturer Name"));
    tReceivedLength = readBlock(MFG_NAME, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    printI2CDataBuffer(tReceivedLength);

    myLCD.setCursor(12, 1);
    if (tReceivedLength > 8) {
        tReceivedLength = 8;
    }
    myLCD.write(sI2CDataBuffer, tReceivedLength);

    prettyPrintDescription(F("Chemistry"));
    tReceivedLength = readBlock(CELL_CHEM, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    printI2CDataBuffer(tReceivedLength);

    prettyPrintDescription(F("Manufacturer Data"));
    tReceivedLength = readBlock(MANUFACTURER_DATA, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    printI2CDataBuffer(tReceivedLength);

    prettyPrintDescription(F("Device Name"));
    tReceivedLength = readBlock(DEV_NAME, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    printI2CDataBuffer(tReceivedLength);

    printFunctionDescriptionArray(sSBMStaticFunctionDescriptionArray,
            (sizeof(sSBMStaticFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)));

    if (sVCCisLIION) {
        delay(5000);
    } else {
        delay(1000);
    }
    myLCD.clear();
}

void printSBMManufacturerInfo(void) {

    uint16_t tType = readWordFromManufacturerAccess(TI_Device_Type);
    prettyPrintDescription(F("Device Type"));
    Serial.print(tType);
    Serial.print(F(" | "));
    printlnHex(tType);

    uint16_t tVersion = readWordFromManufacturerAccess(TI_Firmware_Version);
// check if read valid data
    if (tType != tVersion) {

        prettyPrintDescription(F("Firmware Version"));
        Serial.print((uint8_t) (tVersion >> 8), HEX);
        Serial.print(".");
        Serial.println((uint8_t) tVersion, HEX);

        if (tType == 2083) {
            prettyPrintlnValueDescription(F("bq2085"));
            Serial.print(F("End of Discharge Voltage Level"));
            uint16_t tLevel = readWordFromManufacturerAccess(BQ2084_EDV_level);
            Serial.print(((float) tLevel) / 1000, 3);
            Serial.println(" V");
            Serial.println();

        } else if (tType == 2072) {
            prettyPrintlnValueDescription(F("bq8011/bq8015)"));

        } else if (tType == 2084) {
            prettyPrintlnValueDescription(F("bq2084"));
            Serial.print(F("End of Discharge Voltage Level"));
            uint16_t tLevel = readWordFromManufacturerAccess(BQ2084_EDV_level);
            Serial.print(((float) tLevel) / 1000, 3);
            Serial.println(" V");
            Serial.println();

        } else {
            if (tType == 0x700) {
                prettyPrintlnValueDescription(F("bq20z70, bq20z75, bq29330"));
                printFunctionDescriptionArray(sSBMbq20z70FunctionDescriptionArray,
                        (sizeof(sSBMbq20z70FunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)));
            } else if (tType == 0x451) {
                prettyPrintlnValueDescription(F("bq20z45-R1"));
                printFunctionDescriptionArray(sSBMbq20z70FunctionDescriptionArray,
                        (sizeof(sSBMbq20z70FunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)));
            }

            prettyPrintDescription(F("Hardware Version"));
            printlnHex(readWordFromManufacturerAccess(BQ20Z70_Hardware_Version));
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

/*
 * sets sNonStandardInfoSupportedByPack
 */
void checkForSBMNonStandardInfoSupported() {
    /*
     * Initialize: Very simple check if non standard info supported by pack
     * compare value of CELL1_VOLTAGE and CELL2_VOLTAGE and check if CELL1_VOLTAGE is between 3000 and 5000
     */
    uint16_t tCurrentValue = readWord(sSBMNonStandardFunctionDescriptionArray[0].FunctionCode);
    uint16_t tCurrentValue1 = readWord(sSBMNonStandardFunctionDescriptionArray[1].FunctionCode);
    Serial.print(F("Non standard info is "));

    if (tCurrentValue == tCurrentValue1) {
        sNonStandardInfoSupportedByPack = NON_STANDARD_INFO_NOT_SUPPORTED;
        Serial.println(F("not supported"));
        sGlobalI2CReadError = 0; // Most likely we have read errors here, so reset flag
    } else if ((tCurrentValue < NON_STANDARD_INFO_CELL_MILLIVOLT_LOW) || (NON_STANDARD_INFO_CELL_MILLIVOLT_HIGH < tCurrentValue)) {
        sNonStandardInfoSupportedByPack = NON_STANDARD_INFO_UNKNOWN_SUPPORTED;
        Serial.println(F("supported, but not cell voltages"));
    } else {
        sNonStandardInfoSupportedByPack = NON_STANDARD_INFO_SUPPORTED;
        Serial.println(F("supported"));
    }
}

/*
 * Up to 4 cell voltages + SOH
 */
void printSBMNonStandardInfo() {
    if (sNonStandardInfoSupportedByPack != NON_STANDARD_INFO_NOT_SUPPORTED) {
        sCellVoltageIsBelowSwitchOffThreshold = false;
        printFunctionDescriptionArray(sSBMNonStandardFunctionDescriptionArray,
                (sizeof(sSBMNonStandardFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)));
        sGlobalI2CReadError = 0; // I have seen some read errors here, so better reset flag
    }
}

void printSBMATRateInfo(void) {
    writeWord(AtRate, 100);
    prettyPrintDescription(F("Setting AT rate to"));
    Serial.print(F("100"));
    Serial.print(getCapacityModeUnit());
    long tmA;
    if (sCapacityModePower) {
        // print also mA since changing capacity mode did not work
        Serial.print(" | ");
        tmA = (100 * 10000L) / sDesignVoltageMillivolt;
        Serial.print(tmA);
        Serial.print(StringCapacityModeCurrent);
    }
    Serial.println();
    delay(20); // > 5 ms for bq2085-V1P3
    readWordAndPrint(&sSBMATRateFunctionDescriptionArray[0]);

    writeWord(AtRate, -100);
    prettyPrintDescription(F("Setting AT rate to"));
    Serial.print(F("-100"));
    Serial.print(getCapacityModeUnit());
    if (sCapacityModePower) {
        // print also mA since changing capacity mode did not work
        Serial.print(" | ");
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        Serial.print(-tmA); // !s initialized above, but compiler complains :-(
        Serial.print(StringCapacityModeCurrent);
    }
    Serial.println();

    delay(20); // > 5 ms for bq2085-V1P3
    for (uint_fast8_t i = 1; i < (sizeof(sSBMATRateFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)); ++i) {
        readWordAndPrint(&sSBMATRateFunctionDescriptionArray[i]);
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
        Serial.print(F("Test read address "));
        printByteHex(sRegisterAddress);
        Serial.print('=');
        printlnHex(tTestResult);

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

void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint) {
    for (uint_fast8_t i = 0; i < aNumberOfSpacesToPrint; ++i) {
        myLCD.print(' ');
    }
}

void LCDClearLine(uint8_t aLineNumber) {
    myLCD.setCursor(0, aLineNumber);
    LCDPrintSpaces(20);
    myLCD.setCursor(0, aLineNumber);
}
