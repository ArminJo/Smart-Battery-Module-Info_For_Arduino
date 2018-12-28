/*
 *  SBMInfo.ino
 *  Shows Smart Battery Info
 *
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
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
#include <SoftwareWire.h>
#include <LiquidCrystal.h>

#define LCD_COLUMNS 20
#define LCD_ROWS 4

#define VERSION "3.0"

/*
 *  Corresponds to A4/A5 - the hardware I2C pins on Arduino
 */
#define SDA_PIN A4
#define SCL_PIN A5
SoftwareWire SBMConnection(SDA_PIN, SCL_PIN);

#define DATA_BUFFER_LENGTH 32
uint8_t sI2CDataBuffer[DATA_BUFFER_LENGTH];

uint8_t sI2CDeviceAddress;

LiquidCrystal myLCD(2, 3, 4, 5, 6, 7);

void printBinary(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aValue);
void printSigned(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aValue);
void printCapacity(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aCapacity);
void printPercentage(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aPercentage);

void printTime(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aMinutes);
void printBatteryMode(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aMode);
void printBatteryStatus(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aStatus);
void printManufacturerDate(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aDate);
void printVoltage(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aVoltage);
void printCurrent(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aCurrent);
void printTemperature(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aTemperature);

void printFunctionDescriptionArray(struct SBMFunctionDescriptionStruct * aSBMFunctionDescription, uint8_t aLengthOfArray,
bool aOnlyPrintIfValueChanged);
void printSMBStaticInfo(void);
void printSMBManufacturerInfo(void);
void printSMBNonStandardInfo(bool aOnlyPrintIfValueChanged);
void printSMBATRateInfo(void);

bool checkForAttachedI2CDevice(uint8_t aI2CDeviceAddress);
int scanForAttachedI2CDevice(void);

void TogglePin(uint8_t aPinNr);
int readWord(uint8_t aFunction);
void writeWord(uint8_t aFunction, uint16_t aValue);

// Pin 13 has an LED connected on most Arduino boards.
const int LED_PIN = 13;

/*
 * Command definitions
 */
const char Serial_Number[] = "Serial Number: ";
const char Manufacture_Date[] = "Manufacture Date (YYYY-MM-DD):";
const char Design_Capacity[] PROGMEM = "Design Capacity: ";
const char Design_Voltage[] PROGMEM = "Design Voltage: ";
const char Charging_Current[] PROGMEM  = "Charging Current: ";
const char Charging_Voltage[] PROGMEM = "Charging Voltage: ";
const char Remaining_Capacity_Alarm[] PROGMEM = "Remaining Capacity Alarm: ";

#define INDEX_OF_DESIGN_VOLTAGE 3 // to retrieve last value for mWh to mA conversion
struct SBMFunctionDescriptionStruct sSBMStaticFunctionDescriptionArray[] = { {
SERIAL_NUM, Serial_Number }, {
MFG_DATE, Manufacture_Date, &printManufacturerDate }, {
DESIGN_CAPACITY, Design_Capacity, &printCapacity }, {
DESIGN_VOLTAGE, Design_Voltage, &printVoltage }, {
CHARGING_CURRENT, Charging_Current, &printCurrent }, {
CHARGING_VOLTAGE, Charging_Voltage, &printVoltage }, {
SPEC_INFO, "Specification Info: " }, {
CYCLE_COUNT, "Cycle Count: " }, {
MAX_ERROR, "Max Error of charge calculation (%): " }, {
REMAINING_TIME_ALARM, "RemainingTimeAlarm: ", &printTime }, {
REMAINING_CAPACITY_ALARM, Remaining_Capacity_Alarm, &printCapacity }, {
BATTERY_MODE, "Battery Mode (BIN): 0b", &printBatteryMode }, {
PACK_STATUS, "Pack Status (BIN): ", &printBinary } };

const char Full_Charge_Capacity[] PROGMEM = "Full Charge Capacity: ";
const char Remaining_Capacity[] PROGMEM = "Remaining Capacity: ";

const char Voltage[] PROGMEM = "Voltage: ";
const char Current[] PROGMEM = "Current: ";
const char Average_Current_of_last_minute[] PROGMEM = "Average Current of last minute: ";

struct SBMFunctionDescriptionStruct sSBMDynamicFunctionDescriptionArray[] = { {
FULL_CHARGE_CAPACITY, Full_Charge_Capacity, &printCapacity }, {
REMAINING_CAPACITY, Remaining_Capacity, &printCapacity, "Capacity " }, {
RELATIVE_SOC, "Relative Charge: ", &printPercentage, " rel Charge " }, {
ABSOLUTE_SOC, "Absolute Charge(%): ", NULL, "% Abs Charge " }, {
RUN_TIME_TO_EMPTY, "Minutes remaining until empty: ", &printTime }, {
AVERAGE_TIME_TO_EMPTY, "Average minutes remaining until empty: ", &printTime, " min to Empty " }, {
TIME_TO_FULL, "Minutes remaining for full charge: ", &printTime, " min to Full " }, {
BATTERY_STATUS, "Battery Status (BIN): 0b", &printBatteryStatus }, {
VOLTAGE, Voltage, &printVoltage, "Voltage: " }, {
CURRENT, Current, &printCurrent, "Current: " }, {
AverageCurrent, Average_Current_of_last_minute, &printCurrent }, {
TEMPERATURE, "Temperature: ", &printTemperature } };

/*
 * These aren't part of the standard, but work with some packs.
 */
const char Cell_1_Voltage[] PROGMEM = "Cell 1 Voltage: ";
const char Cell_2_Voltage[] PROGMEM = "Cell 2 Voltage: ";
const char Cell_3_Voltage[] PROGMEM = "Cell 3 Voltage: ";
const char Cell_4_Voltage[] PROGMEM = "Cell 4 Voltage: ";

int nonStandardInfoSupportedByPack = 0; // 0 not initialized, 1 supported, > 1 not supported
struct SBMFunctionDescriptionStruct sSBMNonStandardFunctionDescriptionArray[] = { {
CELL1_VOLTAGE, Cell_1_Voltage, &printVoltage }, {
CELL2_VOLTAGE, Cell_2_Voltage, &printVoltage }, {
CELL3_VOLTAGE, Cell_3_Voltage, &printVoltage }, {
CELL4_VOLTAGE, Cell_4_Voltage, &printVoltage }, {
STATE_OF_HEALTH, "State of Health: " } };

bool sCapacityModePower = false; // false = current, true = power
uint16_t sDesignVoltage; // to retrieve last value for mWh to mA conversion

/*
 * Value depends on capacity mode
 */
struct SBMFunctionDescriptionStruct sSBMATRateFunctionDescriptionArray[] = { {
AtRateTimeToFull, "TimeToFull at rate: ", &printTime }, {
AtRateTimeToEmpty, "TimeToEmpty at rate: ", &printTime }, {
AtRateOK, "Can be delivered for 10 seconds at rate: " }, };

const char Pack_Voltage[] PROGMEM = "Pack Voltage: ";
struct SBMFunctionDescriptionStruct sSBMbq20z70FunctionDescriptionArray[] = { {
BQ20Z70_ChargingStatus, "Charging Status: ", &printBinary }, {
BQ20Z70_OperationStatus, "Operation Status: ", &printBinary }, {
BQ20Z70_PackVoltage, Pack_Voltage, &printVoltage } };

/*
 * Program starts here
 */

void setup() {
// initialize the digital pin as an output.
    pinMode(LED_PIN, OUTPUT);

    // Shutdown SPI and TWI, timers, and ADC
    PRR = (1 << PRSPI) | (1 << PRTWI) | (1 << PRTIM1) | (1 << PRTIM2) | (1 << PRADC);
    // Disable  digital input on all unused ADC channel pins to reduce power consumption
    DIDR0 = ADC0D | ADC1D | ADC2D | ADC3D;

    Serial.begin(115200);
    while (!Serial) {
        ; // wait for Leonardo enumeration, others continue immediately
    }

    // set up the LCD's number of columns and rows:
    myLCD.begin(LCD_COLUMNS, LCD_ROWS);
    Serial.println(F("START SBMInfo\r\nVersion " VERSION " from " __DATE__));
    myLCD.print(F("SBMInfo " VERSION));
    myLCD.setCursor(0, 1);
    myLCD.print(F(__DATE__));
    /*
     * The workaround to set __FILE__ with #line __LINE__ "LightToServo.cpp" disables source output including in .lss file (-S option)
     */

    SBMConnection.begin();
    SBMConnection.setClock(25000);

    /*
     * Check for I2C device and blink until device attached
     */
    if (!checkForAttachedI2CDevice(SBM_DEVICE_ADDRESS)) {
        int tDeviceAttached;
        do {
            tDeviceAttached = scanForAttachedI2CDevice();
            delay(500);
            TogglePin(LED_PIN);
        } while (tDeviceAttached < 0);
    }

    uint16_t tVoltage;
    do {
        tVoltage = readWord(VOLTAGE);
        delay(500);
        TogglePin(LED_PIN);
    } while (tVoltage == 0xFFFF);

    Serial.println(F("\r\n*** STATIC INFO ***"));
    Serial.flush(); // in order not to interfere with i2c timing
    printSMBStaticInfo();

    Serial.println(F("\r\n*** MANUFACTURER INFO ***"));
    Serial.flush();
    printSMBManufacturerInfo();

    Serial.println(F("\r\n*** RATE TEST INFO ***"));
    Serial.flush();
    printSMBATRateInfo();

    Serial.println(F("\r\n*** DYNAMIC INFO ***"));
    Serial.flush();
    printFunctionDescriptionArray(sSBMDynamicFunctionDescriptionArray,
            (sizeof(sSBMDynamicFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), false);

    Serial.println(F("\r\n*** DYNAMIC NON STANDARD INFO ***"));
    Serial.flush();
    printSMBNonStandardInfo(false);

    Serial.println(F("\r\n*** CHANGED VALUES ***"));
    Serial.flush();
}

void loop() {
    printFunctionDescriptionArray(sSBMDynamicFunctionDescriptionArray,
            (sizeof(sSBMDynamicFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), true);
    printSMBNonStandardInfo(true);
    delay(3000);
}

void TogglePin(uint8_t aPinNr) {
    if (digitalRead(aPinNr) == HIGH) {
        digitalWrite(aPinNr, LOW);
    } else {
        digitalWrite(aPinNr, HIGH);
    }
}

bool checkForAttachedI2CDevice(uint8_t aStandardDeviceAddress) {
    SBMConnection.beginTransmission(aStandardDeviceAddress);
    uint8_t tOK = SBMConnection.endTransmission();
    if (tOK == SOFTWAREWIRE_NO_ERROR) {
        Serial.print(F("Found attached I2C device at 0x"));
        Serial.println(aStandardDeviceAddress, HEX);
        sI2CDeviceAddress = SBM_DEVICE_ADDRESS;
        return true;
    } else {
        Serial.print(F("Transmission error code="));
        Serial.println(tOK);
        return false;
    }
}

int sScanCount = 0;
int scanForAttachedI2CDevice(void) {
    int tFoundAdress = -1;
    for (uint8_t i = 0; i < 127; i++) {
        SBMConnection.beginTransmission(i);
        uint8_t tOK = SBMConnection.endTransmission(true);
        if (tOK == SOFTWAREWIRE_NO_ERROR) {
            Serial.print(F("Found I2C device attached at address: 0x"));
            Serial.println(i, HEX);
            tFoundAdress = i;
        }
    }
    if (tFoundAdress < 0) {
        Serial.print(F("Scan found no attached I2C device - "));
        Serial.println(sScanCount);
        myLCD.setCursor(0, 3);
        // print the number of seconds since reset:
        myLCD.print("Scan for device ");
        myLCD.print(sScanCount);
        sScanCount++;
    } else {
        // clear LCD line
        myLCD.setCursor(0, 3);
        myLCD.print("                    ");
        sI2CDeviceAddress = tFoundAdress;
    }
    return tFoundAdress;
}

int readWord(uint8_t aFunction) {
    cli();
    SBMConnection.beginTransmission(sI2CDeviceAddress);
    SBMConnection.write(aFunction);
    SBMConnection.requestFrom(sI2CDeviceAddress, (uint8_t) 2);
    sei();
    uint8_t tLSB = SBMConnection.read();
    uint8_t tMSB = SBMConnection.read();
    return (int) tLSB | (((int) tMSB) << 8);
}

void writeWord(uint8_t aFunction, uint16_t aValue) {
    cli();
    SBMConnection.beginTransmission(sI2CDeviceAddress);
    SBMConnection.write(aFunction);
    SBMConnection.write(aValue & 0xFF);
    SBMConnection.write((aValue >> 8) & 0xFF);
    SBMConnection.endTransmission();
    sei();
}

int readWordFromManufacturerAccess(uint16_t aCommand) {
    writeWord(MANUFACTURER_ACCESS, aCommand);
    return readWord(MANUFACTURER_ACCESS);
}

uint8_t readBlock(uint8_t aCommand, uint8_t* aDataBufferPtr, uint8_t aDataBufferLength) {
    cli();
    SBMConnection.beginTransmission(sI2CDeviceAddress);
    SBMConnection.write(aCommand);
    SBMConnection.requestFrom(sI2CDeviceAddress, (uint8_t) 1);
// First read length of data
    uint8_t tLengthOfData = SBMConnection.read();

    tLengthOfData++; // since the length is read again
    if (tLengthOfData > aDataBufferLength) {
        tLengthOfData = aDataBufferLength;
    }
    SBMConnection.requestFrom(sI2CDeviceAddress, tLengthOfData, false);

    SBMConnection.read();
    tLengthOfData--; // since the length must be skipped
    SBMConnection.readBytes(aDataBufferPtr, tLengthOfData);

    sei();
    return tLengthOfData;
}

/*
 * checks if description string is in progmen
 */
void printDescriptionPGM(const char * aDescription) {
    Serial.print((const __FlashStringHelper *) aDescription);
}

void printValue(struct SBMFunctionDescriptionStruct* aSBMFunctionDescription, uint16_t tActualValue) {
    {
        if (aSBMFunctionDescription->ValueFormatter == NULL) {
            Serial.print(aSBMFunctionDescription->Description);
            Serial.println(tActualValue);
            aSBMFunctionDescription->lastValue = tActualValue;
        } else {
            aSBMFunctionDescription->ValueFormatter(aSBMFunctionDescription, tActualValue);
        }
        Serial.flush();
        aSBMFunctionDescription->lastValue = tActualValue;
    }
}

/*
 * Read word and print if value has changed.
 * To avoid spurious outputs check changed values 3 times.
 */
void readWordAndPrint(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription, bool aOnlyPrintIfValueChanged) {
    uint16_t tActualValue = readWord(aSBMFunctionDescription->FunctionCode);

    if (aOnlyPrintIfValueChanged) {
        if (tActualValue != aSBMFunctionDescription->lastValue) {
            // check value again, maybe it was a transmit error
            delay(33); // just guessed the value
            uint16_t tActualValue2 = readWord(aSBMFunctionDescription->FunctionCode);
            if (tActualValue2 != aSBMFunctionDescription->lastValue) {
                delay(17); // just guessed the value
                uint16_t tActualValue3 = readWord(aSBMFunctionDescription->FunctionCode);
                if (tActualValue3 != aSBMFunctionDescription->lastValue) {
                    printValue(aSBMFunctionDescription, tActualValue);
                }
            }
        }

    } else {
        printValue(aSBMFunctionDescription, tActualValue);
    }
}

void printBinary(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aValue) {
    Serial.print(aDescription->Description);
    Serial.print("0b");
    Serial.println(aValue, BIN);
}

void printSigned(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aValue) {
    Serial.print(aDescription->Description);
    Serial.println((int) aValue);
}

const char * getCapacityModeUnit() {
    if (sCapacityModePower) {
        return StringCapacityModePower;
    }
    return StringCapacityModeCurrent;

}

void printCapacity(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aCapacity) {
    Serial.print((const __FlashStringHelper *) aDescription->Description);
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
    Serial.println();

    if (aDescription->DescriptionLCD != NULL) {
        // always print as mAh
        myLCD.setCursor(0, 3);
        myLCD.print(aDescription->DescriptionLCD);
        myLCD.print(aCapacity);
        myLCD.print(StringCapacityModeCurrent);
        myLCD.print('h');
    }
}

void printPercentage(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aPercentage) {
    Serial.print(aDescription->Description);
    Serial.print(aPercentage);
    Serial.println(" %");
    if (aDescription->DescriptionLCD != NULL) {
        myLCD.setCursor(0, 2);
        myLCD.print(aPercentage);
        myLCD.print(" %");
        myLCD.print(aDescription->DescriptionLCD);
    }
}

void printTime(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aMinutes) {
    Serial.print(aDescription->Description);
    if (aMinutes == 65535) {
        Serial.println(F("Battery not beeing (dis)charged"));
    } else {
        Serial.print(aMinutes);
        Serial.println(" min");
        if (aDescription->DescriptionLCD != NULL) {
            myLCD.setCursor(0, 1);
            myLCD.print(aMinutes);
            myLCD.print(aDescription->DescriptionLCD);
        }
    }
}

/*
 * Print only if changed by two ore more mV
 */
void printVoltage(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aVoltage) {
    if (aVoltage < aDescription->lastValue - 1 || aDescription->lastValue + 1 < aVoltage) {
        Serial.print((const __FlashStringHelper *) aDescription->Description);
        Serial.print((float) aVoltage / 1000, 3);
        Serial.println(" Volt");
        if (aDescription->DescriptionLCD != NULL) {
            myLCD.setCursor(0, 0);
            myLCD.print((float) aVoltage / 1000, 3);
            myLCD.print(" Volt");
        }
    }
}

/*
 * Print only if changed by two ore more mA
 */
void printCurrent(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aCurrent) {
    if (aCurrent < aDescription->lastValue - 1 || aDescription->lastValue + 1 < aCurrent) {
        Serial.print((const __FlashStringHelper *) aDescription->Description);
        Serial.print((int) aCurrent);
        Serial.println(" mA");
        if (aDescription->DescriptionLCD != NULL) {
            // clear old value
            myLCD.setCursor(12, 0);
            myLCD.print("        ");
            myLCD.setCursor(12, 0);
            myLCD.print((int) aCurrent);
            myLCD.print(" mA");
        }
    }
}

/*
 * Print only if changed by more than 0.1 C
 */
void printTemperature(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aTemperature) {
    if (aTemperature < aDescription->lastValue - 100 || aDescription->lastValue + 100 < aTemperature) {
        Serial.print(aDescription->Description);
        Serial.print((float) (aTemperature / 10.0) - 273.15);
        Serial.println(" C");
    }
}

/*
 * Format as ISO date
 */
void printManufacturerDate(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aDate) {
    Serial.print(aDescription->Description);

    int tDay = aDate & 0x1F;
    int tMonth = (aDate >> 5) & 0x0F;
    int tYear = 1980 + ((aDate >> 9) & 0x7F);
    String tDateAsString = " ";
    tDateAsString += tYear;
    tDateAsString += "-";
    tDateAsString += tMonth;
    tDateAsString += "-";
    tDateAsString += tDay;
    Serial.println(tDateAsString);
}

void printBatteryMode(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aMode) {
    Serial.print(aDescription->Description);

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
        sCapacityModePower = true;
        Serial.println(F("- Using power (10mWh) instead of current (mAh)"));
    }
}
void printBatteryStatus(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aStatus) {
    Serial.print(aDescription->Description);
    Serial.println(aStatus, BIN);
    /*
     * Error Bits
     */
    if (aStatus & OVER_CHARGED_ALARM) {
        Serial.println(F("- OVER_CHARGED_ALARM"));
    }
    if (aStatus & TERMINATE_CHARGE_ALARM) {
        Serial.println(F("- TERMINATE_CHARGE_ALARM"));
    }
    if (aStatus & OVER_TEMP_ALARM) {
        Serial.println(F("- OVER_TEMP_ALARM"));
    }
    if (aStatus & TERMINATE_DISCHARGE_ALARM) {
        Serial.println(F("- TERMINATE_DISCHARGE_ALARM"));
    }
    if (aStatus & REMAINING_CAPACITY_ALARM) {
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

}

void printFunctionDescriptionArray(struct SBMFunctionDescriptionStruct * aSBMFunctionDescription, uint8_t aLengthOfArray,
bool aOnlyPrintIfValueChanged) {
    for (uint8_t i = 0; i < aLengthOfArray; ++i) {
        readWordAndPrint(aSBMFunctionDescription, aOnlyPrintIfValueChanged);
        aSBMFunctionDescription++;
    }
}

void printSMBStaticInfo(void) {
    uint8_t tReceivedLength = 0;

    Serial.print(F("Chemistry: "));
    tReceivedLength = readBlock(CELL_CHEM, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    Serial.write(sI2CDataBuffer, tReceivedLength);
    Serial.println("");

    Serial.print(F("Manufacturer Name: "));
    tReceivedLength = readBlock(MFG_NAME, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    Serial.write(sI2CDataBuffer, tReceivedLength);
    Serial.println("");

    Serial.print(F("Manufacturer Data: "));
    tReceivedLength = readBlock(MANUFACTURER_DATA, sI2CDataBuffer, DATA_BUFFER_LENGTH);
    Serial.write(sI2CDataBuffer, tReceivedLength);
    Serial.write(" - 0x");
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
    sDesignVoltage = sSBMStaticFunctionDescriptionArray[INDEX_OF_DESIGN_VOLTAGE].lastValue;

}

void printSMBManufacturerInfo(void) {

    uint16_t tType = readWordFromManufacturerAccess(TI_Device_Type);
    Serial.print(F("Device Type: "));
    Serial.print(tType);
    Serial.print(F(" / 0x"));
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

void printSMBNonStandardInfo(bool aOnlyPrintIfValueChanged) {
    if (nonStandardInfoSupportedByPack > 1) {
        return;
    }
    if (nonStandardInfoSupportedByPack == 0) {
        // very simple check if non standard info supported by pack
        uint16_t tActualValue = readWord(sSBMNonStandardFunctionDescriptionArray[0].FunctionCode);
        uint16_t tActualValue1 = readWord(sSBMNonStandardFunctionDescriptionArray[1].FunctionCode);
        if (tActualValue == tActualValue1) {
            nonStandardInfoSupportedByPack = 2;
            return;
        }
    }

    printFunctionDescriptionArray(sSBMNonStandardFunctionDescriptionArray,
            (sizeof(sSBMNonStandardFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)), aOnlyPrintIfValueChanged);
}

void printSMBATRateInfo(void) {
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
