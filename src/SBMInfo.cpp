/*
 *  SBMInfo.cpp
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

/*
 *  Corresponds to A4/A5 - the hardware I2C pins on Arduinos
 */
#define SDA_PORT PORTC
#define SDA_PIN 4

#define SCL_PORT PORTC
#define SCL_PIN 5
#define I2C_SLOWMODE 1
// Otherwise it may give read errors because of the arduino 1 ms clock interrupt.
#define I2C_NOINTERRUPT  1
#include <SoftI2CMaster.h>

#define DATA_BUFFER_LENGTH 32
uint8_t sI2CDataBuffer[DATA_BUFFER_LENGTH];

uint8_t sI2CDeviceAddress;

void printBatteryMode(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aMode);
void printBatteryStatus(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aStatus);
void printManufacturerDate(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aDate);
void printVoltage(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aVoltage);
void printCurrent(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aCurrent);
void printTemperature(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aTemperature);

void printSMBStaticInfo(void);
void printSMBDynamicInfo(void);
void printSMBNonStandardInfo(void);
void printSMBTestInfo(void);

bool checkForAttachedI2CDevice(uint8_t aI2CDeviceAddress);
int scanForAttachedI2CDevice(void);

void BlinkLedForever(int aBinkDelay);
void TogglePin(uint8_t aPinNr);
int readWord(uint8_t aFunction);

// Pin 13 has an LED connected on most Arduino boards.
const int LED_PIN = 13;

/*
 * Command definitions
 */
struct SBMFunctionDescriptionStruct sSBMStaticFunctionDescriptionArray[] = { {
SERIAL_NUM, "Serial Number: " }, {
MFG_DATE, "Manufacture Date (YYYY-MM-DD):", 0xFFFF, &printManufacturerDate }, {
DESIGN_CAPACITY, "Design Capacity (mAh): " }, {
DESIGN_VOLTAGE, "Design Voltage: ", 0xFFFF, &printVoltage }, {
CHARGING_CURRENT, "Charging Current (mA): " }, {
CHARGING_VOLTAGE, "Charging Voltage: ", 0, &printVoltage }, {
SPEC_INFO, "Specification Info: " }, {
CYCLE_COUNT, "Cycle Count: " }, {
MAX_ERROR, "Max Error of charge calculation (%): " }, {
REMAINING_TIME_ALARM, "RemainingTimeAlarm (min): " }, {
REMAINING_CAPACITY_ALARM, "Remaining Capacity Alarm: " }, {
BATTERY_MODE, "Battery Mode (BIN): 0b", 0xFFFF, &printBatteryMode } };

struct SBMFunctionDescriptionStruct sSBMDynamicFunctionDescriptionArray[] = { {
FULL_CHARGE_CAPACITY, "Full Charge Capacity (mAh): " }, {
REMAINING_CAPACITY, "Remaining Capacity (mAh): " }, {
RELATIVE_SOC, "Relative Charge(%): " }, {
ABSOLUTE_SOC, "Absolute Charge(%): " }, {
RUN_TIME_TO_EMPTY, "Minutes remaining until empty: ", 0xFFFF }, {
AVERAGE_TIME_TO_EMPTY, "Average minutes remaining until empty: ", 0xFFFF }, {
TIME_TO_FULL, "Minutes remaining for full charge: ", 0xFFFF }, {
BATTERY_STATUS, "Battery Status (BIN): 0b", 0xFFFF, &printBatteryStatus }, {
VOLTAGE, "Voltage: ", 0xFFFF, &printVoltage }, {
CURRENT, "Current (mA): ", 0xFFFF, &printCurrent }, {
TEMPERATURE, "Temperature (C): ", 0xFFFF, &printTemperature } };

/*
 * These aren't part of the standard, but work with some packs.
 */
struct SBMFunctionDescriptionStruct sSBMNonStandardFunctionDescriptionArray[] = { {
STATE_OF_HEALTH, "State of Health: " }, {
CELL1_VOLTAGE, "Cell 1 Voltage: ", 0, &printVoltage }, {
CELL2_VOLTAGE, "Cell 2 Voltage: ", 0, &printVoltage }, {
CELL3_VOLTAGE, "Cell 3 Voltage: ", 0, &printVoltage }, {
CELL4_VOLTAGE, "Cell 4 Voltage: ", 0, &printVoltage } };

struct SBMFunctionDescriptionStruct sSBMTestFunctionDescriptionArray[] = { {
AverageCurrent, "Average Current (mA): " }, {
AtRate, "AtRate: " }, {
AtRateTimeToFull, "AtRateTimeToFull: " }, {
AtRateTimeToEmpty, "AtRateTimeToEmpty: " }, {
AtRateOK, "AtRateOK: " }, };

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

	bool tI2CSucessfullyInitialized = i2c_init();
	if (tI2CSucessfullyInitialized) {
		Serial.println(F("I2C initalized sucessfully"));
	} else {
		Serial.println(F("I2C pullups missing"));
		BlinkLedForever(100);
	}
	Serial.flush();

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
	Serial.flush();

	printSMBStaticInfo();
	Serial.println(F("\r\n*** TEST INFO ***"));
	Serial.flush();

	printSMBTestInfo();
	Serial.println(F("\r\n*** DYNAMIC INFO ***"));
	Serial.flush();

	printSMBDynamicInfo();
	printSMBNonStandardInfo();
	Serial.println(F("\r\n*** CHANGED VALUES ***"));
	Serial.flush();
}

void loop() {
	printSMBDynamicInfo();
	printSMBNonStandardInfo();
	delay(3000);
}

void TogglePin(uint8_t aPinNr) {
	if (digitalRead(aPinNr) == HIGH) {
		digitalWrite(aPinNr, LOW);
	} else {
		digitalWrite(aPinNr, HIGH);
	}
}

void BlinkLedForever(int aBinkDelay) {
	do {
		digitalWrite(LED_PIN, HIGH);
		delay(aBinkDelay);
		digitalWrite(LED_PIN, LOW);
		delay(aBinkDelay);
	} while (true);
}

bool checkForAttachedI2CDevice(uint8_t aStandardDeviceAddress) {
	bool tOK = i2c_start(aStandardDeviceAddress << 1 | I2C_WRITE);
	i2c_stop();
	if (tOK) {
		Serial.print(F("Found attached I2C device at 0x"));
		Serial.println(aStandardDeviceAddress, HEX);
		sI2CDeviceAddress = SBM_DEVICE_ADDRESS;
		return true;
	} else {
		return false;
	}
}

int sScanCount = 0;
int scanForAttachedI2CDevice(void) {
	int tFoundAdress = -1;
	for (uint8_t i = 0; i < 127; i++) {
		bool ack = i2c_start(i << 1 | I2C_WRITE);
		if (ack) {
			Serial.print(F("Found I2C device attached at address: 0x"));
			Serial.println(i, HEX);
			tFoundAdress = i;
		}
		i2c_stop();
	}
	if (tFoundAdress < 0) {
		Serial.print(F("Found no attached I2C device - "));
		Serial.println(sScanCount++);
	} else {
		sI2CDeviceAddress = tFoundAdress;
	}
	return tFoundAdress;
}

int readWord(uint8_t aFunction) {
	i2c_start((sI2CDeviceAddress << 1) | I2C_WRITE);
	i2c_write(aFunction);
	i2c_rep_start((sI2CDeviceAddress << 1) | I2C_READ);
	uint8_t tLSB = i2c_read(false);
	uint8_t tMSB = i2c_read(true);
	i2c_stop();
	return (int) tLSB | (((int) tMSB) << 8);
}

int readWordFromManufacturerAccess(uint16_t aCommand) {
	i2c_start((sI2CDeviceAddress << 1) | I2C_WRITE);
	i2c_write(MANUFACTURER_ACCESS);
	// Write manufacturer command word
	i2c_rep_start((sI2CDeviceAddress << 1) | I2C_WRITE);
	i2c_write(aCommand);
	i2c_write(aCommand >> 8);
	i2c_stop();
	// Read manufacturer result word
	i2c_start((sI2CDeviceAddress << 1) | I2C_WRITE);
	i2c_write(MANUFACTURER_ACCESS);
	i2c_rep_start((sI2CDeviceAddress << 1) | I2C_READ);
	uint8_t tLSB = i2c_read(false);
	uint8_t tMSB = i2c_read(true);
	i2c_stop();
	return (int) tLSB | (((int) tMSB) << 8);
}

uint8_t readBlock(uint8_t aCommand, uint8_t* aDataBufferPtr, uint8_t aDataBufferLength) {
	i2c_start((sI2CDeviceAddress << 1) + I2C_WRITE);
	i2c_write(aCommand);
	i2c_rep_start((sI2CDeviceAddress << 1) + I2C_READ);

	// First read length of data
	uint8_t tLengthOfData = i2c_read(false);
	if (tLengthOfData > aDataBufferLength) {
		tLengthOfData = aDataBufferLength;
	}

	// then read data
	uint8_t tIndex;
	for (tIndex = 0; tIndex < tLengthOfData - 1; tIndex++) {
		aDataBufferPtr[tIndex] = i2c_read(false);
	}
	// Read last byte with "true"
	aDataBufferPtr[tIndex++] = i2c_read(true);

	i2c_stop();
	return tLengthOfData;
}

/*
 * Read word and print if value has changed.
 * To avoid spurious outputs check changed values 2 times.
 */
void readWordAndPrint(struct SBMFunctionDescriptionStruct *aSBMFunctionDescription) {
	uint16_t tActualValue = readWord(aSBMFunctionDescription->FunctionCode);
	if (tActualValue != aSBMFunctionDescription->lastValue) {
		// check value again, maybe it was a transmit error
		delay(33); // just guessed the value
		uint16_t tActualValue2 = readWord(aSBMFunctionDescription->FunctionCode);
		if (tActualValue2 != aSBMFunctionDescription->lastValue) {
			delay(17); // just guessed the value
			uint16_t tActualValue3 = readWord(aSBMFunctionDescription->FunctionCode);
			if (tActualValue3 != aSBMFunctionDescription->lastValue) {
				if (aSBMFunctionDescription->ValueFormatter == NULL) {
					Serial.print(aSBMFunctionDescription->Description);
					Serial.println(tActualValue);
					aSBMFunctionDescription->lastValue = tActualValue;
				} else {
					aSBMFunctionDescription->ValueFormatter(aSBMFunctionDescription, tActualValue);
				}
				Serial.flush();
			}
		}
	}
}

void printBatteryMode(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aMode) {
	Serial.print(aDescription->Description);
	aDescription->lastValue = aMode;

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
		Serial.println(F("- Report in 10mW or 10mWh"));
	}
}

/*
 * Format as ISO date
 */
void printManufacturerDate(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aDate) {
	Serial.print(aDescription->Description);
	aDescription->lastValue = aDate;

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

void printBatteryStatus(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aStatus) {
	Serial.print(aDescription->Description);
	aDescription->lastValue = aStatus;
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

/*
 * Print only if changed by two ore more
 */
void printVoltage(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aVoltage) {
	if (aVoltage < aDescription->lastValue - 1 || aDescription->lastValue + 1 < aVoltage) {
		Serial.print(aDescription->Description);
		Serial.println((float) aVoltage / 1000, 3);
		aDescription->lastValue = aVoltage;
	}
}

/*
 * Print only if changed by two ore more
 */
void printCurrent(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aCurrent) {
	if (aCurrent < aDescription->lastValue - 1 || aDescription->lastValue + 1 < aCurrent) {
		Serial.print(aDescription->Description);
		aDescription->lastValue = aCurrent;
		Serial.println((int) aCurrent);
	}
}

/*
 * Print only if changed by more than 0.1 C
 */
void printTemperature(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aTemperature) {
	if (aTemperature < aDescription->lastValue - 100 || aDescription->lastValue + 100 < aTemperature) {
		Serial.print(aDescription->Description);
		aDescription->lastValue = aTemperature;
		Serial.println((float) (aTemperature / 10.0) - 273.15);
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
	tReceivedLength = readBlock(MFG_DATA, sI2CDataBuffer, DATA_BUFFER_LENGTH);
	Serial.write(sI2CDataBuffer, tReceivedLength);
	Serial.println("");

	Serial.print(F("Device Name: "));
	tReceivedLength = readBlock(DEV_NAME, sI2CDataBuffer, DATA_BUFFER_LENGTH);
	Serial.write(sI2CDataBuffer, tReceivedLength);
	Serial.println("");

	for (uint8_t i = 0; i < (sizeof(sSBMStaticFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)); ++i) {
		readWordAndPrint(&sSBMStaticFunctionDescriptionArray[i]);
	}

	Serial.println(F("\r\n*** MANUFACTURER INFO ***"));
	Serial.print(F("ManufacturerAccess: "));
	uint16_t tManufacurerAccess = readWord(MANUFACTURER_ACCESS);
	Serial.print(tManufacurerAccess);
	Serial.print(F(" / 0x"));
	Serial.println(tManufacurerAccess, HEX);

	uint16_t tType = readWordFromManufacturerAccess(BQ20Z70_Device_Type);
	uint16_t tVersion = readWordFromManufacturerAccess(BQ20Z70_Firmware_Version);
	if (tType != tVersion) {
		Serial.print(F("Device Type: "));
		Serial.print(tType);
		Serial.print(F(" / 0x"));
		Serial.println(tType, HEX);

		if (tType == 2084) {
			Serial.print(F("Controller IC identified by device type: "));
			Serial.print(F("bq2084"));
			Serial.print(F("End of Discharge Voltage Level: "));
			uint16_t tLevel = readWordFromManufacturerAccess(BQ2084_EDV_level);
			Serial.println(((float) tLevel) / 1000, 3);
		} else {
			if (tType == 0x700) {
				Serial.print(F("Controller IC identified by device type: "));
				Serial.println(F("bq20z70"));
			}
			Serial.print(F("Hardware Version: 0x"));
			Serial.println(readWordFromManufacturerAccess(BQ20Z70_Hardware_Version), HEX);
		}

		Serial.print(F("Firmware Version: "));
		Serial.print((uint8_t) (tVersion >> 8), HEX);
		Serial.print(".");
		Serial.println((uint8_t) tVersion, HEX);

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
			Serial.println(F("Normal Discharge"));
		} else if (tStatus == 0x05) {
			Serial.println(F("Charge"));
		} else if (tStatus == 0x07) {
			Serial.println(F("Charge Termination"));
		} else if (tStatus == 0x0C) {
			Serial.println(F("Battery Failure"));
		} else if (tStatus == 0x09) {
			Serial.println(F("Permanent Failure"));
		} else if (tStatus == 0x0F) {
			Serial.println(F("Battery Pack removed"));
		}
	} else if (tManufacurerAccess != tType) {
		Serial.print(F("Info: "));
		Serial.println(tType, HEX);
	}

}

void printSMBDynamicInfo(void) {
	for (uint8_t i = 0; i < (sizeof(sSBMDynamicFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)); ++i) {
		readWordAndPrint(&sSBMDynamicFunctionDescriptionArray[i]);
	}
}

int nonStandardInfoSupportedByPack = 0; // 0 not initialized, 1 supported, >1 not supported
void printSMBNonStandardInfo(void) {
	if (nonStandardInfoSupportedByPack > 1) {
		return;
	}
	if (nonStandardInfoSupportedByPack == 0) {
		// check
		uint16_t tActualValue = readWord(sSBMNonStandardFunctionDescriptionArray[0].FunctionCode);
		uint16_t tActualValue1 = readWord(sSBMNonStandardFunctionDescriptionArray[1].FunctionCode);
		if (tActualValue == tActualValue1) {
			nonStandardInfoSupportedByPack = 2;
			return;
		}
	}

	for (uint8_t i = 0; i < (sizeof(sSBMNonStandardFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)); ++i) {
		readWordAndPrint(&sSBMNonStandardFunctionDescriptionArray[i]);
	}
}

void printSMBTestInfo(void) {
	for (uint8_t i = 0; i < (sizeof(sSBMTestFunctionDescriptionArray) / sizeof(SBMFunctionDescriptionStruct)); ++i) {
		readWordAndPrint(&sSBMTestFunctionDescriptionArray[i]);
	}
}
