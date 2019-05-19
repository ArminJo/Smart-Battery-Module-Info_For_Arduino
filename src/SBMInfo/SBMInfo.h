/*
 * SBMInfo.h
 *
 *  Copyright (C) 2016  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 */

#ifndef SRC_SBMINFO_H_
#define SRC_SBMINFO_H_

// standard I2C address for Smart Battery packs
#define SBM_DEVICE_ADDRESS 0x0B

// Standard and common non-standard Smart Battery commands
#define MANUFACTURER_ACCESS      0x00 // r/w Manufacturer specific values
#define REMAINING_CAPACITY_ALARM  0x01 // r/w
#define REMAINING_TIME_ALARM   	0x02 // r/w

#define BATTERY_MODE            0x03 // r/w

#define AtRate            		0x04 // r/w
#define AtRateTimeToFull 		0x05 // r
#define AtRateTimeToEmpty 		0x06 // r
#define AtRateOK          		0x07 // r

#define TEMPERATURE             0x08
#define VOLTAGE                 0x09
#define CURRENT                 0x0A
#define AverageCurrent        	0x0B // of last minute
#define MAX_ERROR          		0x0C // Byte - of state of charge calculation

#define RELATIVE_SOC            0x0D // Byte - StateOfCharge
#define ABSOLUTE_SOC            0x0E // Byte
#define REMAINING_CAPACITY      0x0F
#define FULL_CHARGE_CAPACITY    0x10
#define RUN_TIME_TO_EMPTY     	0x11
#define AVERAGE_TIME_TO_EMPTY  	0x12
#define TIME_TO_FULL            0x13
#define CHARGING_CURRENT        0x14  // r/w ?
#define CHARGING_VOLTAGE        0x15  // r/w ?
#define BATTERY_STATUS          0x16  // r/w ?
#define CYCLE_COUNT             0x17
#define DESIGN_CAPACITY         0x18
#define DESIGN_VOLTAGE          0x19
#define SPEC_INFO               0x1A
#define MFG_DATE                0x1B
#define SERIAL_NUM              0x1C
#define RESERVED_1              0x1D - 0x1F
#define MFG_NAME                0x20   // String
#define DEV_NAME                0x21   // String
#define CELL_CHEM               0x22   // String
#define MANUFACTURER_DATA       0x23   // Data
#define RESERVED_2              0x25 - 0x2E

#define PACK_STATUS             0x2F   // r/w Word - OptionalMfgFunction5

#define RESERVED_3              0x30 - 0x3B

#define CELL4_VOLTAGE           0x3C   // r/w Word - OptionalMfgFunction4 - Individual cell voltages don't work on Lenovo and Dell Packs
#define CELL3_VOLTAGE           0x3D   // r/w Word - OptionalMfgFunction3
#define CELL2_VOLTAGE           0x3E   // r/w Word - OptionalMfgFunction2
#define CELL1_VOLTAGE           0x3F   // r/w Word - OptionalMfgFunction1

#define STATE_OF_HEALTH 		0x4F   // in % Byte - = CELL1_VOLTAGE for bq2085

/*
 * Bits of BatteryMode
 */
#define INTERNAL_CHARGE_CONTROLLER	0x0001
#define CONDITION_FLAG				0x0008
#define CHARGE_CONTROLLER			0x0100
#define ALARM_MODE					0x2000
#define CHARGER_MODE				0x4000
#define CAPACITY_MODE				0x8000
const char StringCapacityModeCurrent[] = " mA";
const char StringCapacityModePower[] = "0 mW"; // 10mWh

/*
 * Bits of BatteryStatus
 */
// Alarm Bits
#define OVER_CHARGED_ALARM__FLAG 		0x8000
#define TERMINATE_CHARGE_ALARM_FLAG 	0x4000
#define OVER_TEMP_ALARM_FLAG 			0x1000
#define TERMINATE_DISCHARGE_ALARM_FLAG 	0x0800
#define REMAINING_CAPACITY_ALARM_FLAG 	0x0200
#define REMAINING_TIME_ALARM_FLAG 		0x0100

// Bits of Status
#define INITIALIZED 		0x0080
#define DISCHARGING 		0x0040
#define FULLY_CHARGED 		0x0020
#define FULLY_DISCHARGED 	0x0010

struct SBMFunctionDescriptionStruct {
    uint8_t FunctionCode;
    const char * Description;
    void (*ValueFormatter)(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aValueToFormat);
    const char * DescriptionLCD; // if set output value also on LCD
    uint16_t lastValue;
};

/*
 * TI few ManufacturerAccess Commands
 */
#define TI_Device_Type             0x0001
#define TI_Firmware_Version        0x0002

#define BQ20Z70_Hardware_Version        0x0003
#define BQ40Z50_Hardware_Version        0x0003

#define BQ2084_EDV_level				0x0003

#define BQ20Z70_Manufacturer_Status		0x0006

/*
 * BQ20Z70 extended commands - Not used yet
 */
#define BQ20Z70_AFEData    				0x45 // -11+1 Byte
#define BQ20Z70_FETControl    			0x46 // -Byte
#define BQ20Z70_SafetyStatus    		0x51
#define BQ20Z70_PFStatus    			0x53 // permanent failure
#define BQ20Z70_OperationStatus    		0x54
#define BQ20Z70_ChargingStatus    		0x55
#define BQ20Z70_WDResetData    			0x58
#define BQ20Z70_PackVoltage    			0x5a
#define BQ20Z70_AverageVoltage    		0x5d // of last minute
#define BQ20Z70_UnSealKey    			0x60 // -4 Byte
#define BQ20Z70_FullAccessKey    		0x61 // -4 Byte
#define BQ20Z70_PFKey    				0x62 // -4 Byte
#define BQ20Z70_AuthenKey3    			0x63 // -4 Byte
#define BQ20Z70_AuthenKey2    			0x64 // -4 Byte
#define BQ20Z70_AuthenKey1   	 		0x65 // -4 Byte
#define BQ20Z70_AuthenKey0    			0x66 // -4 Byte
#define BQ20Z70_ManufacturerInfo    	0x70 // -8+1 Byte
#define BQ20Z70_SenseResistor    		0x71
#define BQ20Z70_DataFlashSubClassID   	0x77
#define BQ20Z70_DataFlashSubClassPage1	0x78 // -32 Byte
// ...
#define BQ20Z70_DataFlashSubClassPage8	0x7f // -32 Byte

#endif /* SRC_SBMINFO_H_ */
