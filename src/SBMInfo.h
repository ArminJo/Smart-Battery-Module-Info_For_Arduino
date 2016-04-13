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
#define MANUFACTURER_ACCESS      0x00 // Manufacturer specific values
#define REMAINING_CAPACITY_ALARM  0x01
#define REMAINING_TIME_ALARM   	0x02

#define BATTERY_MODE            0x03
#define AtRate            		0x04
#define AtRateTimeToFull 		0x05
#define AtRateTimeToEmpty 		0x06
#define AtRateOK          		0x07

#define TEMPERATURE             0x08
#define VOLTAGE                 0x09
#define CURRENT                 0x0A
#define AverageCurrent        	0x0B // of last minute
#define MAX_ERROR          		0x0C // of state of charge calculation -Byte

#define RELATIVE_SOC            0x0D // StateOfCharge -Byte
#define ABSOLUTE_SOC            0x0E // -Byte
#define REMAINING_CAPACITY      0x0F
#define FULL_CHARGE_CAPACITY    0x10
#define RUN_TIME_TO_EMPTY     	0x11
#define AVERAGE_TIME_TO_EMPTY  	0x12
#define TIME_TO_FULL            0x13
#define CHARGING_CURRENT        0x14
#define CHARGING_VOLTAGE        0x15
#define BATTERY_STATUS          0x16
#define CYCLE_COUNT             0x17
#define DESIGN_CAPACITY         0x18
#define DESIGN_VOLTAGE          0x19
#define SPEC_INFO               0x1A
#define MFG_DATE                0x1B
#define SERIAL_NUM              0x1C
#define MFG_NAME                0x20   // String
#define DEV_NAME                0x21   // String
#define CELL_CHEM               0x22   // String
#define MFG_DATA                0x23   // String

#define CELL4_VOLTAGE           0x3C   // Individual cell voltages don't work on Lenovo and Dell Packs
#define CELL3_VOLTAGE           0x3D
#define CELL2_VOLTAGE           0x3E
#define CELL1_VOLTAGE           0x3F

#define STATE_OF_HEALTH 		0x4F // in % -Byte

/*
 * Bits of BatteryMode
 */
#define INTERNAL_CHARGE_CONTROLLER	0x0001
#define CONDITION_FLAG				0x0008
#define CHARGE_CONTROLLER			0x0100
#define ALARM_MODE					0x2000
#define CHARGER_MODE				0x4000
#define CAPACITY_MODE				0x8000

/*
 * Bits of BatteryStatus
 */
// Alarm Bits
#define OVER_CHARGED_ALARM 			0x8000
#define TERMINATE_CHARGE_ALARM 		0x4000
#define OVER_TEMP_ALARM 			0x1000
#define TERMINATE_DISCHARGE_ALARM 	0x0800
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
	uint16_t lastValue;
	void (*ValueFormatter)(struct SBMFunctionDescriptionStruct * aDescription, uint16_t aValueToFormat);
};

/*
 * TI few ManufacturerAccess Commands
 */
#define BQ20Z70_Device_Type				0x0001
#define BQ20Z70_Firmware_Version		0x0002

#define BQ20Z70_Hardware_Version		0x0003
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
