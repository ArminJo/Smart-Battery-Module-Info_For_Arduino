/*
 *  MeasureVoltageAndResistance.hpp
 *
 *  Measures voltage and resistance with 1 mV and 2 ohm resolution at the lower end.
 *  First voltage is measured. If voltage is zero, then resistance to ground is measured using 5 volt (VCC) and 10 kOhm or 100 kOhm supply.
 *
 *  Copyright (C) 2021-2025  Armin Joachimsmeyer
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

/*
 *  SCHEMATIC
 *            +----o A3  open/VCC (for R measurement) | open (for U measurement)
 *            |
 *            -
 *           | |
 *           | | R3 (10 kOhm)    _____
 *           | |            +---|_____|---o A1  open(R) | open/GND(U) - For voltage divider
 *            -             |   R1 (22k)
 *            |     _____   |
 * Input <----+----|_____|--+--o  A0 input for R and U measurement
 *   ^        |    R0 100 k - just to protect the pin
 *   |        -
 *   |       | |
 *   |       | | R2 100 kOhm
 *   |       | |
 *   -        -
 *  | |       |
 *  | | Rx    +---o A2  VCC(R) | open(U)
 *  | |
 *   -
 *   |
 *   o GND
 *
 *  The ratio of R2 to Rx is equal the Ratio of (1023 - x) to x
 *
 *  => The formula is: Rx/R2 = x / (1023-x)
 *      Rx = R2 * x / (1023-x)
 *
 */

#ifndef _MEASURE_VOLTAGE_AND_RESISTANCE_HPP
#define _MEASURE_VOLTAGE_AND_RESISTANCE_HPP

#include <Arduino.h>

#include "ADCUtils.hpp"

/*
 * To customize this code to different requirements, there are some compile options / macros available.
 * These macros must be defined in your program before the line #include "MeasureVoltageAndResistance.hpp" to take effect.
 *
 * 1. Voltmeter+Ohmmeter connections and resistors default values
 */
//#define MEASUREMENT_PIN         A0
//#define MEASUREMENT_CHANNEL      0
//#define VOLTAGE_GROUND_PIN      A1
//#define RESISTOR_2_PIN          A2
//#define RESISTOR_3_PIN          A3
//
//#define RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM              100 // R0
//#define RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM    22 // R1
//#define RESISTOR_2_TO_INPUT_KOHM                            RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM // R2
//#define RESISTOR_3_TO_INPUT_KOHM                            10 // R3
//
/*
 * 2. Values to determine functionality
 */
//#define PRINT_OF_RESISTOR_MEASURMENT_VOLTAGE // enables print of voltage at resistor under measurement (0 to VCC).
//#define PRINT_OF_VCC
//#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100UL // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
//#define USE_LCD       // To enable LCD output at the externally provided myLCD object
//#define USE_2004_LCD // for rendering Resistance and resistance voltage in one line
//#define LCD_OBJECT_NAME         myLCD
#if !defined(LCD_OBJECT_NAME)
#define LCD_OBJECT_NAME         myLCD
#endif
//
#if !defined(MEASUREMENT_PIN)
#define MEASUREMENT_PIN         A0
#endif
#if !defined(MEASUREMENT_CHANNEL)
#define MEASUREMENT_CHANNEL      0
#endif
#if !defined(VOLTAGE_GROUND_PIN)
#define VOLTAGE_GROUND_PIN      A1
#endif
#if !defined(RESISTOR_2_PIN)
#define RESISTOR_2_PIN          A2
#endif
#if !defined(RESISTOR_3_PIN)
#define RESISTOR_3_PIN          A3
#endif

#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT)
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100UL // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif

// Fixed attenuator for voltage measurement
#if !defined(RESISTOR_TO_VOLTAGE_PIN_KOHM)
#define RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM              100 // R0
#endif
#if !defined(RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM)
#define RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM    22 // R1
#endif

// fixed resistors for resistor measurement
#if !defined(RESISTOR_2_TO_INPUT_KOHM)
#define RESISTOR_2_TO_INPUT_KOHM                            RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM // R2
#endif
#if !defined(RESISTOR_3_TO_INPUT_KOHM)
#define RESISTOR_3_TO_INPUT_KOHM                            10 // R3
#endif
#define REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT 1050L // Resistance measurement: if the input voltage is below this value, use the internal 1.1 volt reference

struct ResistanceMeasurementResultStruct {
    uint32_t ResistanceOhm;
    float VoltageAtResistor;
    bool isOverflow;
};

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// function prototypes just for documentation
void setDirectVoltageMode();
uint16_t measureVoltage(uint16_t tVCCVoltageMillivolt);
void setResistorMode();
bool measureResistance(uint16_t aVCCVoltageMillivolt, ResistanceMeasurementResultStruct *aResistanceMeasurementResult);
void MeasureVoltageAndResistance();
void printVoltageAndResistanceUsage();

// After all includes
#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file
#endif

void setDirectVoltageMode() {
    pinMode(RESISTOR_2_PIN, INPUT);
    pinMode(RESISTOR_3_PIN, INPUT);
    pinMode(VOLTAGE_GROUND_PIN, INPUT);
}

/*
 * Measure voltage first not attenuated and with VCC as reference
 * If value >= VCC use attenuator, if below 1.050 V use 1.1 V reference
 * @param - VCC to compute voltage
 */
uint16_t measureVoltage(uint16_t tVCCVoltageMillivolt) {
//    uint8_t tVoltageRange = 0;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

    setDirectVoltageMode();
    /*
     * getVoltageMillivolt() waits for ADC channel to switch from VCC measurement channel and INTERNAL reference
     * to voltage channel (A0) with VCC reference
     */
    uint16_t tInputVoltageMillivolt = getVoltageMillivolt(tVCCVoltageMillivolt, MEASUREMENT_CHANNEL);
    if (tInputVoltageMillivolt >= tVCCVoltageMillivolt) {
        /*
         * Voltage >= VCC -> enable Attenuator
         */
        pinMode(VOLTAGE_GROUND_PIN, OUTPUT);
        digitalWrite(VOLTAGE_GROUND_PIN, LOW);
        tInputVoltageMillivolt = getVoltageMillivolt(tVCCVoltageMillivolt, MEASUREMENT_CHANNEL);
        tInputVoltageMillivolt = (((uint32_t) tInputVoltageMillivolt)
                * (RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM + RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM))
                / (RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM);
    } else {
        /*
         * Voltage at ADC input is below VCC
         */
        if (tInputVoltageMillivolt < (ADC_INTERNAL_REFERENCE_MILLIVOLT - 50)) {
            /*
             * Voltage at ADC input is below 1.05 volt, so we can switch to
             * the internal 1.1 volt reference to get a better resolution (around 4 times better => ~ 1 mV)
             */
            tInputVoltageMillivolt = getVoltageMillivoltWith_1_1VoltReference(MEASUREMENT_CHANNEL);
//        tVoltageRange = 1;
        }
    }
#pragma GCC diagnostic pop
#if defined(LOCAL_DEBUG)
    Serial.print(F("Input="));
    Serial.print(tInputVoltageMillivolt);
    Serial.print(F(" mV, VCC="));
    Serial.print(tVCCVoltageMillivolt);
    Serial.println(F(" mV"));
#endif
    return tInputVoltageMillivolt;
}

/*
 * Enable 100 kOhm resistor to VCC
 */
void setResistorMode() {
    pinMode(VOLTAGE_GROUND_PIN, INPUT);
    pinMode(RESISTOR_2_PIN, OUTPUT);
    digitalWrite(RESISTOR_2_PIN, HIGH);
}

/*
 * First measure voltage when input is supplied by VCC and 100 kOhm
 * if high resistance / high voltage, check for overflow
 * if voltage below 1050 mV use the internal 1.1 volt reference to increase resolution
 * if
 */
bool measureResistance(uint16_t aVCCVoltageMillivolt, ResistanceMeasurementResultStruct *aResistanceMeasurementResult) {
    setResistorMode();

#if defined(LOCAL_DEBUG)
    uint8_t tResistanceRange = 0;
#endif
    delay(20); // wait for stray capacity to be charged by resistors
    uint16_t tInputRawReading = waitAndReadADCChannelWithReference(MEASUREMENT_CHANNEL, DEFAULT);
    uint16_t tInputVoltage = (uint32_t) tInputRawReading * aVCCVoltageMillivolt / READING_FOR_AREF;
    uint32_t tRxOhm;

    aResistanceMeasurementResult->isOverflow = false;
    if (tInputVoltage > REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT) {
        // High resistance value here
        if (tInputRawReading == MAX_ADC_VALUE) {
            // Overflow!
            tRxOhm = 9999999;
            aResistanceMeasurementResult->isOverflow = true;
        } else {
            tRxOhm = (RESISTOR_2_TO_INPUT_KOHM * 1000L * tInputRawReading) / (MAX_ADC_VALUE - tInputRawReading);
            // Here we have a resolution of 160 to 350 ohm at 1 MOhm
            // Clip at 10 MOhm
            if (tRxOhm > 9999999) {
                tRxOhm = 9999999;
                aResistanceMeasurementResult->isOverflow = true;
            }
        }
    } else {
        // The input voltage is below 1050 mV -> use the internal 1.1 volt reference
#if defined(LOCAL_DEBUG)
        tResistanceRange = 1;
#endif
        /*
         * Switch to 1.1 volt reference increasing the resolution by around 4
         * This happens at around 28 kOhm at 4.7 VCC / 34 kOhm at 4.1 VCC depending on the current value of VCC
         * Here we have a resolution of 24 to 37 ohm
         */
        tInputRawReading = waitAndReadADCChannelWithReference(MEASUREMENT_CHANNEL, INTERNAL);

        // The compensated VCC reading at 1.1 volt reference
        uint16_t tReadingAtVCCReference = (aVCCVoltageMillivolt * READING_FOR_AREF) / 1100;

        tRxOhm = (RESISTOR_2_TO_INPUT_KOHM * 1000L * tInputRawReading) / (tReadingAtVCCReference - tInputRawReading);

        /*
         * Formula is: (for 5 V and 1050 mV, in order to get a constant value)
         * Rx = Rvcc * 1.050 V / 3.95 V = 2416 ohm
         * Here we have a resolution of 2 to 6 ohm
         */
        const uint16_t tResistanceForThresholdVoltage =
                (RESISTOR_2_TO_INPUT_KOHM * RESISTOR_3_TO_INPUT_KOHM * REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT * 1000)
                        / ((RESISTOR_2_TO_INPUT_KOHM + RESISTOR_3_TO_INPUT_KOHM)
                                * (5000 - REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT));
        if (tRxOhm < tResistanceForThresholdVoltage) {
#if defined(LOCAL_DEBUG)
            tResistanceRange = 2;
            Serial.print(F("ResistanceForThresholdVoltage="));
            Serial.print(tResistanceForThresholdVoltage);
            Serial.println(F(" ohm"));
#endif
            /*
             * Switch on a parallel resistor to VCC increasing the resolution by another factor of 4.
             */
            pinMode(RESISTOR_3_PIN, OUTPUT);
            digitalWrite(RESISTOR_3_PIN, HIGH);
            delay(2);
            tInputRawReading = waitAndReadADCChannelWithReference(MEASUREMENT_CHANNEL, INTERNAL);
            tRxOhm =
                    (((RESISTOR_2_TO_INPUT_KOHM * RESISTOR_3_TO_INPUT_KOHM) / (RESISTOR_2_TO_INPUT_KOHM + RESISTOR_3_TO_INPUT_KOHM))
                            * 1000L * tInputRawReading) / (tReadingAtVCCReference - tInputRawReading);
            digitalWrite(RESISTOR_3_PIN, LOW);
            pinMode(RESISTOR_3_PIN, INPUT);
        }
        /*
         * Input voltage for 1.1 volt reference
         */
        tInputVoltage = tInputRawReading * 1100L / READING_FOR_AREF;
    }
#if defined(LOCAL_DEBUG)
    Serial.print(F("ResistanceRange="));
    Serial.print(tResistanceRange);
    Serial.print(F(" RAW reading="));
    Serial.print(tInputRawReading);
    Serial.print(F(" R="));
    Serial.print(tRxOhm);
    Serial.println(F(" Ohm"));
#endif

    aResistanceMeasurementResult->VoltageAtResistor = ((float) tInputVoltage) / 1000.0;
    aResistanceMeasurementResult->ResistanceOhm = tRxOhm;
    return aResistanceMeasurementResult->isOverflow;
}

/*
 * Voltage and Resistance have 3 decimals
 */
#if !defined(VOLTAGE_RESISTANCE_ROW)
#define VOLTAGE_RESISTANCE_ROW              0
#endif
#if !defined(VOLTAGE_RESISTANCE_START_COLUMN)
#define VOLTAGE_RESISTANCE_START_COLUMN     0 // For resistors up to 1 MOhm, so 10 volt has 2 leading spaces
#endif

// VCC is only printed if voltage is displayed
#if defined(PRINT_OF_VCC)
#  if !defined(VCC_ROW)
#define VCC_ROW                             1
#  endif
#  if !defined(VCC_COLUMN)
#define VCC_COLUMN                          VOLTAGE_RESISTANCE_START_COLUMN // is also rendered as 8 character float like voltage
#  endif
#endif

#if defined(PRINT_OF_RESISTOR_MEASURMENT_VOLTAGE)
#  if defined(USE_2004_LCD) && VOLTAGE_RESISTANCE_START_COLUMN == 0
#    if !defined(RESISTOR_MEASURMENT_VOLTAGE_ROW)
#define RESISTOR_MEASURMENT_VOLTAGE_ROW     VOLTAGE_RESISTANCE_ROW // fits in the same line if column is 0
#define RESISTOR_MEASURMENT_VOLTAGE_COLUMN  12
#    endif
#  endif

#  if !defined(RESISTOR_MEASURMENT_VOLTAGE_ROW)
#define RESISTOR_MEASURMENT_VOLTAGE_ROW     1
#  endif
#  if !defined(RESISTOR_MEASURMENT_VOLTAGE_COLUMN)
#define RESISTOR_MEASURMENT_VOLTAGE_COLUMN   0
#  endif
#endif

/*
 * Convenience function
 * First voltage is measured.
 * If voltage is zero, then resistance to ground is measured using 5 volt (VCC) and 10 kOhm or 100 kOhm supply.
 * For LCD output, the macros USE_LCD and USE_2004_LCD must be set
 */
char sOverflowString[9] = "Overflow";
void MeasureVoltageAndResistance() {
    ResistanceMeasurementResultStruct tResistanceMeasurementResult;

    // to enable discharge of stray capacitance
    pinMode(MEASUREMENT_PIN, OUTPUT);
    digitalWrite(VOLTAGE_GROUND_PIN, LOW);
#if defined(DEBUG)
    uint16_t tVCCVoltageMillivolt = printVCCVoltageMillivolt(&Serial);
#else
    uint16_t tVCCVoltageMillivolt = getVCCVoltageMillivolt();
#endif
    pinMode(MEASUREMENT_PIN, INPUT);

    uint16_t tInputVoltageMillivolt = measureVoltage(tVCCVoltageMillivolt); // VCC to compute voltage which is always measured with VCC as reference
    char tStringForPrint[9];

    if (tInputVoltageMillivolt > 4) {
        /*
         * Print voltage result
         */
        float tInputVoltage = tInputVoltageMillivolt;
        tInputVoltage /= 1000;
        // The dtostrf() requires around 2.1 kByte code
        dtostrf(tInputVoltage, 8, 3, tStringForPrint); // to have the same layout as for kOhm
        Serial.print(tStringForPrint);
#if defined(PRINT_OF_VCC)
        Serial.print(F(" V,"));
#else
        Serial.println(F(" V"));
#endif

#if defined(USE_LCD)
        LCD_OBJECT_NAME.setCursor(VOLTAGE_RESISTANCE_START_COLUMN, VOLTAGE_RESISTANCE_ROW);
        LCD_OBJECT_NAME.print(tStringForPrint);
        LCD_OBJECT_NAME.print(F(" V ")); // clears old resistance Ohm symbol
#endif

#if defined(PRINT_OF_VCC)
        float tVCC = tVCCVoltageMillivolt;
        tVCC /= 1000;
        dtostrf(tVCC, 8, 3, tStringForPrint); // to have the same layout as for kOhm
        Serial.print(tStringForPrint);
        Serial.println(F(" VCC"));
#  if defined(USE_LCD)
        LCD_OBJECT_NAME.setCursor(VCC_COLUMN, VCC_ROW);
        LCD_OBJECT_NAME.print(tStringForPrint);
        LCD_OBJECT_NAME.print(F(" VCC")); // clears old resistance Ohm symbol
#  endif
#endif
    } else {
        /*
         * Print kiloOhm output
         */
        char *tPrintStringPointer;
        if (measureResistance(tVCCVoltageMillivolt, &tResistanceMeasurementResult)) {
            tPrintStringPointer = sOverflowString;
        } else {
            float tResistance = tResistanceMeasurementResult.ResistanceOhm;
            tResistance /= 1000;
            dtostrf(tResistance, 8, 3, tStringForPrint);
            tPrintStringPointer = tStringForPrint;
        }
        Serial.print(tPrintStringPointer);
        Serial.print(F(" kOhm"));

        /*
         * Set all outputs back to inputs
         */
        setDirectVoltageMode();

#if defined(USE_LCD)
        LCD_OBJECT_NAME.setCursor(VOLTAGE_RESISTANCE_START_COLUMN, VOLTAGE_RESISTANCE_ROW);
        LCD_OBJECT_NAME.print(tPrintStringPointer);
        LCD_OBJECT_NAME.print(F(" k\xF4"));
#endif
#if defined(PRINT_OF_RESISTOR_MEASURMENT_VOLTAGE)
        Serial.print(F(" at: "));
        Serial.print(tResistanceMeasurementResult.VoltageAtResistor, 3);
        Serial.println(F(" V"));
//        Serial.println(tResistanceRange);

#  if defined(USE_LCD)
        LCD_OBJECT_NAME.setCursor(RESISTOR_MEASURMENT_VOLTAGE_COLUMN, RESISTOR_MEASURMENT_VOLTAGE_ROW);
#    if defined(USE_2004_LCD) && RESISTOR_MEASURMENT_VOLTAGE_COLUMN == 12
        LCD_OBJECT_NAME.print(F(" @"));
#    else
        LCD_OBJECT_NAME.print(F("at: "));
#    endif
        LCD_OBJECT_NAME.print(tResistanceMeasurementResult.VoltageAtResistor, 3);
        LCD_OBJECT_NAME.print(F(" V ")); // Overwrite VCC
#  endif
#endif
    }
}

void printVoltageAndResistanceUsage() {
    /*
     * Print usage
     */
    Serial.println();
    Serial.println(F("Measurement pin is " STR(MEASUREMENT_PIN)));
    Serial.println(F("Connect input with " STR(RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM) " kOhm to " STR(MEASUREMENT_PIN)));
    Serial.println(F("Connect input with " STR(RESISTOR_2_TO_INPUT_KOHM) " kOhm to " STR(RESISTOR_2_PIN)));
    Serial.println(F("Connect input with " STR(RESISTOR_3_TO_INPUT_KOHM) " kOhm to " STR(RESISTOR_3_PIN)));
    Serial.println(
            F(
                    "Connect " STR(MEASUREMENT_PIN) " with " STR(RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM) " kOhm to " STR(VOLTAGE_GROUND_PIN)));
    Serial.println();

    /*
     * get VCC and print resolution
     */
    delay(200);
    Serial.println(F("Test VCC reading 2 times. Resolution is only 20 mV!"));
    printVCCVoltageMillivolt(&Serial);
    delay(100);
    uint16_t tVCCVoltageMillivolt = printVCCVoltageMillivolt(&Serial);
    Serial.print(F("Maximum input voltage is "));
    Serial.print(
            (tVCCVoltageMillivolt * (uint32_t) ((RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM + RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM)))
                    / RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM);
    Serial.print(F(" mV with resolution of "));
    Serial.print(
            (tVCCVoltageMillivolt * (uint32_t) ((RESISTOR_INPUT_TO_MEASUREMENT_PIN_KOHM + RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM)))
                    / (1023 * RESISTOR_FROM_MEASUREMENT_PIN_TO_GROUND_PIN_KOHM));
    Serial.println(F(" mV"));
    Serial.print(F("Below "));
    Serial.print(tVCCVoltageMillivolt);
    Serial.println(F(" mV resolution is 5 mV, below 1050 mV resolution is 1 mV"));
    Serial.println();
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _MEASURE_VOLTAGE_AND_RESISTANCE_HPP
