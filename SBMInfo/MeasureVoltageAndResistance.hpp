/*
 *  MeasureVoltageAndResistance.hpp
 *
 *  Measures voltage and resistance with 1 mV and 2 ohm resolution at the lower end.
 *  First voltage is measured. If voltage is zero, then resistance to ground is measured using 5 volt (VCC) and 10 kOhm or 100 kOhm supply.
 *
 *  Copyright (C) 2021  Armin Joachimsmeyer
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
 *           | | R3 (10 kOhm)
 *           | |
 *            -
 *            |     _____
 * Input <----+----|_____|---o  A1 input (for R measurement) | open (for U measurement)
 *   ^        |     100 k - just to protect the pin
 *   |        -
 *   |       | |
 *   |       | | R1 (100 kOhm)
 *   |       | |
 *   -        -
 *  | |       |
 *  | | Rx    +---o A0  VCC(R) | input(U)
 *  | |       |
 *   -        -
 *   |       | |
 *   o GND   | | R2 (22k)
 *           | |
 *            -
 *            |
 *            +---o A2  open(R) | open/GND(U)
 *
 *  The ratio of R1 to Rx is equal the Ratio of (1023 - x) to x
 *
 *  => The formula is: Rx/R1 = x / (1023-x)
 *      Rx = R1 * x / (1023-x)
 *
 */

#ifndef _MEASURE_VOLTAGE_AND_RESISTANCE_HPP
#define _MEASURE_VOLTAGE_AND_RESISTANCE_HPP

#include <Arduino.h>

#include "ADCUtils.hpp"

//#define NO_PRINT_OF_RESISTOR_MEASURMENT_VOLTAGE // enables print of voltage at resistor under measurement (0 to VCC).

/*
 * Voltmeter+Ohmmeter connections and resistors
 */
#define VOLTAGE_MEASUREMENT_PIN         A0
#define VOLTAGE_CHANNEL                  0
#define OHM_PIN                         A1
#define OHM_CHANNEL                      1
#define VOLTAGE_GROUND_PIN              A2
#define RESISTOR_3_PIN                  A3

#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT)
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100L // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif

// Fixed attenuator for voltage measurement
#if !defined(RESISTOR_TO_VOLTAGE_PIN_KOHM)
#define RESISTOR_TO_VOLTAGE_PIN_KOHM   100 // R1
#endif
#if !defined(RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM)
#define RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM  22 // R2
#endif

// fixed resistors for resistor measurement
#if !defined(RESISTOR_2_TO_VCC_KOHM)
#define RESISTOR_2_TO_VCC_KOHM    10 // R3
#endif
#define RESISTOR_1_TO_VCC_KOHM    RESISTOR_TO_VOLTAGE_PIN_KOHM // R1
#define REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT 1050L // Resistor measurement: if the input voltage is below this value, use the internal 1.1 volt reference

struct ResistanceMeasurementResultStruct {
    uint32_t ResistanceOhm;
    float VoltageAtResistor;
    bool isOverflow;
};

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// function prototypes just for documentation
void setVoltageMode();
uint16_t measureVoltage(uint16_t tVCCVoltageMillivolt);
void setResistorMode();
bool measureResistance(uint16_t aVCCVoltageMillivolt, ResistanceMeasurementResultStruct *aResistanceMeasurementResult);
void MeasureVoltageAndResistance();
void printVoltageAndResistanceUsage();

//#define DEBUG

void setVoltageMode() {
    pinMode(VOLTAGE_MEASUREMENT_PIN, INPUT);
    pinMode(RESISTOR_3_PIN, INPUT);
    pinMode(VOLTAGE_GROUND_PIN, INPUT);
}

uint16_t measureVoltage(uint16_t tVCCVoltageMillivolt) {
//    uint8_t tVoltageRange = 0;
    setVoltageMode();
    /*
     * We must wait for ADC channel to switch from VCC measurement channel to A0 channel
     */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

    uint16_t tInputVoltageMillivolt = getVoltageMillivolt(tVCCVoltageMillivolt, VOLTAGE_CHANNEL);
    if (tInputVoltageMillivolt >= tVCCVoltageMillivolt) {
        /*
         * Voltage > VCC -> enable Attenuator
         */
        pinMode(VOLTAGE_GROUND_PIN, OUTPUT);
        digitalWrite(VOLTAGE_GROUND_PIN, LOW);
        tInputVoltageMillivolt = getVoltageMillivolt(tVCCVoltageMillivolt, VOLTAGE_CHANNEL);
        tInputVoltageMillivolt = (((uint32_t) tInputVoltageMillivolt)
                * (RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM))
                / (RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM);
    } else {
        /*
         * Voltage at ADC input is below VCC
         */
        if (tInputVoltageMillivolt < (ADC_INTERNAL_REFERENCE_MILLIVOLT - 50)) {
            /*
             * Voltage at ADC input is below 1.05 volt, so we can switch to
             * the internal 1.1 volt reference to get a better resolution (around 4 times better => ~ 1 mV)
             */
            tInputVoltageMillivolt = getVoltageMillivoltWith_1_1VoltReference(VOLTAGE_CHANNEL);
//        tVoltageRange = 1;
        }
    }
#pragma GCC diagnostic pop
#if defined(DEBUG)
        Serial.print(F("Raw="));
        Serial.println(tInputVoltageRaw);
#endif
    return tInputVoltageMillivolt;
}

void setResistorMode() {
    pinMode(VOLTAGE_GROUND_PIN, INPUT);
    pinMode(VOLTAGE_MEASUREMENT_PIN, OUTPUT);
    digitalWrite(VOLTAGE_MEASUREMENT_PIN, HIGH);
}

bool measureResistance(uint16_t aVCCVoltageMillivolt, ResistanceMeasurementResultStruct *aResistanceMeasurementResult) {
    setResistorMode();

//uint8_t tResistanceRange = 0;

    /*
     * We must wait for ADC channel to switch from VCC measurement channel to A1 channel
     */
    uint16_t tInputReading = waitAndReadADCChannelWithReference(OHM_CHANNEL, DEFAULT);
    uint16_t tInputVoltage = (uint32_t) tInputReading * aVCCVoltageMillivolt / READING_FOR_AREF;
    uint16_t tReadingAtVCCReference = 1023;
    uint32_t tRxOhm;

    aResistanceMeasurementResult->isOverflow = false;
    if (tInputVoltage > REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT) {
        if (tReadingAtVCCReference > tInputReading) {
            tRxOhm = (RESISTOR_1_TO_VCC_KOHM * 1000L * tInputReading) / (tReadingAtVCCReference - tInputReading);
            // Here we have a resolution of 160 to 350 ohm at 1 MOhm
            // Clip at 10 MOhm
            if (tRxOhm > 9999999) {
                tRxOhm = 9999999;
                aResistanceMeasurementResult->isOverflow = true;
            }
        } else {
            tRxOhm = 9999999;
            aResistanceMeasurementResult->isOverflow = true;
        }
    } else {
//    tResistanceRange = 1;
        /*
         * Switch to 1.1 volt reference increasing the resolution by around 4
         * This happens at around 28 kOhm (at 4.7 volt) depending on the current value of VCC
         * Here we have a resolution of 24 to 37 ohm
         */
        tInputReading = waitAndReadADCChannelWithReference(OHM_CHANNEL, INTERNAL);

        // The compensated VCC reading at 1.1 volt reference
        tReadingAtVCCReference = (aVCCVoltageMillivolt * READING_FOR_AREF) / 1100;

        tRxOhm = (RESISTOR_1_TO_VCC_KOHM * 1000L * tInputReading) / (tReadingAtVCCReference - tInputReading);

        /*
         * Formula is: (for 5 V and 1050 mV, in order to get a constant value)
         * Rx = Rvcc * 1.050 V / 3.95 V = 2416 ohm
         * Here we have a resolution of 2 to 6 ohm
         */
        const uint16_t tResistanceForThresholdVoltage = (RESISTOR_1_TO_VCC_KOHM * RESISTOR_2_TO_VCC_KOHM
                * REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT * 1000)
                / ((RESISTOR_1_TO_VCC_KOHM + RESISTOR_2_TO_VCC_KOHM) * (5000 - REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT));
        if (tRxOhm < tResistanceForThresholdVoltage) {
#if defined(DEBUG)
    Serial.print(tResistanceForThresholdVoltage);
    Serial.print(F(" ohm "));
#endif
//        tResistanceRange = 2;
            /*
             * Switch on a parallel resistor to VCC increasing the resolution by another factor of 4.
             */
            pinMode(RESISTOR_3_PIN, OUTPUT);
            digitalWrite(RESISTOR_3_PIN, HIGH);
            delay(2);
            tInputReading = waitAndReadADCChannelWithReference(OHM_CHANNEL, INTERNAL);
            tRxOhm = (((RESISTOR_1_TO_VCC_KOHM * RESISTOR_2_TO_VCC_KOHM) / (RESISTOR_1_TO_VCC_KOHM + RESISTOR_2_TO_VCC_KOHM))
                    * 1000L * tInputReading) / (tReadingAtVCCReference - tInputReading);
            digitalWrite(RESISTOR_3_PIN, LOW);
            pinMode(RESISTOR_3_PIN, INPUT);
        }
        /*
         * Input voltage for 1.1 volt reference
         */
        tInputVoltage = tInputReading * 1100L / READING_FOR_AREF;
    }
#if defined(DEBUG)
    Serial.print(tInputReading);
    Serial.println(F(" LSB"));
#endif

    aResistanceMeasurementResult->VoltageAtResistor = ((float) tInputVoltage) / 1000.0;
    aResistanceMeasurementResult->ResistanceOhm = tRxOhm;

    /*
     * Set all outputs back to inputs
     */
    setVoltageMode();

    return aResistanceMeasurementResult->isOverflow;
}

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
    pinMode(VOLTAGE_MEASUREMENT_PIN, OUTPUT);
    digitalWrite(VOLTAGE_GROUND_PIN, LOW);
#if defined(DEBUG)
    uint16_t tVCCVoltageMillivolt = printVCCVoltageMillivolt(&Serial);
#else
    uint16_t tVCCVoltageMillivolt = getVCCVoltageMillivolt();
#endif
    uint16_t tInputVoltageMillivolt = measureVoltage(tVCCVoltageMillivolt);
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
        Serial.println(F(" V"));

#if defined(USE_LCD)
#  if defined(USE_2004_LCD)
        myLCD.setCursor(0, 3);
        myLCD.print(tStringForPrint);
#    if !defined(NO_PRINT_OF_RESISTOR_MEASURMENT_VOLTAGE)
        myLCD.print(F(" V          ")); // clears old resistor output
#    else
        myLCD.print(F(" V ")); // clears old resistor output
#    endif
#  else
        myLCD.setCursor(0, 0);
        myLCD.print(tStringForPrint);
        myLCD.print(F(" V      "));
        myLCD.setCursor(0, 1);
        myLCD.print(F("                "));
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

#if defined(USE_LCD)
#  if defined(USE_2004_LCD)
        myLCD.setCursor(0, 3);
        myLCD.print(tPrintStringPointer);
        myLCD.print(F(" k\xF4"));
#  else
        myLCD.setCursor(0, 0);
        myLCD.print(tPrintStringPointer);
        myLCD.print(F(" k\xF4    "));
#  endif
#endif
#if !defined(NO_PRINT_OF_RESISTOR_MEASURMENT_VOLTAGE)
        Serial.print(F(" at: "));
        Serial.print(tResistanceMeasurementResult.VoltageAtResistor, 3);
        Serial.println(F(" V"));
//        Serial.println(tResistanceRange);

#  if defined(USE_LCD)
#    if defined(USE_2004_LCD)
        myLCD.print(F(" @"));
        myLCD.print(tResistanceMeasurementResult.VoltageAtResistor, 3);
        myLCD.print(F(" V"));
#    else
        myLCD.setCursor(0, 1);
        myLCD.print(F("at: "));
        myLCD.print(tResistanceMeasurementResult.VoltageAtResistor, 3);
        myLCD.print(F(" V     "));
#    endif
#  endif
#endif
        Serial.println();
    }
}

void printVoltageAndResistanceUsage() {
    /*
     * Print usage
     */
    Serial.println();
    Serial.println(F("Connect " STR(OHM_PIN) " with 100 kOhm to input"));
    Serial.println(F("Connect input with " STR(RESISTOR_1_TO_VCC_KOHM) " kOhm to " STR(VOLTAGE_MEASUREMENT_PIN)));
    Serial.println(F("Connect input with " STR(RESISTOR_2_TO_VCC_KOHM) " kOhm to " STR(RESISTOR_3_PIN)));
    Serial.println(
            F(
                    "Connect " STR(VOLTAGE_MEASUREMENT_PIN) " with " STR(RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM) " kOhm to " STR(VOLTAGE_GROUND_PIN)));
    Serial.println();

    /*
     * get VCC and print resolution
     */
    delay(200);
    Serial.println(F("Test VCC reading 2 times. Resolution is only 20 mV!"));
    printVCCVoltageMillivolt(&Serial);
    delay(100);
    uint16_t tVCCVoltageMillivolt = printVCCVoltageMillivolt(&Serial);
    Serial.println(F("Voltage measurement pin is " STR(VOLTAGE_MEASUREMENT_PIN)));
    Serial.print(F("Maximum input voltage is "));
    Serial.print(
            (tVCCVoltageMillivolt * (uint32_t) ((RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM)))
                    / RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM);
    Serial.print(F(" mV with resolution of "));
    Serial.print(
            (tVCCVoltageMillivolt * (uint32_t) ((RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM)))
                    / (1023 * RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM));
    Serial.println(F(" mV"));
    Serial.print(F("Below "));
    Serial.print(tVCCVoltageMillivolt);
    Serial.println(F(" mV resolution is 5 mV, below 1050 mV resolution is 1 mV"));
    Serial.println();
}

#endif // _MEASURE_VOLTAGE_AND_RESISTANCE_HPP
