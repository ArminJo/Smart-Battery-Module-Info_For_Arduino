/*
 *  MeasureVoltageAndResistance.cpp.h
 *
 *  Measures voltage and resistance with 6 mV and 2 Ohm resolution at the lower end.
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 *  SCHEMATIC
 *            +----o A3 open/VCC(R) | open(U)
 *            |
 *            -
 *           | |
 *           | | R3 (10 kOhm)
 *           | |
 *            -
 *            |     _____
 * Input <----+----|_____|---o input(R) | open(U)
 *            |     100 k - just to protect the pin
 *    ^       -
 *    |      | |
 *    |      | | R1 (100 kOhm)
 *    |      | |
 *    -       -
 *   | |      |
 *   | | Rx   +---o A0 VCC(R) | input(U)
 *   | |      |
 *    -       -
 *    |      | |
 *    o GND  | | R2 (22k)
 *           | |
 *            -
 *            |
 *            +---o A2 open(R) | GND(U)
 *
 *  The ratio of R1 to Rx is equal the Ratio of (1023 - x) to x
 *
 *  => The formula is: Rx/R1 = x / (1023-x)
 *      Rx = R1 * x / (1023-x)
 *
 */
#include <Arduino.h>

#include "ADCUtils.h"

/*
 * Voltmeter+Ohmmeter connections and resistors
 */
#define VOLTAGE_IN_AND_RESISTOR_1_2_PIN A0
#define VOLTAGE_CHANNEL                  0
#define OHM_PIN                         A1
#define OHM_CHANNEL                      1
#define VOLTAGE_GROUND_PIN              A2
#define RESISTOR_3_PIN                  A3

// Fixed attenuator for voltage measurement
#ifndef RESISTOR_TO_VOLTAGE_PIN_KOHM
#define RESISTOR_TO_VOLTAGE_PIN_KOHM   100 // R1
#endif
#ifndef RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM
#define RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM  22 // R2
#endif
// Voltage measurement: Threshold input voltage to switch to 1.1 volt reference
#define INPUT_MILLIVOLT_FOR_OUTPUT_1_VOLT ((1000L * (RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM)) / RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM)

// fixed resistors for resistor measurement
#ifndef RESISTOR_2_TO_VCC_KOHM
#define RESISTOR_2_TO_VCC_KOHM    10 // R3
#endif
#define RESISTOR_1_TO_VCC_KOHM    RESISTOR_TO_VOLTAGE_PIN_KOHM // R1
#define REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT 1050L // Resistor measurement: if the input voltage is below this value, use the internal 1.1 volt reference

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// function prototypes just for documentation
void setVoltageMode();
uint16_t measureVoltage(uint16_t tVCCVoltageMillivolt);
void setResistorMode();
uint32_t measureResistance(uint16_t tVCCVoltageMillivolt, uint16_t *rInputVoltageAddress);
void MeasureVoltageAndResistance();
void printVoltageAndResistanceUsage();

//#define DEBUG

void setVoltageMode() {
    pinMode(VOLTAGE_IN_AND_RESISTOR_1_2_PIN, INPUT);
    pinMode(RESISTOR_3_PIN, INPUT);
    pinMode(VOLTAGE_GROUND_PIN, OUTPUT);
    digitalWrite(VOLTAGE_GROUND_PIN, LOW);
}

uint16_t measureVoltage(uint16_t tVCCVoltageMillivolt) {
//    uint8_t tVoltageRange = 0;
    setVoltageMode();
    /*
     * We must wait for ADC channel to switch from VCC measurement channel to A0 channel
     */
    checkAndWaitForReferenceAndChannelToSwitch(VOLTAGE_CHANNEL, DEFAULT);
    uint16_t tInputVoltageRaw = readADCChannelWithReference(VOLTAGE_CHANNEL, DEFAULT);

    uint16_t tInputVoltageMillivolt = (tVCCVoltageMillivolt * (uint32_t) tInputVoltageRaw
            * (RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM))
            / (1023 * RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM);

    if (tInputVoltageMillivolt < INPUT_MILLIVOLT_FOR_OUTPUT_1_VOLT) {
        /*
         * voltage at ADC input is below 1.0 volt, so we can switch to
         * the internal 1.1 volt reference to get a better resolution (around 4 times better => ~ 6 mV)
         */
        checkAndWaitForReferenceAndChannelToSwitch(VOLTAGE_CHANNEL, INTERNAL);
        tInputVoltageRaw = readADCChannelWithReference(VOLTAGE_CHANNEL, INTERNAL);
        tInputVoltageMillivolt = (1100L * tInputVoltageRaw
                * (RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM))
                / (1023 * RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM);
//        tVoltageRange = 1;
    }
#ifdef DEBUG
        Serial.print(F("Raw="));
        Serial.println(tInputVoltageRaw);
#endif
    return tInputVoltageMillivolt;
}

void setResistorMode() {
    pinMode(VOLTAGE_GROUND_PIN, INPUT);
    pinMode(VOLTAGE_IN_AND_RESISTOR_1_2_PIN, OUTPUT); // == VOLTAGE_PIN
    digitalWrite(VOLTAGE_IN_AND_RESISTOR_1_2_PIN, HIGH);
}

uint32_t measureResistance(uint16_t aVCCVoltageMillivolt, uint16_t *rInputVoltageAddress) {
    setResistorMode();

//uint8_t tResistanceRange = 0;

    /*
     * We must wait for ADC channel to switch from VCC measurement channel to A1 channel
     */
    checkAndWaitForReferenceAndChannelToSwitch(OHM_CHANNEL, DEFAULT);
    uint16_t tInputReading = readADCChannelWithReference(OHM_CHANNEL, DEFAULT);
    uint16_t tInputVoltage = (uint32_t) tInputReading * aVCCVoltageMillivolt / 1023;
    uint16_t tReadingAtVCC = 1023;
    uint32_t tRxOhm;

    if (tInputVoltage > REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT) {
        if (tReadingAtVCC > tInputReading) {
            tRxOhm = (RESISTOR_1_TO_VCC_KOHM * 1000L * tInputReading) / (tReadingAtVCC - tInputReading);
            // Here we have a resolution of 160 to 350 Ohm at 1 MOhm
            // Clip at 10 MOhm
            if (tRxOhm > 9999999) {
                tRxOhm = 9999999;
            }
        } else {
            tRxOhm = 9999999;
        }
    } else {
//    tResistanceRange = 1;
        /*
         * Switch to 1.1 volt reference increasing the resolution by around 4
         * This happens at around 28 kOhm (at 4.7 volt) depending on the current value of VCC
         * Here we have a resolution of 24 to 37 Ohm
         */
        checkAndWaitForReferenceAndChannelToSwitch(OHM_CHANNEL, INTERNAL);
        tInputReading = readADCChannelWithReference(OHM_CHANNEL, INTERNAL);

        // The compensated VCC reading at 1.1 volt reference
        tReadingAtVCC = (aVCCVoltageMillivolt * 1023L) / 1100;

        tRxOhm = (RESISTOR_1_TO_VCC_KOHM * 1000L * tInputReading) / (tReadingAtVCC - tInputReading);

        /*
         * Formula is: (for 5 V and 1050 mV, in order to get a constant value)
         * Rx = Rvcc * 1.050 V / 3.95 V = 2416 Ohm
         * Here we have a resolution of 2 to 6 Ohm
         */
        const uint16_t tResistanceForThresholdVoltage = (RESISTOR_1_TO_VCC_KOHM * RESISTOR_2_TO_VCC_KOHM
                * REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT * 1000)
                / ((RESISTOR_1_TO_VCC_KOHM + RESISTOR_2_TO_VCC_KOHM) * (5000 - REFERENCE_SWITCHING_VOLTAGE_THRESHOLD_MILLIVOLT));
        if (tRxOhm < tResistanceForThresholdVoltage) {
#ifdef DEBUG
    Serial.print(tResistanceForThresholdVoltage);
    Serial.print(F(" Ohm "));
#endif
//        tResistanceRange = 2;
            /*
             * Switch on a parallel resistor to VCC increasing the resolution by another factor of 4.
             */
            pinMode(RESISTOR_3_PIN, OUTPUT);
            digitalWrite(RESISTOR_3_PIN, HIGH);
            delay(2);
            checkAndWaitForReferenceAndChannelToSwitch(OHM_CHANNEL, INTERNAL);
            tInputReading = readADCChannelWithReference(OHM_CHANNEL, INTERNAL);
            tRxOhm = (((RESISTOR_1_TO_VCC_KOHM * RESISTOR_2_TO_VCC_KOHM) / (RESISTOR_1_TO_VCC_KOHM + RESISTOR_2_TO_VCC_KOHM))
                    * 1000L * tInputReading) / (tReadingAtVCC - tInputReading);
            digitalWrite(RESISTOR_3_PIN, LOW);
            pinMode(RESISTOR_3_PIN, INPUT);
        }
        /*
         * Input voltage for 1.1 volt reference
         */
        tInputVoltage = tInputReading * 1100L / 1023;
    }
#ifdef DEBUG
    Serial.print(tInputReading);
    Serial.println(F(" LSB"));
#endif

    *rInputVoltageAddress = tInputVoltage;
    return tRxOhm;
}

/*
 * Convenience function
 * For LCD output, it requires the macros USE_LCD and USE_2004_LCD to be set
 */
void MeasureVoltageAndResistance() {
#ifdef DEBUG
    uint16_t tVCCVoltageMillivolt = printVCCVoltageMillivolt(&Serial);
#else
    uint16_t tVCCVoltageMillivolt = getVCCVoltageMillivolt();
#endif
    uint16_t tInputVoltageMillivolt = measureVoltage(tVCCVoltageMillivolt);

    if (tInputVoltageMillivolt > 0) {
        /*
         * Print voltage result
         */
        float tInputVoltage = tInputVoltageMillivolt;
        tInputVoltage /= 1000;
        char tString[8];
        // The dtostrf() requires around 2.1 kByte code
        dtostrf(tInputVoltage, 6, 3, tString);
        Serial.print(tString);
        Serial.println(F(" V "));

        /*
         * This can reduce code size if formatting requirements are relaxed and dtostrf() is only used here
         */
//        if (tInputVoltageMillivolt >= 1000) {
//            Serial.print(tInputVoltageMillivolt / 1000);
//            tInputVoltageMillivolt %= 1000;
//            Serial.print('.');
//            Serial.print(tInputVoltageMillivolt % 1000);
//            Serial.print(F(" V "));
//        } else {
//            Serial.print(tInputVoltageMillivolt);
//            Serial.print(F(" mV "));
//        }
#if defined(USE_LCD)
#  if defined(USE_2004_LCD)
        myLCD.setCursor(0, 3);
        myLCD.print(F("  "));
        myLCD.print(tString);
        myLCD.print(F(" V          "));
#  else
        myLCD.setCursor(0, 0);
        myLCD.print(tString);
        myLCD.print(F(" V        "));
        myLCD.setCursor(0, 1);
        myLCD.print(F("                "));
#  endif
#endif
    } else {
        uint16_t tInputVoltage;
        uint32_t tRxOhm = measureResistance(tVCCVoltageMillivolt, &tInputVoltage);

        /*
         * Format Ohm output
         */
        float tResistance = tRxOhm;
        char tString[9];

        tResistance /= 1000;
        dtostrf(tResistance, 8, 3, tString);
        Serial.print(tString);
        Serial.print(F(" kOhm at "));

#if defined(USE_LCD)
#  if defined(USE_2004_LCD)
        myLCD.setCursor(0, 3);
        myLCD.print(tString);
        myLCD.print(F(" k\xF4 @"));
#  else
        myLCD.setCursor(0, 0);
        myLCD.print(tString);
        myLCD.print(F(" k\xF4    "));
#  endif
#endif

        /*
         * This can reduce code size if formatting requirements are relaxed and dtostrf() is only used here
         */
//        if (tRxOhm >= 1000) {
//            Serial.print(tRxOhm / 1000);
//            tRxOhm %= 1000;
//            Serial.print('.');
//            Serial.print(tRxOhm % 1000);
//            Serial.print(F(" kOhm at "));
//        } else {
//            Serial.print(tRxOhm);
//            Serial.print(F(" Ohm at "));
//        }
        // the sprintf_P() function requires 1.4 kByte code
        sprintf_P(tString, PSTR("%4u"), tInputVoltage);
        Serial.print(tString);
        Serial.println(F(" mV "));
//        Serial.println(tResistanceRange);

#if defined(USE_LCD)
#  if defined(USE_2004_LCD)
        myLCD.print(tString);
        myLCD.print(F(" mV"));
#  else
        myLCD.setCursor(0, 1);
        myLCD.print(F("at: "));
        myLCD.print(tString);
        myLCD.print(F(" mV     "));
#  endif
#endif
    }
}

void printVoltageAndResistanceUsage() {
    /*
     * Print usage
     */
    Serial.println();
    Serial.println(F("Connect " STR(OHM_PIN) " with 100 kOhm to Input"));
    Serial.println(F("Connect input with " STR(RESISTOR_1_TO_VCC_KOHM) " kOhm to " STR(VOLTAGE_PIN)));
    Serial.println(F("Connect input with " STR(RESISTOR_2_TO_VCC_KOHM) " kOhm to " STR(RESISTOR_3_PIN)));
    Serial.println(
            F(
                    "Connect " STR(VOLTAGE_PIN) " with " STR(RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM) " kOhm to " STR(VOLTAGE_GROUND_PIN)));
    Serial.println();
    Serial.println(F("Resistance measurement pin is " STR(OHM_PIN)));
    Serial.println();
    /*
     * get VCC
     */
    delay(200);
    Serial.println(F("Test VCC reading 2 times. Resolution is only 20 mV!"));
    printVCCVoltageMillivolt(&Serial);
    delay(100);
    uint16_t tVCCVoltageMillivolt = printVCCVoltageMillivolt(&Serial);
    Serial.println(F("Voltage measurement pin is " STR(VOLTAGE_PIN)));
    Serial.print(F("Maximum input voltage is "));
    Serial.print(
            (tVCCVoltageMillivolt * (uint32_t) ((RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM)))
                    / RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM);
    Serial.println(F(" mV"));
    Serial.print(F("Resolution is "));
    Serial.print(
            (tVCCVoltageMillivolt * (uint32_t) ((RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM)))
                    / (1023 * RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM));
    Serial.print(F(" mV, below "));
    Serial.print(
            (1000L * (RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM))
                    / RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM);
    Serial.print(F(" mV resolution is "));
    Serial.print(
            (1100L * 100 * (RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM))
                    / (1023 * RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM) / 100);
    Serial.print('.');
    Serial.print(
            (1100L * 100 * (RESISTOR_TO_VOLTAGE_PIN_KOHM + RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM))
                    / (1023 * RESISTOR_FROM_VOLTAGE_PIN_TO_GROUND_PIN_KOHM) % 100);
    Serial.println(F(" mV"));
    Serial.println();
}
