# [SMB](https://github.com/ArminJo/Smart-Battery-Module-Info_For_Arduino) - Smart Battery Module (Laptop Battery Pack) Info
### Version 3.1.1
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/Smart-Battery-Module-Info_For_Arduino/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Smart-Battery-Module-Info_For_Arduino/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FSmart-Battery-Module-Info_For_Arduino)](https://github.com/brentvollebregt/hit-counter)

Prints SBM controller info

Based on https://github.com/PowerCartel/PackProbe from Power Cartel http://powercartel.com/projects/packprobe/.

# Disclaimer
**I do not know how to enter full access mode, clear permanent filure or unlock any controller IC.** Unfortunally according to most datasheets, you need an unlock key.
See also this [article from 2011](https://media.blackhat.com/bh-us-11/Miller/BH_US_11_Miller_Battery_Firmware_Public_WP.pdf).
Extract: *Macbook batteries ship with a default unseal password (0x36720414).  This was found by reverse engineering a Macbook battery update.  On Macbook batteries, the full access mode password is also hardcoded and default (0xffffffff).* 

## Compile with the Arduino IDE
Download and extract the repository. In the Arduino IDE open the sketch with File -> Open... and select the src/SBMInfo folder. 

## Identifying the right connection
Clock und Data connectors have often a resistance of 300 k to 1 MOhm to Ground.
After startup, the program scans for a connected I2C device.
Just try different pin combinations until led stops blinking and `Found I2C device attached at address: 0x0B` is printed.
After connecting`, full data is printed. 
Dynamic values are checked every 3 seconds and printed if changed.

Tested with bq20z70, bq20z451, bq2084, bq80201DBT, bq40z50.

An example schematic for a SBM module can be found in the datasheet of TI bq29311 at page 9.

![My setup](https://github.com/ArminJo/Smart-Battery-Module-Info_For_Arduino/blob/master/extras/Breadboard.jpg)

![My setup](https://github.com/ArminJo/Smart-Battery-Module-Info_For_Arduino/blob/master/extras/With_LCD.jpg)

## German Documentation
Gibt die Daten des SMB Controllers aus.
Basiert auf https://github.com/PowerCartel/PackProbe von Power Cartel http://powercartel.com/projects/packprobe/. Hier gibt es auch weitere wertvolle Informationen.

## Finden der Anschlüsse.
Die Clock und Data Eingänge waren bei meinen Packs die Anschlüsse mit einem Widerstand von ca. 300 k bis 1 MOhm nach Masse.
Nach dem Booten sucht das Programm nach einem angeschlossenen I2C Device.
Man kann also alle möglichen Pinkombinationen von Clock und Data am Battery Pack ausprobieren.
Bei der Richtigen hört das Blinken der Led auf und es kommt sofort die Ausgabe `Found I2C device attached at address: 0x0B` und direkt danach werden die Daten ausgegeben.

Bei den Laptop Battery Packs war Plus und Masse immer außen.
Wenn mehr als 5 Kontakte vorhanden waren, waren sie wie folgt belegt:
- Masse und Plus doppelt. z.B. + | + | Thermo | Data | Clock | - | -
- Ein Enable (nur im Laptop mit Masse verbunden) und eine Signal Anschluss (nur im Battery Pack mit Masse verbunden). z.B. + | + | Clock | Data | Signal | Enable | Thermo | - | -

Der Thermo-Sensor Anschluss war uneinheitlich, mal nicht messbar beschaltet, mal 1 MOhm, mal 1,6 Volt, mal 10 kOhm nach Masse.

Zur Verbindung mit den Kontakten habe ich normales 1,5 qmm Kupferkabel aus der Hausinstallation genommen, dessen eines Ende ich mit einem Hammer etwas plattgeklopft hab. Stecknadeln oder Breadboard Wires gehen auch.

Die Daten werden nur einmalig nach dem Reset ausgegeben, nur die veränderlichen Werte wie Spannung / Strom / Temperatur / Ladung werden alle 3 Sekunden auf Veränderungen gecheckt.

Tested with bq20z70, bq20z451, bq2084, bq80201DBT, bq40z50.

Einen Schaltplan zu den Batterie Modulen gibt es im Datenblatt zum TI bq29311 auf Seite 9.

###Sample output:
Sample outputs can be found in folder extras.

```
START ../src/SBMInfo.cpp
Version 2.1 from Oct 27 2018
I2C initalized successfully
Found attached I2C device at 0xB

*** STATIC INFO ***
Chemistry: LION
Manufacturer Name: GW
Manufacturer Data: †A;0ÿ  È ¬& - 0x86 41 3B 30 FF 1E 0 11 0 C8 0 AC 26 
Device Name:  DELL 0
Serial Number: 46
Manufacture Date (YYYY-MM-DD): 2012-9-12
Design Capacity: 6600 mAh
Design Voltage: 11.100 V
Charging Current: 4100 mA
Charging Voltage: 12.600 V
Specification Info: 33
Cycle Count: 39
Max Error of charge calculation (%): 8
RemainingTimeAlarm: 10 min
Remaining Capacity Alarm: 660 mAh
Battery Mode (BIN): 0b110000000000000
- Battery OK
- Disable AlarmWarning broadcast to Host and Smart Battery Charger
- Disable broadcasts of ChargingVoltage and ChargingCurrent to Smart Battery Charger
Pack Status (BIN): 0b1000011010010000

*** MANUFACTURER INFO ***
Device Type: 2084 / 0x824
Controller IC identified by device type: bq2084
End of Discharge Voltage Level: 9.900 V

Firmware Version: 1.50
Manufacturer Status (BIN): 0b0
- FET Status 0
- State: 0b0

*** RATE TEST INFO ***
Setting AT rate to 100 mA
TimeToFull at rate: 265 min
Setting AT rate to -100 mA
TimeToEmpty at rate: 3061 min
Can be delivered for 10 seconds at rate: 1

*** DYNAMIC INFO ***
Full Charge Capacity: 5545 mAh
Remaining Capacity: 5102 mAh
Relative Charge(%): 92
Absolute Charge(%): 77
Minutes remaining until empty: 2915 min
Average minutes remaining until empty: 2943 min
Minutes remaining for full charge: Battery not beeing (dis)charged
Battery Status (BIN): 0b11000000
- Initialized
- Discharging
Voltage: 12.212 V
Current: -105 mA
Average Current of last minute: -104 mA
Temperature: 25.95 C

*** DYNAMIC NON STANDARD INFO ***
Cell 1 Voltage: 4.074 V
Cell 2 Voltage: 4.070 V
Cell 3 Voltage: 4.068 V
Cell 4 Voltage: 0.000 V
State of Health: 0

*** CHANGED VALUES ***
Remaining Capacity: 5101 mAh
Minutes remaining until empty: 2914 min
Average minutes remaining until empty: 2942 min
Average minutes remaining until empty: 2914 min
Voltage: 12.209 V
Remaining Capacity: 5099 mAh
Minutes remaining until empty: 2913 min
Average minutes remaining until empty: 2913 min
```

# Revision History
### Version 3.1.1
- Better prints at scanning.

#### If you find this library useful, please give it a star.
