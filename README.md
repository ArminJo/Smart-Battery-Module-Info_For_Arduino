# SMB - Smart Battery Module (Laptop Battery Pack) Info

Gibt die Daten des SMB Controllers aus.

Basiert auf https://github.com/PowerCartel/PackProbe von Power Cartel http://powercartel.com/projects/packprobe/. Hier gibt es auch weitere wertvolle Informationen.

Benötigt SoftI2CMaster Library für I2C / SMBus / https://github.com/felias-fogg/SoftI2CMaster/archive/master.zip

## Finden der Anschlüsse.
Nach dem Booten sucht das Programm nach einem angeschlossenen I2C Device.
Man kann also alle möglichen Pinkombinationen von Clock und Data am Battery Pack ausprobieren.
Bei der Richtigen hört das Blinken der Led auf und es kommt sofort die Ausgabe "Found I2C device attached at address: ox0B" und direkt danach werden die Daten ausgegeben.

Bei den Laptop Battery Packs war Plus und Masse immer außen.
Wenn mehr als 5 Kontakte vorhanden waren, waren sie wie folgt belegt:
- Masse und Plus doppelt. z.B. + | + | Thermo | Data | Clock | - | -
- Ein Enable (nur im Laptop mit Masse verbunden) und eine Signal Anschluss (nur im Battery Pack mit Masse verbunden). z.B. + | + | Clock | Data | Signal | Enable | Thermo | - | -

Die Clock und Data Eingänge waren bei meinen Packs die Anschlüsse mit einem Widerstand von ca. 300 k bis 1 MOhm nach Masse.
Der Thermo Sensor Anschluss war uneinheitlich, mal nicht messabar beschaltet, mal 1 MOhm, mal 1,6 Volt, mal 10 kOhm nach Masse.

Zur Verbindung mit den Kontakten habe ich normales 1,5 qmm Kupferkabel aus der Hausinstallation genommen, dessen eines Ende ich mit einem Hammer etwas plattgeklopft hab. Stecknadeln oder Breadboard Wires gehen auch.

Die Daten werden nur einmalig nach dem Reset ausgegeben, nur die veränderlichen Werte wie Spannung / Strom / Temperatur / Ladung werden alle 3 Sekunden auf Veränderungen gecheckt.

Tested with bq20z70, bq20z451, bq2084, bq80201DBT, bq40z50.

Einen Schaltplan zu den Batterie Modulen gibt es im Datenblatt zum TI bq29311 auf Seite 9.

![Breadboard](https://github.com/ArminJo/Smart-Battery-Module-Info_For_Arduino/blob/master/img/Breadboard.jpg)

###Beispieloutput:
I2C initalized sucessfully
Found attached I2C device at 0xB

*** STATIC INFO ***
```
Chemistry: LION
Manufacturer Name: DP
Manufacturer Data:          K24005SDI   
Device Name: bq20z451
Serial Number: 11518
Manufacture Date (YYYY-MM-DD): 2009-8-27
Design Capacity (mAh): 5450
Design Voltage: 10.950
Specification Info: 49
Cycle Count: 255
Max Error of charge calculation (%): 1
RemainingTimeAlarm (min): 10
Remaining Capacity Alarm: 300
Battery Mode (BIN): 0b110000000000001
- Internal Charge Controller Supported
- Battery OK
- Disable AlarmWarning broadcast to Host and Smart Battery Charger
- Disable broadcasts of ChargingVoltage and ChargingCurrent to Smart Battery Charger

*** MANUFACTURER INFO ***
ManufacturerAccess: 304 / 0x130
Device Type: 1105 / 0x451
Hardware Version: 0xA6
Firmware Version: 0.3
Manufacturer Status (BIN): 0b10001111
- FET Status 2
- State: 0b1111
Battery Pack removed

*** TEST INFO ***
Average Current (mA): 1
AtRateTimeToFull: 65535
AtRateTimeToEmpty: 65535
AtRateOK: 1

*** DYNAMIC INFO ***
Full Charge Capacity (mAh): 2845
Remaining Capacity (mAh): 2720
Relative Charge(%): 96
Absolute Charge(%): 50
Minutes remaining for full charge: 7500
Battery Status (BIN): 0b100100011100000
- TERMINATE_CHARGE_ALARM
- TERMINATE_DISCHARGE_ALARM
- Initialized
- Discharging
- Fully Charged
Voltage: 12.253
Current (mA): 0
Temperature (C): 24.85
State of Health: 4084
Cell 1 Voltage: 4.084
Cell 2 Voltage: 4.085
Cell 3 Voltage: 4.084

*** CHANGED VALUES ***
```
