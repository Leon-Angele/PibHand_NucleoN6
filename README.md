# PIB Hand Control

Firmware fuer eine einzelne Roboterhand mit sechs STS3215-Servos auf dem
NUCLEO-N657X0-Q.

Die Hand kann entweder positionsgefuehrt oder mit einer individuellen
Admittanzregelung fuer vier Finger betrieben werden. Der Daumen bleibt als
Anker rein positionsgeregelt.

## Systemuebersicht

```text
PC / Gegenstelle
       |
       | LPUART1 / VCP, ASCII
       v
SerialCommander
       |
       v
HandController
       |                     +------------------+
       |                     | ADC1 + GPDMA     |
       |                     | 5 x FSR400       |
       |                     +--------+---------+
       |                              |
       |                              v
       |                     Kraft-/Admittanzregler
       v
ServoBus / USART3 / STS3215
```

## Hardware und Echtzeit

| Funktion | Peripherie | Einstellung |
|---|---|---|
| VCP zu Gegenstelle | LPUART1 | 460800 Baud, 8N1 |
| Servo-Bus | USART3 | 1000000 Baud |
| FSR-Sensoren | ADC1 + GPDMA | 5 Kanaele, 12 Bit |
| Abtasttrigger | TIM6 TRGO | 500 Hz, 2 ms |
| Positionstrajektorie | HandController | Smoothstep |
| Servo-Feedback | USART3 Readback | Position und Strom, round-robin |

Die ADC-Reihenfolge ist:

| FSR-Index | Finger | ADC-Kanal | Pin |
|---:|---|---|---|
| 0 | Daumen | ADC1_INP5 | PA8 |
| 1 | Zeigefinger | ADC1_INP10 | PA9 |
| 2 | Mittelfinger | ADC1_INP16 | PF3 |
| 3 | Ringfinger | ADC1_INP11 | PA10 |
| 4 | Kleiner Finger | ADC1_INP13 | PA12 |

## Servoachsen

| Achse | Index | Servo-ID | Sensor | Regelungsart |
|---|---:|---:|---:|---|
| Daumenbeugung | 0 | 1 | FSR 0 | Position |
| Zeigefinger | 1 | 2 | FSR 1 | Position + Admittanz |
| Mittelfinger | 2 | 3 | FSR 2 | Position + Admittanz |
| Ringfinger | 3 | 4 | FSR 3 | Position + Admittanz |
| Kleiner Finger | 4 | 5 | FSR 4 | Position + Admittanz |
| Daumenrotation | 5 | 6 | keiner | Position |

Positionen werden als Prozent der konfigurierten Achsbewegung gesendet:

```text
0 %   = offene Referenzposition
100 % = geschlossene Referenzposition
```

Die aktuelle Startkonfiguration verwendet fuer alle Achsen:

```text
zeroPos = 2047
maxPos  = 4095
```

## Admittanzregelung

Die Kraftregelung ist pro Finger getrennt. Jeder geregelte Finger besitzt:

- eigene Positionsreferenz `q_ref`
- eigenen virtuellen Admittanzzustand
- eigenen FSR-Kraftwert
- eigenen Kraftsollwert
- eigene Geschwindigkeits- und Positionsgrenzen

Die Posen- oder Einzelpositionsbefehle setzen den Arbeitspunkt. Nach Erreichen
der Referenzposition wird die Admittanz aktiv, wenn `ADM:ON` gesetzt wurde.

Das verwendete einseitige Modell lautet:

```text
M * q_ddot + D * q_dot + K * min(q - q_ref, 0) = F_soll - F_ist
```

Verhalten:

- Bei zu kleiner Kraft beugt der Finger weiter.
- Bei zu grosser Kraft gibt der Finger in Streckrichtung nach.
- Die virtuelle Feder wirkt nur unterhalb der Referenzposition.
- Schnelle Auslenkung erzeugt durch den Daempfer eine groessere Gegenkraft.
- Der Daumen und die Daumenrotation werden nicht durch FSR-Werte verschoben.

Startparameter:

```text
Reglertakt:              500 Hz
Virtuelle Steifigkeit K: 0.1 N/%
Eigenfrequenz:            3 Hz
Kraft-Totzone:            0.05 N
Standardgeschwindigkeit: 200 deg/s
Maximale Sollkraft:       5 N
Servo-Torque-Limit:       50 %
```

## FSR-Auswertung

Die FSR400-Sensoren werden per ADC/DMA eingelesen und mit einem IIR-Filter
geglattet. Beim Start werden unbelastete Samples fuer den Nullpunkt-Tare
gesammelt. Ein erneuter Tare ist per `FSR:TARE` moeglich.

Die Kraft wird aktuell ueber eine zentrale Datenblatt-Naeherung berechnet. Ein
FSR400 ist keine kalibrierte Loadcell. Fuer eine belastbare 1-N-Regelung muss
fuer jeden Sensor eine reale Kraftkalibrierung als LUT hinterlegt werden.

Der per VCP akzeptierte Kraftbereich ist:

```text
0.0 ... 5.0 N
```

## VCP-Steuerung

Das vollstaendige Protokoll steht in:

`VCP_HAND_SERIAL_PROTOCOL.md`

UART-Einstellung:

```text
460800 Baud, 8 Datenbits, keine Paritaet, 1 Stoppbit, kein Flow-Control
```

Beispielsequenz:

```text
FSR:TARE\r\n
POSE:4:1.0\r\n
ADM:ON\r\n
POS:1:50:1.0\r\n
STATUS:STREAM:10\r\n
```

Alle Befehle sind ASCII-Zeilen und werden mit `LF`, `CR` oder `CRLF`
abgeschlossen. Es gibt keine alten `Side`-Parameter mehr. Die Gegenstelle
steuert genau eine konfigurierte Hand.

## Statusdaten

Mit `STATUS?` oder `STATUS:STREAM:<Hz>` koennen folgende Daten abgefragt werden:

- Controller-Modus
- Fehlerflags
- Positionsreferenzen
- ausgegebene Positionssollwerte
- echte Servo-Istpositionen in Prozent und Ticks
- FSR-Rohwerte
- FSR-Kraftnaeherungen
- Servo-Iststroeme
- globale Geschwindigkeit
- globales Torque-Limit

## Wichtige Dateien

| Datei | Aufgabe |
|---|---|
| `Appli/Core/Inc/hand/hand_config.hpp` | Achsen, Servo-IDs, Posen und Limits |
| `Appli/Core/Inc/hand/hand_controller.hpp` | Controllerzustand und Statusstruktur |
| `Appli/Core/Src/hand/hand_controller.cpp` | Referenztrajektorien und Fingerregelung |
| `Appli/Core/Inc/hand/admittance_controller.hpp` | HAL-freie Reglerdefinition |
| `Appli/Core/Src/hand/admittance_controller.cpp` | Diskreter Admittanzschritt |
| `Appli/Core/Src/hand/fsr400.cpp` | ADC/DMA, Filter, Tare und Kraft-Naeherung |
| `Appli/Core/Src/hand/servo.cpp` | STS3215-Protokoll und Servo-Feedback |
| `Appli/Core/Src/hand/serial_commander.cpp` | VCP-Parser und Antwortqueue |
| `Appli/Core/Src/hand/hand_bridge.cpp` | VCP-, Controller- und Busintegration |
| `Appli/Core/Src/main.c` | STM32-Initialisierung und Mainloop |

## Build

Voraussetzung ist eine vorhandene STM32CubeIDE- beziehungsweise CubeCLT-
Toolchain mit `arm-none-eabi-g++` und MinGW Make.

Debug-Build aus dem Projektstamm:

```powershell
& 'C:\MinGW\bin\mingw32-make.exe' -C Appli/Debug -j2 all -B
```

Ohne erzwungenen Vollbuild:

```powershell
& 'C:\MinGW\bin\mingw32-make.exe' -C Appli/Debug -j2 all
```

Die Ausgaben liegen danach in `Appli/Debug`:

```text
PibHand_NucleoN6_Appli.elf
PibHand_NucleoN6_Appli.bin
PibHand_NucleoN6_Appli.map
```

## Grenzen und Inbetriebnahme

- Die FSR-N-Werte muessen vor einer sicherheitskritischen Nutzung kalibriert werden.
- Die Datenblatt-Naeherung ersetzt keine mechanische Kraftbegrenzung.
- `TORQUE:<Percent>` schreibt ein fluechtiges STS3215-SRAM-Limit und wird nach Reset erneut gesetzt.
- Ein Servo- oder Sensorfehler muss ueber die Status-Fehlerflags behandelt werden.
- Vor dem Krafttest zuerst Sensor-Tare, Servo-Ping, Torque-Limit und Einzelbewegungen pruefen.
- Die Firmware wurde gebaut, aber die Admittanz wurde noch nicht mit realer Kontaktlast auf der Hardware validiert.
