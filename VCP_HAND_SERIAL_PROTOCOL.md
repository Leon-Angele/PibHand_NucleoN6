# VCP Handsteuerungsprotokoll

Dieses Dokument beschreibt die serielle Steuerung der konfigurierten Einzelhand.
Die Gegenstelle sendet ASCII-Kommandos an LPUART1. Die Firmware berechnet
Positionstrajektorien und Admittanz und steuert die sechs STS3215-Servos ueber
USART3.

## 1. Transport

```text
Schnittstelle: LPUART1 / VCP
Baudrate:      460800
Datenbits:     8
Paritaet:      keine
Stoppbits:     1
Flow-Control:  keine
Zeichenformat: ASCII
```

Jedes Kommando ist eine Textzeile und muss mit einem Zeilenende abgeschlossen
werden:

```text
LF    = \n
CR    = \r
CRLF  = \r\n
```

`CRLF` wird als ein einziges Zeilenende behandelt. Fuer neue Gegenstellen wird
`CRLF` empfohlen.

Die VCP-Verbindung spricht nicht direkt das STS3215-Binaerprotokoll:

```text
Gegenstelle -> LPUART1 -> SerialCommander -> HandController -> USART3 -> Servos
```

## 2. Achsen, Servos und FSRs

| Achse | Index | Servo-ID | FSR-Index | Regelung |
|---|---:|---:|---:|---|
| Daumenbeugung | `0` | `1` | `0` | Position |
| Zeigefinger | `1` | `2` | `1` | Position + Admittanz |
| Mittelfinger | `2` | `3` | `2` | Position + Admittanz |
| Ringfinger | `3` | `4` | `3` | Position + Admittanz |
| Kleiner Finger | `4` | `5` | `4` | Position + Admittanz |
| Daumenrotation | `5` | `6` | keiner | Position |

Die VCP-Positionswerte sind Prozentwerte:

```text
0 %    = offene Referenzposition
100 %  = geschlossene Referenzposition
```

Die Firmware mappt diese Werte intern auf Servo-Ticks. Aktuelle Startwerte:

```text
zeroPos = 2047
maxPos  = 4095
```

## 3. FSR-Kraftmessung

Die FSR400-Sensoren werden mit ADC1 und GPDMA bei 500 Hz eingelesen. Der
Softwareindex ist:

| FSR-Index | Finger | ADC-Kanal | Pin |
|---:|---|---|---|
| `0` | Daumen | ADC1_INP5 | PA8 |
| `1` | Zeigefinger | ADC1_INP10 | PA9 |
| `2` | Mittelfinger | ADC1_INP16 | PF3 |
| `3` | Ringfinger | ADC1_INP11 | PA10 |
| `4` | Kleiner Finger | ADC1_INP13 | PA12 |

Die Firmware filtert die ADC-Werte und bildet daraus eine Newton-Naeherung.
Der FSR400 ist keine kalibrierte Loadcell. Die Newtonwerte muessen fuer eine
praezise Kraftregelung mit realen Lasten je Sensor kalibriert werden.

Der akzeptierte Kraftsollwert ist:

```text
0.0 ... 5.0 N
```

## 4. Antwortregeln

Ein erfolgreich verarbeitetes Kommando erzeugt:

```text
OK
```

Ein fehlerhaftes oder nicht ausfuehrbares Kommando erzeugt eine der folgenden
Antworten:

```text
ERR:SYNTAX
ERR:EXEC
ERR:NOEXEC
ERR:QUEUE_FULL
```

Bedeutung:

| Antwort | Bedeutung |
|---|---|
| `OK` | Kommando wurde akzeptiert |
| `ERR:SYNTAX` | Kommandoformat oder Wertebereich ist ungueltig |
| `ERR:EXEC` | Format ist gueltig, Ausfuehrung im aktuellen Zustand nicht moeglich |
| `ERR:NOEXEC` | Kein Command-Executor registriert |
| `ERR:QUEUE_FULL` | Empfangs- oder Antwortqueue ist voll |

Wichtig: Eine Antwort `OK` bestaetigt die Annahme des Kommandos. Sie bedeutet
nicht, dass die Bewegung bereits abgeschlossen ist.

## 5. Posen

### 5.1 Pose ohne Kraftaenderung

```text
POSE:<GripID>\r\n
```

Gueltige Grip-IDs:

| ID | Name |
|---:|---|
| `0` | Open |
| `1` | Spitzgriff / Zeigen |
| `2` | Dreipunktgriff |
| `3` | Schluesselgriff |
| `4` | Zylindergriff |
| `5` | Hakengriff |
| `6` | Sphaerischer Griff |
| `7` | Mittelfinger |

Beispiel:

```text
POSE:4\r\n
```

Die Positionsreferenzen aller sechs Achsen werden auf die Pose gesetzt.
Bereits gespeicherte Kraftsollwerte der Finger 1 bis 4 bleiben erhalten.

### 5.2 Pose mit gemeinsamer Sollkraft

```text
POSE:<GripID>:<ForceN>\r\n
```

Beispiel:

```text
POSE:4:1.0\r\n
```

Die Pose wird gesetzt und fuer Zeige-, Mittel-, Ring- und kleinen Finger wird
`1.0 N` als Kraftsollwert gespeichert.

## 6. Einzelne Positionsreferenz

### 6.1 Position ohne Kraftaenderung

```text
POS:<Axis>:<Percent>\r\n
```

`Axis` muss zwischen `0` und `5` liegen. `Percent` muss zwischen `0` und `100`
liegen und darf Nachkommastellen enthalten.

Beispiele:

```text
POS:0:25.5\r\n
POS:1:50\r\n
POS:5:75\r\n
```

### 6.2 Position mit Sollkraft

```text
POS:<Finger>:<Percent>:<ForceN>\r\n
```

Diese Variante ist nur fuer Finger `1..4` erlaubt. Daumenbeugung (`0`) und
Daumenrotation (`5`) bleiben rein positionsgeregelt.

Beispiel:

```text
POS:1:50:1.0\r\n
```

Bedeutung:

```text
Zeigefinger-Referenzposition: 50 %
Zeigefinger-Sollkraft:       1.0 N
```

Nach Erreichen der Referenzposition kann die Admittanz den Finger weiter
beugen oder in Streckrichtung nachgeben, wenn `ADM:ON` aktiv ist.

## 7. Kraftsollwerte

### 7.1 Kraft fuer alle geregelten Finger

```text
FORCE:ALL:<ForceN>\r\n
```

Beispiel:

```text
FORCE:ALL:1.0\r\n
```

Der Wert wird auf Finger `1`, `2`, `3` und `4` angewendet.

### 7.2 Kraft fuer einen Finger

```text
FORCE:<Finger>:<ForceN>\r\n
```

Beispiele:

```text
FORCE:1:1.0\r\n
FORCE:2:0.5\r\n
FORCE:3:1.2\r\n
FORCE:4:0.8\r\n
```

## 8. Admittanz ein-/ausschalten

```text
ADM:ON\r\n
ADM:OFF\r\n
```

`ADM:ON` wird nur akzeptiert, wenn der FSR-Nullpunkt bereits erfasst wurde.
Nach dem Start erfolgt automatisch ein Tare. Andernfalls zuerst senden:

```text
FSR:TARE\r\n
```

`ADM:OFF` deaktiviert die Admittanz und verwirft die aktuelle Admittanz-
Auslenkung. Der Controller verwendet danach wieder die Positionsreferenz.

### 8.1 Regelgesetz

Fuer Finger 1 bis 4 wird ein eigener diskreter Regler verwendet:

```text
M * q_ddot + D * q_dot + K * min(q - q_ref, 0) = F_soll - F_ist
```

Die Bedeutung ist:

- `q_ref` ist die Positionsreferenz aus `POSE` oder `POS`.
- Bei `F_ist < F_soll` wird weiter gebeugt.
- Bei `F_ist > F_soll` wird in Streckrichtung nachgegeben.
- Die virtuelle Feder wirkt nur unterhalb von `q_ref`.
- Der Daempfer erhoeht die Gegenwirkung bei schneller Auslenkung.
- `F_soll=0` erzeugt passive Nachgiebigkeit.
- Daumen und Daumenrotation werden nicht durch den FSR-Regler beeinflusst.

Aktuelle Parameter:

```text
Kraft-Totzone:          0.05 N
Virtuelle Steifigkeit:  0.1 N/%
Eigenfrequenz:           3 Hz
Reglertakt:              500 Hz
Maximale Sollkraft:      5 N
```

## 9. Sensor-Tare

```text
FSR:TARE\r\n
```

Der Tare speichert die aktuelle unbelastete Leitfaehigkeit jedes FSR als
Nullpunkt. Der Befehl wird abgewiesen, wenn eine Bewegung oder aktive Admittanz
laeuft.

Beim Firmwarestart werden automatisch 250 ADC-Sequenzen fuer den Start-Tare
gesammelt. Bei 500 Hz entspricht das etwa 500 ms.

Tare korrigiert nur den Nullpunkt. Fuer absolute Newtonwerte ist weiterhin eine
reale Sensor-Kalibrierung erforderlich.

## 10. Geschwindigkeit

```text
SPEED:<DegreesPerSecond>\r\n
```

Gueltiger Bereich:

```text
1 ... 270 deg/s
```

Startwert:

```text
200 deg/s
```

Beispiel:

```text
SPEED:100\r\n
```

Die Geschwindigkeit begrenzt sowohl normale Positionsfahrten als auch die
maximale virtuelle Admittanzgeschwindigkeit.

## 11. Globales Servo-Torque-Limit

```text
TORQUE:<Percent>\r\n
```

Gueltiger Bereich:

```text
0 ... 100 %
```

Startwert:

```text
50 %
```

Beispiel:

```text
TORQUE:50\r\n
```

Der Wert wird nacheinander auf alle sechs STS3215-Servos geschrieben. Das
Limit liegt im Servo-SRAM und muss nach einem Neustart erneut gesetzt werden.
Das Torque-Limit ersetzt keine mechanischen Endanschlaege oder einen Not-Aus.

## 12. Stop und Hold

```text
STOP\r\n
HOLD\r\n
```

Beide Befehle beenden laufende Referenztrajektorien und halten den zuletzt
ausgegebenen Positionssollwert.

`STOP` schaltet zusaetzlich die Admittanz aus. `HOLD` verwendet derzeit denselben
sicheren Haltezustand.

## 13. Statusabfrage

### 13.1 Einmalige Abfrage

```text
STATUS?\r\n
```

Die Firmware sendet eine `STAT:`-Zeile und danach `OK`.

### 13.2 Periodischer Statusstream

```text
STATUS:STREAM:<Hz>\r\n
```

Gueltiger Bereich:

```text
0 ... 20 Hz
```

Stream ausschalten:

```text
STATUS:STREAM:0\r\n
```

Stream mit 10 Hz einschalten:

```text
STATUS:STREAM:10\r\n
```

Nach dem Stream-Kommando wird einmal `OK` bestaetigt. Danach werden nur noch
periodische `STAT:`-Zeilen gesendet.

## 14. Statusformat

Die Statuszeile besitzt dieses Format:

```text
STAT:<seq>:<mode>:<faultHex>:R:<r0,..,r5>:C:<c0,..,c5>:P:<p0,..,p5>:PT:<t0,..,t5>:S:<s1,..,s4>:F:<f0,..,f4>:A:<a0,..,a4>:I:<i0,..,i5>:SP:<speed>:TQ:<torque>
```

Beispielstruktur:

```text
STAT:125:ADM:00000000:R:50.0,0.0,0.0,0.0,0.0,0.0:C:50.0,0.0,0.0,0.0,0.0,0.0:P:49.8,0.1,0.0,0.0,0.0,0.0:PT:3069,2050,2047,2047,2047,2047:S:0.00,1.00,1.00,1.00:F:0.00,1.01,0.98,0.00,0.00:A:210,720,700,200,190:I:120,340,320,110,100,90:SP:200:TQ:50
```

Feldbedeutung:

| Feld | Inhalt |
|---|---|
| `seq` | Laufende Controller-Sequenznummer |
| `mode` | `BOOT`, `MOVE`, `POS`, `ADM`, `HOLD` oder `FAULT` |
| `faultHex` | Fehlerbitmaske als Hexadezimalwert |
| `R` | Positionsreferenzen fuer Achsen 0 bis 5 in Prozent |
| `C` | ausgegebene Positionssollwerte fuer Achsen 0 bis 5 in Prozent |
| `P` | Servo-Istpositionen fuer Achsen 0 bis 5 in Prozent |
| `PT` | Servo-Istpositionen fuer Achsen 0 bis 5 in Ticks |
| `S` | Kraftsollwerte fuer Finger 1 bis 4 in Newton |
| `F` | FSR-Kraftnaeherungen fuer FSR 0 bis 4 in Newton |
| `A` | ADC-Rohwerte fuer FSR 0 bis 4 |
| `I` | Servo-Iststroeme fuer Achsen 0 bis 5 in mA |
| `SP` | globale Geschwindigkeit in Grad/s |
| `TQ` | globales Torque-Limit in Prozent |

Die Werte in `P` und `PT` stammen aus Servo-Readback. Sie sind nicht nur die
zuletzt gesendeten Sollwerte. Die Werte in `F` sind wegen der fehlenden realen
FSR-Kalibrierung nur Naeherungen.

## 15. Empfohlener Ablauf

```text
1. Firmware starten und 500 ms fuer den automatischen Tare warten.
2. FSR:TARE senden, wenn die Hand sicher unbelastet ist.
3. TORQUE:50 senden.
4. SPEED:100 senden fuer einen konservativen ersten Test.
5. POSE:0 senden und offene Position pruefen.
6. POS:1:20 senden und den Zeigefinger einzeln pruefen.
7. FORCE:1:0.5 senden.
8. ADM:ON senden.
9. POS:1:50:1.0 senden.
10. STATUS:STREAM:10 fuer Diagnose einschalten.
11. ADM:OFF oder STOP zum sicheren Beenden senden.
```

## 16. Senderregeln

1. Nach jedem gesendeten Kommando auf `OK` oder `ERR:*` warten.
2. `STAT:`-Zeilen koennen zwischen Antworten eintreffen und muessen separat verarbeitet werden.
3. Nur ASCII und Dezimalpunkt verwenden, zum Beispiel `1.0`.
4. Keine alten Befehle mit `Side`-Parameter senden.
5. Positionen immer im Bereich `0..100 %` senden.
6. Kraftsollwerte immer im Bereich `0..5 N` senden.
7. Eine einzelne Kommandozeile darf maximal 127 Zeichen lang sein.
8. Bei `ERR:QUEUE_FULL` die Uebertragung kurz anhalten und das Kommando erneut senden.
9. Bei Krafttests immer zuerst den Statusstream einschalten und Fehlerflags auswerten.
