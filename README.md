# PIB Hand — STM32N6 Firmware

Firmware für die Steuerung von zwei 6-DOF Roboterhänden auf dem **NUCLEO-N657X0-Q** (STM32N657X0, Cortex-M55 @ 800 MHz).


## Hardware

| Komponente | Details |
|---|---|
| MCU | STM32N657X0 (Cortex-M55 @ 800 MHz) |
| Servos | Waveshare/Feetech **STS3215** Serial Bus Servos |
| Servo-Bus | USART3 — 1 Mbaud, Half-Duplex DMA |
| VCP (ROS 2) | LPUART1 — 460800 Baud |
| Encoder | 3× AS5600 (12-bit magnetisch) via TCA9548A I2C-Multiplexer |
| Encoder-Bus | I2C4 — 400 kHz, DMA (GPDMA1 Ch4/Ch5) |

### Encoder-Verdrahtung (TCA9548A @ 0x70)

| TCA-Kanal | Encoder | Gelenk |
|---|---|---|
| 0 | AS5600 #0 | Base |
| 1 | AS5600 #1 | Middle |
| 2 | AS5600 #2 | Tip |

Jeder AS5600 (3.3V-Betrieb): VDD5V + VDD3V3 zusammen auf 3.3V, 100nF Bypass-Cap, DIR → GND (CW) oder 3.3V (CCW).  
TCA9548A: A0/A1/A2 → GND, RESET → 3.3V (darf nicht floaten).  
Pull-ups: 4.7 kΩ auf dem Haupt-I2C-Bus (PE13/PE14) und je 4.7 kΩ auf jedem aktiven TCA-Kanal.

## Software-Architektur

| Datei | Funktion |
|---|---|
| `main.c` | Init, 100-Hz-Hauptloop |
| `hand_config.hpp` | Servo-IDs, Griff-Positionen, Achsen-Limits |
| `hand_controller.cpp` | Smoothstep-Trajektorie, SyncWrite, Bus-Polling |
| `serial_commander.cpp` | Ringpuffer-Parser für ASCII-Kommandos |
| `servo.cpp` | STS3215-Protokoll, DMA-basierter ServoBus |
| `as5600.cpp` | AS5600 + TCA9548A non-blocking DMA-Treiber |

**Loop:** `as5600_update()` und `hand_bridge_update()` werden im Hauptloop aufgerufen. Hand-Controller: 100 Hz. Encoder Round-Robin: ~30 ms pro Encoder (10 ms Poll-Periode, 3 Encoder).

## Serielles Protokoll (VCP, 460800 Baud)

**Side:** `0` = links, `1` = rechts

| Befehl | Beschreibung | Beispiel |
|---|---|---|
| `G:<Side>:<GripID>` | Griff setzen (Standardgeschwindigkeit) | `G:1:0` |
| `G:<Side>:<GripID>:V:<pct>` | Griff mit globaler Geschwindigkeit (0–100 %) | `G:0:2:V:50` |
| `G:<Side>:<GripID>:Vx:<v0>,...,<v5>` | Griff mit Per-Finger-Geschwindigkeit (0–100 %) | `G:1:3:Vx:50,60,70,80,90,100` |
| `F:<Side>:<Finger>:<Pos>[:<Speed>]` | Einzelnen Finger positionieren (Pos 0–4095, Speed °/s) | `F:0:2:3000:120` |
| `STOP:<Side>` | Alle Bewegungen sofort stoppen | `STOP:1` |
| `HOLD:<Side>` | Aktuelle Position halten | `HOLD:0` |
| `GET:STATUS` | Statusausgabe via VCP | `GET:STATUS` |

**Antworten:** `OK` bei Erfolg — `ERR SYNTAX` / `ERR GRIPID` / `ERR SPEED` / `ERR POS` / `ERR NOEXEC` / `ERR EXEC` bei Fehler.

## Verfügbare Griffe

Positionen in nativen Servo-Einheiten (0–4095). Reihenfolge: Thumb, Index, Middle, Ring, Pinky, ThumbRotation.

| ID | Name | Thumb | Index | Middle | Ring | Pinky | ThumbRot |
|---|---|---|---|---|---|---|---|
| 0 | OPEN | 0 | 0 | 0 | 0 | 0 | 0 |
| 1 | ZEIGEN | 4095 | 0 | 4095 | 4095 | 4095 | 2047 |
| 2 | DREIPUNKTGRIFF | 3185 | 3185 | 3185 | 0 | 0 | 2047 |
| 3 | SCHLUESSELGRIFF | 2730 | 1365 | 0 | 0 | 0 | 2730 |
| 4 | ZYLINDERGRIFF | 3640 | 3640 | 3640 | 3640 | 3640 | 1365 |
| 5 | HAKENGRIFF | 0 | 3640 | 3640 | 3640 | 3640 | 0 |
| 6 | SPHAERISCHER_GRIFF | 2730 | 2730 | 2730 | 2730 | 2730 | 1820 |
| 7 | MITTELFINGER | 4095 | 4095 | 0 | 4095 | 4095 | 2000 |
| 8 | ROCKS | 0 | 0 | 4095 | 4095 | 0 | 2047 |

## Flashen

Voraussetzungen: STM32CubeIDE + STM32CubeProgrammer im Standardpfad installiert, beide Projekte (`FSBL` und `Appli`) gebaut.

```powershell
# Im Workspace-Root ausführen:
.\scripts\sign_and_deploy.ps1
```

Das Script führt folgende Schritte aus:
1. **Signiert** `FSBL\Debug\*_FSBL.bin` mit dem STM32 SigningTool (Header v2.3, Zieladresse `0x80000000`)
2. **Signiert** `Appli\Debug\*_Appli.bin` (Zieladresse `0x34000000`)
3. **Flasht** FSBL-Image auf externen Flash → `0x70000000`
4. **Flasht** Appli-Image auf externen Flash → `0x70100000` (via External Loader `MX25UM51245G`)
5. **Reset** des Boards via SWD

Nach dem Flashen ggf. manuell RESET drücken, falls das Board nicht automatisch startet.
