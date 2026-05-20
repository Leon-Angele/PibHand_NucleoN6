 # PIB Hand Control - STM32N6 Intelligence Edition

 Diese Firmware ermöglicht die Steuerung von zwei Roboterhänden (jeweils 6 Freiheitsgrade) auf Basis des **NUCLEO-N657X0-Q** (STM32N6-Serie). Die Architektur ist auf geringe Latenz, organische Bewegungen und die zukünftige Integration von Edge-AI optimiert.

 ## 🚀 Kern-Features

 * **100Hz Real-Time Control Loop**: Aktualisierung aller Fingerpositionen alle 10ms für flüssige Bewegungen.

 	Hinweis: Im aktuellen Code wird der Hand-Controller in `main.c` alle 10 ms aufgerufen (100 Hz). In der Bridge-Implementierung ist aktuell nur die rechte Hand aktiv — `leftHand.update()` ist in `Appli/Core/Src/hand/hand_bridge.cpp` auskommentiert.
 * **Smooth Trajectories**: Ruckelfreie Beschleunigung und Abbremsung durch S-Kurven-Interpolation (**Smoothstep**).
 * **ROS 2 Interface**: ASCII-basiertes Protokoll für die einfache Integration in ROS 2-Systeme über USB/Seriell.
 * **Adaptive Grasping (AI Placeholder)**: Integrierte Schnittstelle für den **ST Neural-ART Accelerator**, um neuronale Netze zur Griffoptimierung direkt auf der Hardware auszuführen.
 * **DMA-Optimierung**: Non-blocking Kommunikation mit den Servos über den GPDMA-Controller des STM32N6.
 * **Sync Write**: Zeitgleicher Start und Stopp aller Finger einer Hand durch optimierte Bus-Pakete (Instruction 0x83).

 ## 🛠 Hardware-Konfiguration

 * **MCU**: STM32N657X0 (Cortex-M55 @ 800 MHz).
 * **Beschleuniger**: Integrierte NPU (Neural-ART) für Deep Learning Tasks.
 * **Servos**: Waveshare / Feetech **STS3215** Serial Bus Servos.
 * **Bus**: USART3 mit 1.000.000 Baud (1 Mbps).

 ## 📡 ROS 2 Protokoll

 Der `SerialCommander` verarbeitet Befehle im folgenden Format:

 **BAUD:** `460800`
 **Syntax:** `G:<Side>:<GripID>\n`

 * **Side**: `0` für die linke Hand, `1` für die rechte Hand.
 * **GripID**: Ganzzahliger Index des gewünschten Griffs aus der Konfigurations-Datenbank.

 **Beispiele:**
 * `G:1:0\n` -> Rechte Hand öffnen (OPEN).
 * `G:0:4\n` -> Linke Hand schließt zum Zylindergriff.

 ## 🖐 Verfügbare Griffe

 Die Griff-Positionen sind in `hand_config.hpp` als native Servo‑Einheiten (0–4095) definiert. Die hier gezeigten Werte entsprechen direkt den Einträgen in der `GripDatabase`:

 | ID | Name | Finger-Konfiguration (0–4095) |
 | :--- | :--- | :--- |
 | 0 | **OPEN** | {0, 0, 0, 0, 0, 0} |
 | 1 | **SPITZGRIFF** | {4095, 4095, 4095, 4095, 4095, 4095} |
 | 2 | **DREIPUNKTGRIFF** | {3185, 3185, 3185, 0, 0, 2047} |
 | 3 | **SCHLUESSELGRIFF** | {2730, 1365, 0, 0, 0, 2730} |
 | 4 | **ZYLINDERGRIFF** | {3640, 3640, 3640, 3640, 3640, 1365} |
 | 5 | **HAKENGRIFF** | {0, 3640, 3640, 3640, 3640, 0} |
 | 6 | **SPHAERISCHER GRIFF** | {2730, 2730, 2730, 2730, 2730, 1820} |
 | 7 | **Stinkefinger** | {4095, 4095, 0, 4095, 4095, 2000} |
 

 ## 📂 Software-Architektur

 * `main.c`: Systemstart, Initialisierung der High-End Peripherie (CACHEAXI, RIF) und 100Hz Loop-Taktung.
 * `hand_config.hpp`: Typ-sichere Enums für Finger und Griffe sowie Hardware-Limits.
 * `hand_controller.cpp`: Berechnung der Zwischenpositionen und Telemetrie-Abfrage der Servos.
 * `serial_commander.cpp`: Ringpuffer-basierter Parser für eintreffende USB-Befehle.
 * `servo.cpp`: Low-Level DMA-Treiber für das STS/SCS-Protokoll.

 ### HandController-Logik
 - **Taktung:** Läuft mit 100 Hz (je 10 ms Zyklus).
 - **setTargetGrip:** Setzt Zielgriff sofort und startet eine sanfte Trajektorie zum Ziel. Jeder Finger bewegt sich mit seiner individuellen Geschwindigkeit aus `AxisSettings` (in Grad/Sekunde, wobei 0-4095 Servo-Einheiten = 360°).
 - **Geschwindigkeitssteuerung:** Config-basiert (`maxSpeed` in °/s), jeder Finger berechnet seine Fahrtzeit automatisch: `duration = (Δ Position × 1000) / (maxSpeed × 4095/360)`. Finger kommen asynchron an.
 - **Interpolation:** Zwischenpositionen werden via Smoothstep (S‑Kurve) berechnet für gleichmäßige Bewegung mit sanftem Anfahren/Abbremsen.
 - **SyncWrite:** Positionsbefehle werden mit `syncWritePositions` an alle Finger gesendet (non-blocking), servo `time` Parameter konstant bei 10 ms für smoothe Ausführung.
 - **Telemetrie (Round‑Robin):** Bus wird mit `bus.poll()` getaktet; `startReadCurrent` initiiert RX-before-TX; bei `DATA_READY` wird das Ergebnis verarbeitet und zum nächsten Finger weitergerückt.
 - **Predict-Hook:** `predictGraspAdjustment` dient als Hook für zukünftige AI‑Anpassungen (Slip/Force).
 - **Scope:** Aktuell wird nur die rechte Hand regelmäßig upgedatet (`leftHand.update()` auskommentiert).

 ## 🧠 Edge-AI Integration

 Dank der Cortex-M55 Architektur und der dedizierten NPU auf dem N6-Chip können komplexe Modelle zur Slip-Detection (Rutsch-Erkennung) oder taktilen Rückmeldung implementiert werden. Die Funktion `predictGraspAdjustment` im `HandController` dient als dedizierter Hook für X-CUBE-AI generierten Code.

**Serial Commands**

- **Command:** `G:<Side>:<GripID>`
	- **Description:** Legacy-Aufruf zum Setzen eines vordefinierten Griffs. Nutzt die in `hand_config.hpp` konfigurierten `maxSpeed`-Werte.
	- **Example:** `G:1:0` — Rechte Hand öffnen (OPEN)

- **Command:** `G:<Side>:<GripID>:V:<percent>`
	- **Description:** Gleicher Griff, aber alle Finger bewegen sich mit `percent` (0–100) relativ zur konfigurierten `maxSpeed`.
	- **Example:** `G:0:2:V:50` — Linke Hand, Griff 2, 50% der Max-Geschwindigkeit

- **Command:** `G:<Side>:<GripID>:Vx:<v0>,...,<v5>`
	- **Description:** Per-Finger-Prozentwerte (je 0–100). Reihenfolge: Thumb, Index, Middle, Ring, Pinky, ThumbRotation.
	- **Example:** `G:1:3:Vx:50,60,70,80,90,100`

- **Command:** `F:<Side>:<Finger>:<Pos>[:<Speed>]`
	- **Description:** Setzt einen einzelnen Finger (`Finger` Index 0..5) auf Position `Pos` (0..4095). Optionaler `Speed` in °/s; wenn weggelassen, wird `maxSpeed` aus `hand_config.hpp` verwendet.
	- **Example:** `F:0:2:3000` — Linke Hand, Middle auf 3000 mit Standardgeschwindigkeit
	- **Example:** `F:0:2:3000:120` — Linke Hand, Middle auf 3000 mit 120 °/s

- **Command:** `STOP:<Side>` / `HOLD:<Side>`
	- **Description:** `STOP` bricht alle laufenden Trajektorien ab und hält die Servos in ihrer aktuellen Position mittels Sync-Write. `HOLD` verhält sich gleich (Reserviert für spätere Unterscheidung).
	- **Example:** `STOP:0` — Stoppe/halte linke Hand sofort

- **Command:** `GET:STATUS`
	- **Description:** Liefert einen kompakten Statusreport (Bus- und Controller-Status). Ausgabe erfolgt via VCP.
	- **Example:** `GET:STATUS`

**Fehlerantworten & Limits**

- `ERR SYNTAX` — Allgemeiner Syntaxfehler oder unvollständiges Kommando.
- `ERR GRIPID` — Ungültige Grip-ID (außerhalb der definierten `GripDatabase`).
- `ERR SPEED` — Ungültiger Prozent- oder Speedwert (z.B. >100% oder negative Werte).
- `ERR POS` — Ungültige Position (außerhalb 0..4095).
- `ERR NOEXEC` / `ERR EXEC` — Kein Executor registriert oder Ausführungsfehler.
- `OK` — Erfolg.

Hinweis: Alle Befehle sind abwärtskompatibel; das ursprüngliche `G:<Side>:<GripID>` Verhalten bleibt unverändert.
