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
 - **setTargetGrip:** Setzt Zielgriff sofort und startet eine sanfte Trajektorie zum Ziel (Dauer in ms wird übernommen).
 - **Interpolation:** Zwischenpositionen werden via Smoothstep (S‑Kurve) berechnet für gleichmäßige Bewegung.
 - **SyncWrite:** Positionsbefehle werden mit `syncWritePositions` an alle Finger gesendet (non-blocking).
 - **Telemetrie (Round‑Robin):** Bus wird mit `bus.poll()` getaktet; `startReadCurrent` initiiert RX-before-TX; bei `DATA_READY` wird das Ergebnis verarbeitet und zum nächsten Finger weitergerückt.
 - **Predict-Hook:** `predictGraspAdjustment` dient als Hook für zukünftige AI‑Anpassungen (Slip/Force).
 - **Scope:** Aktuell wird nur die rechte Hand regelmäßig upgedatet (`leftHand.update()` auskommentiert).

 ## 🧠 Edge-AI Integration

 Dank der Cortex-M55 Architektur und der dedizierten NPU auf dem N6-Chip können komplexe Modelle zur Slip-Detection (Rutsch-Erkennung) oder taktilen Rückmeldung implementiert werden. Die Funktion `predictGraspAdjustment` im `HandController` dient als dedizierter Hook für X-CUBE-AI generierten Code.
