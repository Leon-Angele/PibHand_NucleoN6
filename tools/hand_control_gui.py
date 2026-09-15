"""Windows GUI for the PIB hand VCP ASCII protocol."""

from __future__ import annotations

import math
import queue
import threading
import time
import tkinter as tk
from collections.abc import Callable
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Keep protocol helpers importable for unit tests.
    serial = None
    list_ports = None


BAUD_RATE = 460800
AXIS_NAMES = (
    "Daumenbeugung",
    "Zeigefinger",
    "Mittelfinger",
    "Ringfinger",
    "Kleiner Finger",
    "Daumenrotation",
)
POSE_NAMES = (
    "Open",
    "Spitzgriff / Zeigen",
    "Dreipunktgriff",
    "Schluesselgriff",
    "Zylindergriff",
    "Hakengriff",
    "Sphaerischer Griff",
    "Mittelfinger",
)
STATUS_MODES = {"BOOT", "TARE", "MOVE", "POS", "ADM", "HOLD", "FAULT"}


def _number(value: float, minimum: float, maximum: float, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} muss eine Zahl sein.") from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{name} muss zwischen {minimum:g} und {maximum:g} liegen.")
    return result


def _integer(value: int, minimum: int, maximum: int, name: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} muss eine ganze Zahl sein.") from exc
    if isinstance(value, float) and not value.is_integer():
        raise ValueError(f"{name} muss eine ganze Zahl sein.")
    if not minimum <= result <= maximum:
        raise ValueError(f"{name} muss zwischen {minimum} und {maximum} liegen.")
    return result


def _format_number(value: float) -> str:
    return f"{value:.3f}".rstrip("0").rstrip(".")


def position_command(axis: int, percent: float, force: float | None = None) -> str:
    axis = _integer(axis, 0, 5, "Achse")
    percent = _number(percent, 0.0, 100.0, "Position")
    command = f"POS:{axis}:{_format_number(percent)}"
    if force is not None:
        if axis not in range(1, 5):
            raise ValueError("Ein POS-Kraftwert ist nur fuer Finger 1 bis 4 erlaubt.")
        command += f":{_format_number(_number(force, 0.0, 5.0, 'Kraft'))}"
    return command


def pose_command(pose: int, force: float | None = None) -> str:
    command = f"POSE:{_integer(pose, 0, 7, 'Pose')}"
    if force is not None:
        command += f":{_format_number(_number(force, 0.0, 5.0, 'Kraft'))}"
    return command


def force_command(force: float, finger: int | None = None) -> str:
    target = "ALL" if finger is None else str(_integer(finger, 1, 4, "Finger"))
    return f"FORCE:{target}:{_format_number(_number(force, 0.0, 5.0, 'Kraft'))}"


def speed_command(speed: int) -> str:
    return f"SPEED:{_integer(speed, 1, 270, 'Geschwindigkeit')}"


def torque_command(percent: int) -> str:
    return f"TORQUE:{_integer(percent, 0, 100, 'Torque-Limit')}"


def stream_command(rate_hz: int) -> str:
    return f"STATUS:STREAM:{_integer(rate_hz, 0, 20, 'Statusrate')}"


@dataclass(frozen=True)
class HandStatus:
    sequence: int
    mode: str
    faults: int
    reference: tuple[float, ...]
    command: tuple[float, ...]
    actual: tuple[float, ...]
    ticks: tuple[int, ...]
    force_setpoint: tuple[float, ...]
    force_measured: tuple[float, ...]
    adc_raw: tuple[int, ...]
    current_ma: tuple[int, ...]
    speed: int
    torque: int


def _csv(values: str, count: int, convert: Callable[[str], float | int], name: str):
    parts = values.split(",")
    if len(parts) != count:
        raise ValueError(
            f"Statusfeld {name} erwartet {count} Werte, erhielt {len(parts)}."
        )
    try:
        converted = tuple(convert(part) for part in parts)
    except ValueError as exc:
        raise ValueError(
            f"Statusfeld {name} enthaelt einen ungueltigen Zahlenwert."
        ) from exc
    if convert is float and not all(math.isfinite(value) for value in converted):
        raise ValueError(f"Statusfeld {name} enthaelt keinen endlichen Wert.")
    return converted


def parse_status(line: str) -> HandStatus:
    parts = line.strip().split(":")
    if len(parts) < 6 or parts[0] != "STAT":
        raise ValueError("Keine gueltige STAT-Zeile.")
    try:
        sequence = int(parts[1], 10)
        mode = parts[2]
        faults = int(parts[3], 16)
    except ValueError as exc:
        raise ValueError("Ungueltiger STAT-Kopf.") from exc
    if mode not in STATUS_MODES:
        raise ValueError(f"Unbekannter Controller-Modus: {mode}")
    if (len(parts) - 4) % 2:
        raise ValueError("STAT-Felder sind nicht vollstaendig.")
    fields = dict(zip(parts[4::2], parts[5::2]))
    required = {"R", "C", "P", "PT", "S", "F", "A", "I", "SP", "TQ"}
    if set(fields) != required:
        missing = ", ".join(sorted(required - set(fields))) or "keine"
        extra = ", ".join(sorted(set(fields) - required)) or "keine"
        raise ValueError(f"Ungueltige STAT-Felder (fehlen: {missing}; extra: {extra}).")
    try:
        speed = int(fields["SP"], 10)
        torque = int(fields["TQ"], 10)
    except ValueError as exc:
        raise ValueError("SP oder TQ ist kein Ganzzahlwert.") from exc
    return HandStatus(
        sequence=sequence,
        mode=mode,
        faults=faults,
        reference=_csv(fields["R"], 6, float, "R"),
        command=_csv(fields["C"], 6, float, "C"),
        actual=_csv(fields["P"], 6, float, "P"),
        ticks=_csv(fields["PT"], 6, int, "PT"),
        force_setpoint=_csv(fields["S"], 4, float, "S"),
        force_measured=_csv(fields["F"], 5, float, "F"),
        adc_raw=_csv(fields["A"], 5, int, "A"),
        current_ma=_csv(fields["I"], 6, int, "I"),
        speed=speed,
        torque=torque,
    )


class SerialWorker:
    """Owns the serial port and serializes commands with their responses."""

    _STOP = object()

    def __init__(self, port: str, events: queue.Queue[tuple[str, object]]) -> None:
        self.port = port
        self.events = events
        self.commands: queue.Queue[object] = queue.Queue()
        self.stop_now = threading.Event()
        self.thread = threading.Thread(target=self._run, name="hand-vcp", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def send(self, command: str) -> None:
        if not self.stop_now.is_set():
            self.commands.put(command)

    def close(self, graceful: bool = True) -> None:
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        if graceful:
            self.commands.put("STATUS:STREAM:0")
            self.commands.put(self._STOP)
        else:
            self.stop_now.set()

    def _run(self) -> None:
        if serial is None:
            self.events.put(
                ("error", "pyserial fehlt. Bitte tools/requirements.txt installieren.")
            )
            return
        connection = None
        pending: str | None = None
        deadline = 0.0
        retry_count = 0
        try:
            connection = serial.Serial(
                self.port,
                BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=1.0,
            )
            self.events.put(("connected", self.port))
            while not self.stop_now.is_set():
                if pending is None:
                    try:
                        item = self.commands.get_nowait()
                    except queue.Empty:
                        item = None
                    if item is self._STOP:
                        break
                    if isinstance(item, str):
                        pending = item
                        retry_count = 0
                        connection.write((item + "\r\n").encode("ascii"))
                        self.events.put(("tx", item))
                        deadline = time.monotonic() + 2.0

                raw = connection.readline()
                if raw:
                    line = raw.decode("ascii", errors="replace").strip("\r\n")
                    if line.startswith("STAT:"):
                        try:
                            self.events.put(("status", parse_status(line)))
                        except ValueError as exc:
                            self.events.put(("log", f"Ungueltiger Status: {exc}"))
                    elif line == "OK" or line.startswith("ERR:"):
                        if pending is None:
                            self.events.put(("log", f"Unzugeordnete Antwort: {line}"))
                        elif line == "ERR:QUEUE_FULL" and retry_count == 0:
                            retry_count = 1
                            self.events.put(
                                ("log", f"Queue voll, wiederhole: {pending}")
                            )
                            time.sleep(0.1)
                            connection.write((pending + "\r\n").encode("ascii"))
                            self.events.put(("tx", pending))
                            deadline = time.monotonic() + 2.0
                        else:
                            self.events.put(("response", (pending, line)))
                            pending = None
                    elif line:
                        self.events.put(("rx", line))

                if pending is not None and time.monotonic() >= deadline:
                    self.events.put(("response", (pending, "TIMEOUT")))
                    pending = None
        except (OSError, ValueError) as exc:
            self.events.put(("error", f"Serielle Verbindung fehlgeschlagen: {exc}"))
        finally:
            if connection is not None and connection.is_open:
                connection.close()
            self.events.put(("disconnected", self.port))


class HandControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PIB Hand VCP-Steuerung")
        self.root.geometry("1220x800")
        self.root.minsize(980, 680)
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.worker: SerialWorker | None = None
        self.connected = False
        self.closing = False
        self.command_widgets: list[tk.Widget] = []
        self.port_by_label: dict[str, str] = {}
        self.sliders_initialized = False
        self.slider_vars = [tk.DoubleVar(value=0.0) for _ in AXIS_NAMES]
        self.slider_labels = [tk.StringVar(value="0.0 %") for _ in AXIS_NAMES]
        self.pos_force_enabled = [tk.BooleanVar(value=False) for _ in AXIS_NAMES]
        self.pos_force_values = [tk.StringVar(value="1.0") for _ in AXIS_NAMES]
        self._build_ui()
        self._refresh_ports()
        self._set_controls_enabled(False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._poll_events)

    def _build_ui(self) -> None:
        style = ttk.Style()
        style.configure(
            "Emergency.TButton", foreground="#a40000", font=("Segoe UI", 10, "bold")
        )

        connection = ttk.Frame(self.root, padding=10)
        connection.pack(fill=tk.X)
        ttk.Label(connection, text="VCP-Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=42)
        self.port_combo.pack(side=tk.LEFT, padx=(6, 5))
        ttk.Button(connection, text="Aktualisieren", command=self._refresh_ports).pack(
            side=tk.LEFT
        )
        self.connect_button = ttk.Button(
            connection, text="Verbinden", command=self._toggle_connection
        )
        self.connect_button.pack(side=tk.LEFT, padx=8)
        self.connection_var = tk.StringVar(value="Getrennt")
        ttk.Label(connection, textvariable=self.connection_var).pack(
            side=tk.LEFT, padx=8
        )

        safety = ttk.Frame(connection)
        safety.pack(side=tk.RIGHT)
        self.open_button = self._command_button(
            safety, "Hand oeffnen", lambda: self._send("POSE:0")
        )
        self.open_button.pack(side=tk.LEFT, padx=3)
        self.hold_button = self._command_button(
            safety, "HOLD", lambda: self._send("HOLD")
        )
        self.hold_button.pack(side=tk.LEFT, padx=3)
        self.stop_button = self._command_button(
            safety, "STOP", lambda: self._send("STOP"), style="Emergency.TButton"
        )
        self.stop_button.pack(side=tk.LEFT, padx=3)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        position_tab = ttk.Frame(notebook, padding=12)
        control_tab = ttk.Frame(notebook, padding=12)
        status_tab = ttk.Frame(notebook, padding=12)
        notebook.add(position_tab, text="Positionen und Posen")
        notebook.add(control_tab, text="Kraft und Einstellungen")
        notebook.add(status_tab, text="Status und Log")
        self._build_position_tab(position_tab)
        self._build_control_tab(control_tab)
        self._build_status_tab(status_tab)

    def _command_button(self, parent, text: str, command, **kwargs) -> ttk.Button:
        button = ttk.Button(parent, text=text, command=command, **kwargs)
        self.command_widgets.append(button)
        return button

    def _build_position_tab(self, parent: ttk.Frame) -> None:
        sliders = ttk.LabelFrame(
            parent, text="Einzelachsen - Senden beim Loslassen", padding=10
        )
        sliders.pack(fill=tk.X)
        ttk.Label(sliders, text="Achse", width=20).grid(row=0, column=0, sticky=tk.W)
        ttk.Label(sliders, text="Zielposition").grid(row=0, column=1)
        ttk.Label(sliders, text="Wert", width=10).grid(row=0, column=2)
        ttk.Label(sliders, text="Kraft bei POS (nur 1..4)").grid(
            row=0, column=3, columnspan=2
        )
        sliders.columnconfigure(1, weight=1)
        for axis, name in enumerate(AXIS_NAMES):
            ttk.Label(sliders, text=f"{axis}: {name}").grid(
                row=axis + 1, column=0, sticky=tk.W, pady=5
            )
            scale = ttk.Scale(sliders, from_=0, to=100, variable=self.slider_vars[axis])
            scale.grid(row=axis + 1, column=1, sticky=tk.EW, padx=8)
            scale.configure(
                command=lambda value, i=axis: self.slider_labels[i].set(
                    f"{float(value):.1f} %"
                )
            )
            scale.bind(
                "<ButtonRelease-1>", lambda _event, i=axis: self._send_position(i)
            )
            scale.bind("<KeyRelease>", lambda _event, i=axis: self._send_position(i))
            self.command_widgets.append(scale)
            ttk.Label(sliders, textvariable=self.slider_labels[axis], width=10).grid(
                row=axis + 1, column=2
            )
            if axis in range(1, 5):
                check = ttk.Checkbutton(
                    sliders, text="mit Kraft", variable=self.pos_force_enabled[axis]
                )
                check.grid(row=axis + 1, column=3, padx=5)
                spin = ttk.Spinbox(
                    sliders,
                    from_=0.0,
                    to=5.0,
                    increment=0.1,
                    textvariable=self.pos_force_values[axis],
                    width=7,
                )
                spin.grid(row=axis + 1, column=4, padx=5)
                ttk.Label(sliders, text="N").grid(row=axis + 1, column=5, sticky=tk.W)
                self.command_widgets.extend((check, spin))

        poses = ttk.LabelFrame(parent, text="Posen", padding=10)
        poses.pack(fill=tk.X, pady=(12, 0))
        self.pose_var = tk.StringVar(value=f"0 - {POSE_NAMES[0]}")
        ttk.Combobox(
            poses,
            textvariable=self.pose_var,
            state="readonly",
            width=32,
            values=[f"{index} - {name}" for index, name in enumerate(POSE_NAMES)],
        ).grid(row=0, column=0, padx=(0, 8))
        self.pose_force_enabled = tk.BooleanVar(value=False)
        pose_force_check = ttk.Checkbutton(
            poses, text="Gemeinsame Kraft", variable=self.pose_force_enabled
        )
        pose_force_check.grid(row=0, column=1, padx=5)
        self.pose_force_var = tk.StringVar(value="1.0")
        pose_force = ttk.Spinbox(
            poses,
            from_=0.0,
            to=5.0,
            increment=0.1,
            textvariable=self.pose_force_var,
            width=7,
        )
        pose_force.grid(row=0, column=2, padx=5)
        ttk.Label(poses, text="N").grid(row=0, column=3)
        send = self._command_button(poses, "Pose anfahren", self._send_pose)
        send.grid(row=0, column=4, padx=12)
        self.command_widgets.extend((pose_force_check, pose_force))

    def _build_control_tab(self, parent: ttk.Frame) -> None:
        force_frame = ttk.LabelFrame(parent, text="Kraftsollwerte", padding=10)
        force_frame.pack(fill=tk.X)
        self.force_vars: dict[int | None, tk.StringVar] = {}
        entries = [(None, "Alle Finger 1..4")] + [
            (i, AXIS_NAMES[i]) for i in range(1, 5)
        ]
        for row, (finger, label) in enumerate(entries):
            ttk.Label(force_frame, text=label, width=24).grid(
                row=row, column=0, sticky=tk.W, pady=4
            )
            variable = tk.StringVar(value="1.0")
            self.force_vars[finger] = variable
            spin = ttk.Spinbox(
                force_frame,
                from_=0.0,
                to=5.0,
                increment=0.1,
                textvariable=variable,
                width=8,
            )
            spin.grid(row=row, column=1, padx=5)
            ttk.Label(force_frame, text="N").grid(row=row, column=2)
            button = self._command_button(
                force_frame, "Setzen", lambda f=finger: self._send_force(f)
            )
            button.grid(row=row, column=3, padx=10)
            self.command_widgets.append(spin)

        action_frame = ttk.LabelFrame(parent, text="Admittanz und Sensoren", padding=10)
        action_frame.pack(fill=tk.X, pady=(12, 0))
        for column, (text, command) in enumerate(
            (
                ("Admittanz EIN", "ADM:ON"),
                ("Admittanz AUS", "ADM:OFF"),
                ("FSR Tare", "FSR:TARE"),
            )
        ):
            self._command_button(
                action_frame, text, lambda cmd=command: self._send(cmd)
            ).grid(row=0, column=column, padx=5)

        settings = ttk.LabelFrame(parent, text="Servoeinstellungen", padding=10)
        settings.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(settings, text="Geschwindigkeit:").grid(row=0, column=0, sticky=tk.W)
        self.speed_var = tk.StringVar(value="200")
        speed = ttk.Spinbox(
            settings, from_=1, to=270, textvariable=self.speed_var, width=8
        )
        speed.grid(row=0, column=1, padx=5)
        ttk.Label(settings, text="deg/s").grid(row=0, column=2)
        self._command_button(settings, "Setzen", self._send_speed).grid(
            row=0, column=3, padx=10
        )
        ttk.Label(settings, text="Torque-Limit:").grid(
            row=1, column=0, sticky=tk.W, pady=8
        )
        self.torque_var = tk.StringVar(value="50")
        torque = ttk.Spinbox(
            settings, from_=0, to=100, textvariable=self.torque_var, width=8
        )
        torque.grid(row=1, column=1, padx=5)
        ttk.Label(settings, text="%").grid(row=1, column=2)
        self._command_button(settings, "Setzen", self._send_torque).grid(
            row=1, column=3, padx=10
        )
        self.command_widgets.extend((speed, torque))

        status_control = ttk.LabelFrame(parent, text="Statusabfrage", padding=10)
        status_control.pack(fill=tk.X, pady=(12, 0))
        self._command_button(
            status_control, "STATUS?", lambda: self._send("STATUS?")
        ).grid(row=0, column=0)
        ttk.Label(status_control, text="Streamrate:").grid(
            row=0, column=1, padx=(20, 5)
        )
        self.stream_var = tk.StringVar(value="10")
        stream = ttk.Spinbox(
            status_control, from_=0, to=20, textvariable=self.stream_var, width=7
        )
        stream.grid(row=0, column=2)
        ttk.Label(status_control, text="Hz").grid(row=0, column=3, padx=4)
        self._command_button(status_control, "Uebernehmen", self._send_stream).grid(
            row=0, column=4, padx=8
        )
        self.command_widgets.append(stream)

    def _build_status_tab(self, parent: ttk.Frame) -> None:
        summary = ttk.Frame(parent)
        summary.pack(fill=tk.X)
        self.sequence_var = tk.StringVar(value="-")
        self.mode_var = tk.StringVar(value="-")
        self.fault_var = tk.StringVar(value="-")
        self.status_speed_var = tk.StringVar(value="-")
        self.status_torque_var = tk.StringVar(value="-")
        for column, (label, variable) in enumerate(
            (
                ("Sequenz", self.sequence_var),
                ("Modus", self.mode_var),
                ("Faults", self.fault_var),
                ("Speed", self.status_speed_var),
                ("Torque", self.status_torque_var),
            )
        ):
            frame = ttk.LabelFrame(summary, text=label, padding=(12, 5))
            frame.grid(row=0, column=column, padx=(0, 8), sticky=tk.EW)
            summary.columnconfigure(column, weight=1)
            value_label = tk.Label(
                frame, textvariable=variable, font=("Segoe UI", 10, "bold")
            )
            value_label.pack()
            if label == "Faults":
                self.fault_label = value_label

        columns = (
            "axis",
            "ref",
            "cmd",
            "actual",
            "ticks",
            "setpoint",
            "force",
            "adc",
            "current",
        )
        self.status_tree = ttk.Treeview(
            parent, columns=columns, show="headings", height=7
        )
        headings = {
            "axis": "Achse",
            "ref": "Referenz %",
            "cmd": "Kommando %",
            "actual": "Ist %",
            "ticks": "Ist Ticks",
            "setpoint": "Soll N",
            "force": "Ist N",
            "adc": "ADC",
            "current": "Strom mA",
        }
        for name in columns:
            self.status_tree.heading(name, text=headings[name])
            self.status_tree.column(
                name, width=135 if name == "axis" else 90, anchor=tk.CENTER
            )
        self.status_tree.pack(fill=tk.X, pady=(10, 8))
        for axis, name in enumerate(AXIS_NAMES):
            self.status_tree.insert(
                "",
                tk.END,
                iid=str(axis),
                values=(name, "-", "-", "-", "-", "-", "-", "-", "-"),
            )

        ttk.Label(parent, text="Kommunikationslog").pack(anchor=tk.W, pady=(5, 3))
        self.log = scrolledtext.ScrolledText(
            parent, height=15, state=tk.DISABLED, font=("Consolas", 9)
        )
        self.log.pack(fill=tk.BOTH, expand=True)

    def _refresh_ports(self) -> None:
        if list_ports is None:
            self.connection_var.set("pyserial nicht installiert")
            self.port_combo["values"] = ()
            return
        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        self.port_by_label = {
            f"{item.device} - {item.description}": item.device for item in ports
        }
        labels = tuple(self.port_by_label)
        self.port_combo["values"] = labels
        if labels and self.port_var.get() not in labels:
            self.port_var.set(labels[0])

    def _toggle_connection(self) -> None:
        if self.connected or self.worker is not None:
            self._disconnect()
            return
        label = self.port_var.get().strip()
        port = self.port_by_label.get(label, label.split(" - ", 1)[0])
        if not port:
            messagebox.showerror("VCP-Port", "Bitte einen COM-Port auswaehlen.")
            return
        self.connection_var.set(f"Verbinde mit {port} ...")
        self.connect_button.configure(text="Abbrechen")
        self.worker = SerialWorker(port, self.events)
        self.worker.start()

    def _disconnect(self) -> None:
        if self.worker is not None:
            self.connection_var.set("Trenne Verbindung ...")
            self.worker.close(graceful=self.connected)
        self.connected = False
        self._set_controls_enabled(False)
        self.connect_button.configure(text="Verbinden")

    def _set_controls_enabled(self, enabled: bool) -> None:
        for widget in self.command_widgets:
            try:
                widget.state(["!disabled"] if enabled else ["disabled"])
            except tk.TclError:
                widget.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def _send(self, command: str) -> None:
        if not self.connected or self.worker is None:
            messagebox.showwarning(
                "Keine Verbindung", "Zuerst eine VCP-Verbindung herstellen."
            )
            return
        self.worker.send(command)

    def _checked_send(self, factory: Callable[[], str]) -> None:
        try:
            self._send(factory())
        except ValueError as exc:
            messagebox.showerror("Ungueltige Eingabe", str(exc))

    def _send_position(self, axis: int) -> None:
        force = (
            self.pos_force_values[axis].get()
            if self.pos_force_enabled[axis].get()
            else None
        )
        self._checked_send(
            lambda: position_command(axis, self.slider_vars[axis].get(), force)
        )

    def _send_pose(self) -> None:
        pose = int(self.pose_var.get().split(" ", 1)[0])
        force = self.pose_force_var.get() if self.pose_force_enabled.get() else None
        self._checked_send(lambda: pose_command(pose, force))

    def _send_force(self, finger: int | None) -> None:
        self._checked_send(lambda: force_command(self.force_vars[finger].get(), finger))

    def _send_speed(self) -> None:
        self._checked_send(lambda: speed_command(self.speed_var.get()))

    def _send_torque(self) -> None:
        self._checked_send(lambda: torque_command(self.torque_var.get()))

    def _send_stream(self) -> None:
        self._checked_send(lambda: stream_command(self.stream_var.get()))

    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "connected":
                    self.connected = True
                    self.sliders_initialized = False
                    self.connection_var.set(f"Verbunden: {payload} @ {BAUD_RATE} Baud")
                    self.connect_button.configure(text="Trennen")
                    self._set_controls_enabled(True)
                    self._send("STATUS:STREAM:10")
                elif kind == "disconnected":
                    self.connected = False
                    self.worker = None
                    self._set_controls_enabled(False)
                    self.connect_button.configure(text="Verbinden")
                    if not self.closing:
                        self.connection_var.set("Getrennt")
                elif kind == "error":
                    self.connection_var.set("Verbindungsfehler")
                    self._append_log("!", str(payload))
                    if not self.closing:
                        messagebox.showerror("Serielle Verbindung", str(payload))
                elif kind == "tx":
                    self._append_log(">", str(payload))
                elif kind == "rx":
                    self._append_log("<", str(payload))
                elif kind == "log":
                    self._append_log("!", str(payload))
                elif kind == "response":
                    command, response = payload
                    marker = "<" if response == "OK" else "!"
                    self._append_log(marker, f"{response} [{command}]")
                elif kind == "status":
                    self._update_status(payload)
        except queue.Empty:
            pass
        if not self.closing:
            self.root.after(50, self._poll_events)

    def _update_status(self, status: HandStatus) -> None:
        self.sequence_var.set(str(status.sequence))
        self.mode_var.set(status.mode)
        self.fault_var.set(f"0x{status.faults:08X}")
        self.status_speed_var.set(f"{status.speed} deg/s")
        self.status_torque_var.set(f"{status.torque} %")
        self.fault_label.configure(fg="#b00020" if status.faults else "#146c2e")
        if not self.sliders_initialized:
            for axis, value in enumerate(status.reference):
                self.slider_vars[axis].set(value)
                self.slider_labels[axis].set(f"{value:.1f} %")
            self.sliders_initialized = True
        for axis, name in enumerate(AXIS_NAMES):
            setpoint = (
                f"{status.force_setpoint[axis - 1]:.2f}" if axis in range(1, 5) else "-"
            )
            measured = f"{status.force_measured[axis]:.2f}" if axis < 5 else "-"
            adc = str(status.adc_raw[axis]) if axis < 5 else "-"
            self.status_tree.item(
                str(axis),
                values=(
                    name,
                    f"{status.reference[axis]:.1f}",
                    f"{status.command[axis]:.1f}",
                    f"{status.actual[axis]:.1f}",
                    status.ticks[axis],
                    setpoint,
                    measured,
                    adc,
                    status.current_ma[axis],
                ),
            )

    def _append_log(self, direction: str, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, f"{stamp} {direction} {text}\n")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 500:
            self.log.delete("1.0", f"{lines - 500}.0")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _on_close(self) -> None:
        self.closing = True
        if self.worker is not None:
            self.worker.close(graceful=self.connected)
        self.root.after(150, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    HandControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
