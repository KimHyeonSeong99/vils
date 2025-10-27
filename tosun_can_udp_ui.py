#!/usr/bin/env python3
"""Simple UI to bridge TOSUN CAN USB frames to UDP packets."""

from __future__ import annotations

import json
import re
import struct
import queue
import socket
import threading
import time
from ctypes import byref, c_char_p, c_int32
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from libTSCANAPI import TSCAN
from libTSCANAPI.TSEnumdefine import (
    A120,
    CHANNEL_INDEX,
    TLIBCANFDControllerMode,
    TLIBCANFDControllerType,
)
from libTSCANAPI.TSStructure import DLC_DATA_BYTE_CNT, TLIBCANFD, size_t


LIB_INITIALIZED = False
CONFIG_FILE_NAMES = (
    "tosun_can_udp_ui.conf",
)


def ensure_lib_initialized() -> None:
    """Initialize the underlying TSCAN library once."""
    global LIB_INITIALIZED
    if not LIB_INITIALIZED:
        TSCAN.initialize_lib_tscan(True, True, False)
        LIB_INITIALIZED = True


def finalize_library() -> None:
    """Finalize the library when the UI exits."""
    global LIB_INITIALIZED
    if LIB_INITIALIZED:
        try:
            TSCAN.finalize_lib_tscan()
        finally:
            LIB_INITIALIZED = False


def _dlc_to_length(dlc: int) -> int:
    if 0 <= dlc < len(DLC_DATA_BYTE_CNT):
        return DLC_DATA_BYTE_CNT[dlc]
    return max(dlc, 0)


def format_canfd_message(msg: TLIBCANFD) -> Tuple[Dict[str, Any], str, bytes]:
    """Convert a TLIBCANFD message into a dictionary payload, log string, and raw data bytes."""
    is_extended = bool(msg.FProperties & 0x04)
    is_remote = bool(msg.FProperties & 0x02)
    is_error = bool(msg.FProperties & 0x80)
    is_tx = bool(msg.FProperties & 0x01)
    is_fd = bool(msg.FFDProperties & 0x01)
    bitrate_switch = bool(msg.FFDProperties & 0x02)
    error_state_indicator = bool(msg.FFDProperties & 0x04)

    raw_identifier = int(msg.FIdentifier) & 0x1FFFFFFF
    identifier = raw_identifier if is_extended else raw_identifier & 0x7FF
    dlc_index = int(msg.FDLC)
    data_length = _dlc_to_length(dlc_index)
    data_bytes = bytes(msg.FData[:data_length])

    timestamp_seconds = float(msg.FTimeUs) / 1_000_000.0

    payload: Dict[str, Any] = {
        "type": "canfd" if is_fd else "can",
        "timestamp_seconds": timestamp_seconds,
        "timestamp_us": int(msg.FTimeUs),
        "channel": int(msg.FIdxChn),
        "id": identifier,
        "is_extended_id": is_extended,
        "direction": "tx" if is_tx else "rx",
        "is_remote_frame": is_remote,
        "is_error_frame": is_error,
        "is_fd": is_fd,
        "bitrate_switch": bitrate_switch,
        "error_state_indicator": error_state_indicator,
        "dlc": data_length,
        "data": data_bytes.hex().upper(),
    }

    id_width = 8 if is_extended else 3
    id_fmt = f"0x{identifier:0{id_width}X}"
    flag_tokens: List[str] = []
    if is_fd:
        flag_tokens.append("FD")
    if bitrate_switch:
        flag_tokens.append("BRS")
    if error_state_indicator:
        flag_tokens.append("ESI")
    if is_remote:
        flag_tokens.append("RTR")
    if is_error:
        flag_tokens.append("ERR")
    flags = ",".join(flag_tokens) if flag_tokens else "-"

    data_str = " ".join(f"{byte:02X}" for byte in data_bytes)
    log_line = (
        f"{timestamp_seconds:10.6f}s  "
        f"CH{msg.FIdxChn:<2}  "
        f"{'Tx' if is_tx else 'Rx':<2}  "
        f"{'EXT' if is_extended else 'STD'}  "
        f"{id_fmt:<12}  "
        f"DL={data_length:<2}  "
        f"{flags:<11}  "
        f"{data_str}"
    )
    return payload, log_line, data_bytes


class TosunCANInterface:
    """Thin wrapper around the TSCAN API used in this tool."""

    def __init__(self) -> None:
        ensure_lib_initialized()
        self.handle = size_t(0)
        self.connected = False

    def connect(
        self,
        serial_number: str,
        channel: int,
        arb_bitrate_kbps: float,
        data_bitrate_kbps: float,
        use_fd: bool,
        enable_termination: bool,
    ) -> None:
        if self.connected:
            raise RuntimeError("Device is already connected.")

        serial_bytes = serial_number.encode("utf-8") if serial_number else b""
        result = TSCAN.tsapp_connect(serial_bytes, self.handle)
        if result not in (0, 5):
            raise RuntimeError(f"Failed to connect device (error code {result}).")

        term_setting = A120.ENABLEA120 if enable_termination else A120.DEABLEA120
        channel_index = CHANNEL_INDEX(channel)

        if use_fd:
            result = TSCAN.tsapp_configure_baudrate_canfd(
                self.handle,
                channel_index,
                float(arb_bitrate_kbps),
                float(data_bitrate_kbps),
                TLIBCANFDControllerType.lfdtISOCAN,
                TLIBCANFDControllerMode.lfdmNormal,
                term_setting,
            )
        else:
            result = TSCAN.tsapp_configure_baudrate_can(
                self.handle,
                channel_index,
                float(arb_bitrate_kbps),
                term_setting,
            )

        if result != 0:
            TSCAN.tsapp_disconnect_by_handle(self.handle)
            raise RuntimeError(f"Failed to configure channel (error code {result}).")

        self.connected = True

    def disconnect(self) -> None:
        if self.connected:
            TSCAN.tsapp_disconnect_by_handle(self.handle)
            self.connected = False

    def receive_messages(
        self,
        channel: int,
        include_tx_frames: bool,
        batch_size: int = 32,
    ) -> List[TLIBCANFD]:
        if not self.connected:
            return []

        batch_size = max(batch_size, 1)
        buffer = (TLIBCANFD * batch_size)()
        requested = c_int32(batch_size)
        TSCAN.tsfifo_receive_canfd_msgs(
            self.handle,
            buffer,
            requested,
            channel,
            1 if include_tx_frames else 0,
        )

        actual = max(0, requested.value)
        return [buffer[index] for index in range(actual)]


class CANToUDPApp:
    """Tkinter based UI application."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("TOSUN CAN -> UDP Bridge")
        self.root.geometry("700x1000")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.devices: List[Dict[str, str]] = []
        self.interface: Optional[TosunCANInterface] = None
        self.udp_socket: Optional[socket.socket] = None
        self.udp_target: Optional[Tuple[str, int]] = None

        self.reader_thread: Optional[threading.Thread] = None
        self.reader_active = False
        self.log_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self.frame_counter = 0

        self.conf_settings: Dict[str, Any] = {}
        self.conf_settings_order: List[str] = []
        self.conf_settings_types: Dict[str, str] = {}
        self.conf_signal_exprs: Dict[str, Any] = {}
        self.conf_signal_raw: Dict[str, str] = {}
        self.conf_signal_order: List[str] = []
        self.conf_signal_types: Dict[str, str] = {}
        self.conf_signal_values: Dict[str, Any] = {}
        self.latest_frames: Dict[int, bytes] = {}
        self.conf_lock = threading.Lock()
        self.required_can_ids: Set[int] = set()

        self.listen_port: Optional[int] = None
        self.listen_socket: Optional[socket.socket] = None
        self.listen_thread: Optional[threading.Thread] = None
        self.listen_active = False
        self._last_payload_hex: str = ""
        self.gear_override_enabled = tk.BooleanVar(value=False)
        self.gear_override_selection = tk.StringVar(value="P")
        self._gear_override_buttons: List[ttk.Radiobutton] = []

        self.last_udp_payload_var = tk.StringVar(value="(no data)")
        try:
            script_dir = Path(__file__).resolve().parent
        except (NameError, OSError):
            script_dir = Path.cwd()
        self.default_config_paths = [script_dir / name for name in CONFIG_FILE_NAMES]
        self.last_config_path: Optional[Path] = None
        self.send_period_var = tk.StringVar(value="0")
        self.send_period_sec: float = 0.0
        self._next_udp_send_time: float = 0.0
        self._last_payload_bytes: Optional[bytes] = None
        self._last_payload_text: str = ""
        self.active_channel: Optional[int] = None
        self._mousewheel_bound = False

        self._build_layout()
        ensure_lib_initialized()
        self.refresh_devices()
        self._schedule_queue_processing()
        self._load_default_config()

    def _build_layout(self) -> None:
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        outer_frame = ttk.Frame(self.root)
        outer_frame.grid(row=0, column=0, sticky="nsew")
        outer_frame.grid_rowconfigure(0, weight=1)
        outer_frame.grid_columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(outer_frame, borderwidth=0, highlightthickness=0)
        vscroll = ttk.Scrollbar(outer_frame, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=vscroll.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vscroll.grid(row=0, column=1, sticky="ns")

        self.content_frame = ttk.Frame(self.canvas)
        self._content_window = self.canvas.create_window((0, 0), window=self.content_frame, anchor="nw")
        self.content_frame.bind("<Configure>", self._on_content_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.content_frame.bind("<Enter>", lambda _event: self._bind_mousewheel())
        self.content_frame.bind("<Leave>", lambda _event: self._unbind_mousewheel())

        self.content_frame.grid_columnconfigure(0, weight=1)

        device_frame = ttk.LabelFrame(self.content_frame, text="Device")
        device_frame.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))
        device_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(device_frame, text="Detected devices:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.device_var = tk.StringVar()
        self.device_combo = ttk.Combobox(device_frame, textvariable=self.device_var, state="readonly", width=45)
        self.device_combo.grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        self.device_combo.bind("<<ComboboxSelected>>", self._on_device_selected)

        self.refresh_button = ttk.Button(device_frame, text="Refresh", command=self.refresh_devices)
        self.refresh_button.grid(row=0, column=2, padx=4, pady=4)

        ttk.Label(device_frame, text="Serial override:").grid(row=1, column=0, padx=4, pady=4, sticky="w")
        self.serial_var = tk.StringVar()
        self.serial_entry = ttk.Entry(device_frame, textvariable=self.serial_var)
        self.serial_entry.grid(row=1, column=1, padx=4, pady=4, sticky="ew")
        ttk.Label(device_frame, text="Leave empty to use the first available device.").grid(
            row=1, column=2, padx=4, pady=4, sticky="w"
        )

        can_frame = ttk.LabelFrame(self.content_frame, text="CAN Configuration")
        can_frame.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        can_frame.grid_columnconfigure(1, weight=1)
        can_frame.grid_columnconfigure(3, weight=1)

        ttk.Label(can_frame, text="Channel:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.channel_var = tk.IntVar(value=0)
        self.channel_spin = ttk.Spinbox(can_frame, from_=0, to=31, textvariable=self.channel_var, width=5)
        self.channel_spin.grid(row=0, column=1, padx=4, pady=4, sticky="w")

        ttk.Label(can_frame, text="Arbitration bitrate (kbps):").grid(row=0, column=2, padx=4, pady=4, sticky="e")
        self.arb_bitrate_var = tk.StringVar(value="500")
        self.arb_bitrate_entry = ttk.Entry(can_frame, textvariable=self.arb_bitrate_var, width=10)
        self.arb_bitrate_entry.grid(row=0, column=3, padx=4, pady=4, sticky="w")

        ttk.Label(can_frame, text="Data bitrate (kbps):").grid(row=1, column=2, padx=4, pady=4, sticky="e")
        self.data_bitrate_var = tk.StringVar(value="2000")
        self.data_bitrate_entry = ttk.Entry(can_frame, textvariable=self.data_bitrate_var, width=10)
        self.data_bitrate_entry.grid(row=1, column=3, padx=4, pady=4, sticky="w")

        self.use_fd_var = tk.BooleanVar(value=False)
        self.use_fd_check = ttk.Checkbutton(
            can_frame, text="Use CAN FD", variable=self.use_fd_var, command=self._update_fd_state
        )
        self.use_fd_check.grid(row=1, column=0, padx=4, pady=4, sticky="w")

        self.termination_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(can_frame, text="Enable 120 Ohm termination", variable=self.termination_var).grid(
            row=1, column=1, padx=4, pady=4, sticky="w"
        )

        self.include_tx_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(can_frame, text="Include TX frames", variable=self.include_tx_var).grid(
            row=2, column=0, padx=4, pady=4, sticky="w"
        )

        udp_frame = ttk.LabelFrame(self.content_frame, text="UDP Target")
        udp_frame.grid(row=2, column=0, sticky="ew", padx=8, pady=4)
        udp_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(udp_frame, text="IP address:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.udp_ip_var = tk.StringVar(value="127.0.0.1")
        self.udp_ip_entry = ttk.Entry(udp_frame, textvariable=self.udp_ip_var)
        self.udp_ip_entry.grid(row=0, column=1, padx=4, pady=4, sticky="ew")

        ttk.Label(udp_frame, text="Port:").grid(row=0, column=2, padx=4, pady=4, sticky="e")
        self.udp_port_var = tk.StringVar(value="5000")
        self.udp_port_entry = ttk.Entry(udp_frame, textvariable=self.udp_port_var, width=8)
        self.udp_port_entry.grid(row=0, column=3, padx=4, pady=4, sticky="w")

        ttk.Label(udp_frame, text="Send period (ms):").grid(row=1, column=0, padx=4, pady=4, sticky="w")
        self.send_period_entry = ttk.Entry(udp_frame, textvariable=self.send_period_var, width=10)
        self.send_period_entry.grid(row=1, column=1, padx=4, pady=4, sticky="w")
        ttk.Label(udp_frame, text="0 = no delay").grid(row=1, column=2, columnspan=2, padx=4, pady=4, sticky="w")

        gear_frame = ttk.LabelFrame(self.content_frame, text="Gear Override")
        gear_frame.grid(row=3, column=0, sticky="ew", padx=8, pady=4)
        for col in range(4):
            gear_frame.grid_columnconfigure(col, weight=1)

        ttk.Checkbutton(
            gear_frame,
            text="Enable manual gear override",
            variable=self.gear_override_enabled,
            command=self._on_gear_override_toggle,
        ).grid(row=0, column=0, columnspan=4, padx=4, pady=4, sticky="w")

        self._gear_override_buttons = []
        for idx, (label, value) in enumerate(
            [("P (Park)", "P"), ("R (Reverse)", "R"), ("N (Neutral)", "N"), ("D (Drive)", "D")]
        ):
            btn = ttk.Radiobutton(
                gear_frame,
                text=label,
                value=value,
                variable=self.gear_override_selection,
                command=self._on_gear_override_selection,
            )
            btn.grid(row=1, column=idx, padx=4, pady=4, sticky="w")
            self._gear_override_buttons.append(btn)

        button_frame = ttk.Frame(self.content_frame)
        button_frame.grid(row=4, column=0, sticky="ew", padx=8, pady=4)
        button_frame.grid_columnconfigure(1, weight=1)

        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_bridge)
        self.start_button.grid(row=0, column=0, padx=4, pady=4)

        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_bridge, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1, padx=4, pady=4)

        payload_frame = ttk.LabelFrame(self.content_frame, text="Last UDP Payload")
        payload_frame.grid(row=6, column=0, sticky="ew", padx=8, pady=4)
        payload_frame.grid_columnconfigure(0, weight=1)
        payload_frame.grid_rowconfigure(0, weight=1)
        self.payload_text = scrolledtext.ScrolledText(
            payload_frame,
            height=5,
            wrap=tk.WORD,
            state=tk.DISABLED,
            font=("Consolas", 10),
        )
        self.payload_text.grid(row=0, column=0, sticky="nsew", padx=6, pady=4)

        hex_frame = ttk.Frame(payload_frame)
        hex_frame.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 4))
        ttk.Label(hex_frame, text="Raw HEX:").grid(row=0, column=0, padx=(0, 4), pady=2, sticky="w")
        self.payload_hex_text = scrolledtext.ScrolledText(
            hex_frame,
            height=3,
            wrap=tk.WORD,
            state=tk.DISABLED,
            font=("Consolas", 10),
        )
        self.payload_hex_text.grid(row=0, column=1, sticky="ew")
        hex_frame.grid_columnconfigure(1, weight=1)

        status_frame = ttk.Frame(self.content_frame)
        status_frame.grid(row=7, column=0, sticky="ew", padx=8, pady=(0, 8))
        status_frame.grid_columnconfigure(0, weight=1)
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(status_frame, textvariable=self.status_var).grid(row=0, column=0, sticky="w")

        self._update_fd_state()
        self._update_gear_override_state()
        # Populate payload display with any existing data
        self._update_payload_display(self.last_udp_payload_var.get())

    def _on_content_configure(self, _event: Any) -> None:
        if hasattr(self, "canvas"):
            self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event: Any) -> None:
        if hasattr(self, "_content_window"):
            self.canvas.itemconfigure(self._content_window, width=event.width)

    def _bind_mousewheel(self) -> None:
        if hasattr(self, "_mousewheel_bound") and self._mousewheel_bound:
            return
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel, add="+")
        self.canvas.bind_all("<Button-4>", self._on_mousewheel, add="+")
        self.canvas.bind_all("<Button-5>", self._on_mousewheel, add="+")
        self._mousewheel_bound = True

    def _unbind_mousewheel(self) -> None:
        if not getattr(self, "_mousewheel_bound", False):
            return
        self.canvas.unbind_all("<MouseWheel>")
        self.canvas.unbind_all("<Button-4>")
        self.canvas.unbind_all("<Button-5>")
        self._mousewheel_bound = False

    def _on_mousewheel(self, event: Any) -> None:
        if event.num == 4 or getattr(event, "delta", 0) > 0:
            self.canvas.yview_scroll(-1, "units")
        elif event.num == 5 or getattr(event, "delta", 0) < 0:
            self.canvas.yview_scroll(1, "units")

    def _on_device_selected(self, _event: Any) -> None:
        index = self.device_combo.current()
        if 0 <= index < len(self.devices):
            serial = self.devices[index].get("serial", "")
            self.serial_var.set(serial)

    def _update_fd_state(self) -> None:
        if self.use_fd_var.get():
            self.data_bitrate_entry.configure(state=tk.NORMAL)
        else:
            self.data_bitrate_entry.configure(state=tk.DISABLED)

    def _on_gear_override_toggle(self) -> None:
        self._update_gear_override_state()
        self._refresh_gear_override_payload()

    def _on_gear_override_selection(self) -> None:
        self._refresh_gear_override_payload()

    def _update_gear_override_state(self) -> None:
        state = tk.NORMAL if self.gear_override_enabled.get() else tk.DISABLED
        for btn in getattr(self, "_gear_override_buttons", []):
            btn.configure(state=state)
        if not self.gear_override_enabled.get():
            # Rebuild payload with overrides disabled
            self._refresh_gear_override_payload()

    def _refresh_gear_override_payload(self) -> None:
        # If no configuration loaded, nothing to refresh
        if not (self.conf_settings_order or self.conf_signal_order):
            return
        payload = self._build_conf_payload()
        payload_bytes, payload_text = self._pack_conf_payload(payload)
        self._store_last_payload(payload_bytes, payload_text)
        self._update_payload_display(payload_text)
        # If streaming without interval, send immediately so override takes effect
        if self.reader_active and self.send_period_sec <= 0:
            self._send_last_payload(force=True)
        else:
            # Ensure next scheduled send uses updated payload
            self._last_payload_bytes = payload_bytes
            self._last_payload_text = payload_text

    def _apply_gear_override(self, payload: Dict[str, Any]) -> None:
        if not self.gear_override_enabled.get():
            return
        selection = self.gear_override_selection.get().upper()
        mapping = {"P": "GearP", "R": "GearR", "N": "GearN", "D": "GearD"}
        for gear, key in mapping.items():
            if key in payload:
                payload[key] = 1 if gear == selection else 0
        if "ParkingBrake" in payload:
            payload["ParkingBrake"] = 0.0

    def _start_listener(self) -> None:
        listen_port = self.listen_port
        if listen_port is None:
            return
        self._stop_listener()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", listen_port))
            sock.settimeout(1.0)
        except OSError as err:
            self.log_queue.put(("warning", f"Listen port error: {err}"))
            return
        self.listen_socket = sock
        self.listen_active = True
        self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listen_thread.start()
        self.log_queue.put(("listen", f"Listening on UDP port {listen_port}"))

    def _stop_listener(self) -> None:
        self.listen_active = False
        if self.listen_socket is not None:
            try:
                self.listen_socket.close()
            except OSError:
                pass
            self.listen_socket = None
        if self.listen_thread and self.listen_thread.is_alive():
            self.listen_thread.join(timeout=1.0)
        self.listen_thread = None

    def _listen_loop(self) -> None:
        sock = self.listen_socket
        if sock is None:
            return
        expected = self.expected_listener_payload
        expected_ascii = expected.hex().upper().encode("ascii")
        while self.listen_active:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not self.listen_active:
                break
            clean_data = data.strip()
            if data == expected or clean_data.upper() == expected_ascii:
                message = f"Heartbeat OK from {addr[0]}:{addr[1]}"
            else:
                message = f"Listen data {data.hex().upper()} from {addr[0]}:{addr[1]}"
            self.log_queue.put(("listen", message))
            self.log_queue.put(("listen_log", message))
        try:
            sock.close()
        except OSError:
            pass
        self.listen_socket = None
        self.listen_active = False

    def _load_default_config(self) -> None:
        candidates: List[Path] = []
        for path in self.default_config_paths:
            if path.exists():
                candidates.append(path)
        cwd = Path.cwd()
        for name in CONFIG_FILE_NAMES:
            candidate = cwd / name
            if candidate.exists() and candidate not in candidates:
                candidates.append(candidate)
        for candidate in candidates:
            if self._load_config_from_path(candidate, display_dialogs=False):
                break

    def _load_config_from_path(self, path: Path, display_dialogs: bool = True) -> bool:
        suffix = path.suffix.lower()
        if suffix == ".conf":
            return self._load_conf_file(path, display_dialogs=display_dialogs)

        try:
            raw_text = path.read_text(encoding="utf-8")
        except FileNotFoundError:
            if display_dialogs:
                messagebox.showerror("Config load", f"Config file not found: {path}")
            return False
        except OSError as exc:
            if display_dialogs:
                messagebox.showerror("Config load", f"Failed to open config file: {exc}")
            return False

        try:
            data = json.loads(raw_text)
        except json.JSONDecodeError as exc:
            if display_dialogs:
                messagebox.showerror("Config load", f"Invalid JSON: {exc}")
            return False

        if not isinstance(data, dict):
            if display_dialogs:
                messagebox.showerror("Config load", "Config file must contain a JSON object at the top level.")
            return False

        self.last_config_path = path
        with self.conf_lock:
            self.conf_settings = {}
            self.conf_settings_order = []
            self.conf_settings_types = {}
            self.conf_signal_raw = {}
            self.conf_signal_exprs = {}
            self.conf_signal_order = []
            self.conf_signal_types = {}
            self.conf_signal_values = {}
            self.latest_frames.clear()
            self.listen_port = None
            self.required_can_ids.clear()
        warnings: List[str] = []
        applied: List[str] = []

        def parse_int(value: Any) -> Optional[int]:
            if isinstance(value, int):
                return value
            if isinstance(value, str):
                text_value = value.strip()
                if not text_value:
                    return None
                base = 10
                lower = text_value.lower()
                if lower.startswith("-0x") or lower.startswith("0x"):
                    base = 16
                return int(text_value, base)
            return None

        udp_section = data.get("udp") if isinstance(data.get("udp"), dict) else None
        udp_ip = None
        udp_port_raw = None
        send_period_raw: Any = None

        listen_port_raw: Any = None

        if udp_section is not None:
            udp_ip = udp_section.get("ip") or udp_section.get("address")
            udp_port_raw = udp_section.get("port")
            send_period_raw = (
                udp_section.get("send_period_ms")
                if "send_period_ms" in udp_section
                else udp_section.get("send_period")
            )
            for key in ("listen_port", "listenPort", "myport", "MyPort"):
                if key in udp_section:
                    listen_port_raw = udp_section.get(key)
                    break
        else:
            udp_ip = data.get("udp_ip")
            udp_port_raw = data.get("udp_port")
            send_period_raw = data.get("send_period_ms")
            for key in ("listen_port", "listenPort", "myport", "MyPort"):
                if data.get(key) is not None:
                    listen_port_raw = data.get(key)
                    break

        if udp_ip is not None:
            ip_text = str(udp_ip).strip()
            if ip_text:
                self.udp_ip_var.set(ip_text)
                applied.append("udp ip")
            else:
                warnings.append("UDP IP value is empty.")

        if udp_port_raw is not None:
            try:
                udp_port_parsed = parse_int(udp_port_raw)
            except (TypeError, ValueError):
                warnings.append(f"Invalid UDP port value: {udp_port_raw}")
            else:
                if udp_port_parsed is None:
                    warnings.append("UDP port value is empty.")
                elif not (0 <= udp_port_parsed <= 65535):
                    warnings.append(f"UDP port out of range (0-65535): {udp_port_parsed}")
                else:
                    self.udp_port_var.set(str(udp_port_parsed))
                    applied.append("udp port")

        if send_period_raw is not None:
            try:
                send_period_value = float(str(send_period_raw))
            except (TypeError, ValueError):
                warnings.append(f"Invalid UDP send period value: {send_period_raw}")
            else:
                if send_period_value < 0:
                    warnings.append(f"UDP send period must be non-negative, got {send_period_value}.")
                else:
                    self.send_period_var.set(str(send_period_value))
                    applied.append("send period")
        if listen_port_raw is not None:
            try:
                listen_port_parsed = parse_int(listen_port_raw)
            except (TypeError, ValueError):
                listen_port_parsed = None
            if listen_port_parsed is None:
                warnings.append(f"Invalid listen port value: {listen_port_raw}")
            elif not (0 <= listen_port_parsed <= 65535):
                warnings.append(f"Listen port out of range (0-65535): {listen_port_parsed}")
            else:
                with self.conf_lock:
                    self.listen_port = listen_port_parsed
                applied.append("listen port")

        if applied:
            summary = "; ".join(dict.fromkeys(applied))
            status_message = f"Config loaded from {path}: {summary}"
        else:
            status_message = f"Config loaded from {path}"
        final_status = status_message
        if warnings:
            if display_dialogs:
                messagebox.showwarning("Config load", "\n".join(warnings))
            final_status = f"{status_message} (warnings: {len(warnings)})"
        self.status_var.set(final_status)

        return True

    def _load_conf_file(self, path: Path, display_dialogs: bool = True) -> bool:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except FileNotFoundError:
            if display_dialogs:
                messagebox.showerror("Config load", f"Config file not found: {path}")
            return False
        except OSError as exc:
            if display_dialogs:
                messagebox.showerror("Config load", f"Failed to open config file: {exc}")
            return False

        settings: Dict[str, Any] = {}
        settings_order: List[str] = []
        settings_types: Dict[str, str] = {}
        raw_exprs: Dict[str, str] = {}
        compiled_exprs: Dict[str, Any] = {}
        signal_order: List[str] = []
        signal_types: Dict[str, str] = {}
        warnings: List[str] = []
        listen_port_value: Optional[int] = None
        collected_ids: Set[int] = set()

        for idx, line in enumerate(lines, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if "#" in stripped:
                stripped = stripped.split("#", 1)[0].strip()
            if not stripped:
                continue
            type_hint: Optional[str] = None
            key: Optional[str] = None
            expression_text: str = ""
            tokens = stripped.split()
            if not tokens:
                continue
            if tokens[0].lower() in {"float", "double", "int", "short", "byte"}:
                if len(tokens) < 2:
                    continue
                type_hint = tokens[0].lower()
                key = tokens[1]
                expression_text = " ".join(tokens[2:]) if len(tokens) > 2 else "0"
            else:
                if len(tokens) < 2:
                    continue
                key = tokens[0]
                expression_text = " ".join(tokens[1:])
            key = key.strip()
            expression_text = expression_text.strip()
            if not expression_text:
                expression_text = "0"
            raw_expression_text = expression_text
            exists_mode: Optional[str] = None
            if expression_text.endswith("?!"):
                exists_mode = "neg"
                expression_text = expression_text[:-2].rstrip()
            elif expression_text.endswith("?"):
                exists_mode = "pos"
                expression_text = expression_text[:-1].rstrip()
            if not expression_text:
                expression_text = "0"
            key_lower = key.lower()

            def parse_int_token(token: str) -> Optional[int]:
                try:
                    return int(token, 0)
                except ValueError:
                    return None

            if key_lower in {"headermask", "protocolversion", "packetsize", "messagetype"}:
                value = parse_int_token(expression_text)
                if value is None:
                    warnings.append(f"Line {idx}: unable to parse integer for '{key}'.")
                    continue
                settings[key] = value
                if key not in settings_order:
                    settings_order.append(key)
                resolved_type = type_hint or self._infer_type_from_expression(expression_text)
                settings_types[key] = resolved_type
                continue
            if key_lower == "target":
                self.udp_ip_var.set(expression_text)
                continue
            if key_lower == "port":
                value = parse_int_token(expression_text)
                if value is None:
                    warnings.append(f"Line {idx}: invalid UDP port '{line}'.")
                    continue
                self.udp_port_var.set(str(value))
                continue
            if key_lower in {"listenport", "myport"}:
                value = parse_int_token(expression_text)
                if value is None:
                    warnings.append(f"Line {idx}: invalid listen port '{line}'.")
                    continue
                listen_port_value = value
                continue
            if key_lower == "period":
                try:
                    float(expression_text)
                except ValueError:
                    warnings.append(f"Line {idx}: invalid period '{line}'.")
                    continue
                self.send_period_var.set(str(expression_text))
                continue

            converted = self._convert_conf_expression(expression_text)
            if exists_mode == "pos":
                converted = f"(1 if ({converted}) else 0)"
            elif exists_mode == "neg":
                converted = f"(1 if (not ({converted})) else 0)"
            try:
                compiled = compile(converted, f"{path.name}:{idx}", "eval")
            except Exception as exc:  # pragma: no cover - configuration errors
                warnings.append(f"Line {idx}: failed to compile expression for '{key}': {exc}")
                continue
            raw_exprs[key] = raw_expression_text

            compiled_exprs[key] = compiled
            if key not in signal_order:
                signal_order.append(key)
            if exists_mode is not None and type_hint is None:
                resolved_signal_type = "byte"
            else:
                inference_expr = raw_expression_text if exists_mode is not None else expression_text
                resolved_signal_type = type_hint or self._infer_type_from_expression(inference_expr)
            signal_types[key] = resolved_signal_type

        with self.conf_lock:
            self.conf_settings = settings
            self.conf_settings_order = settings_order
            self.conf_settings_types = settings_types
            self.conf_signal_raw = raw_exprs
            self.conf_signal_exprs = compiled_exprs
            self.conf_signal_order = signal_order
            self.conf_signal_types = signal_types
            self.conf_signal_values = {name: 0 for name in signal_order}
            self.latest_frames.clear()
            self.listen_port = listen_port_value
            self.required_can_ids.clear()
            self.required_can_ids.update(collected_ids)

        self.last_config_path = path
        summary_parts: List[str] = []
        if settings:
            summary_parts.append(f"settings:{len(settings)}")
        if compiled_exprs:
            summary_parts.append(f"signals:{len(compiled_exprs)}")
        summary = ", ".join(summary_parts) if summary_parts else "no entries"
        status = f"Conf loaded from {path.name} ({summary})"
        if warnings:
            if display_dialogs:
                messagebox.showwarning("Config load", "\n".join(warnings))
            status += f" with {len(warnings)} warning(s)"
        self.status_var.set(status)
        return True

    def _convert_conf_expression(self, expression: str) -> str:
        def replace_range(match: re.Match[str]) -> str:
            can_id = match.group("id")
            start = match.group("start")
            end = match.group("end")
            return f"get_bits(0x{can_id.upper()}, {start}, {end})"

        def replace_single(match: re.Match[str]) -> str:
            can_id = match.group("id")
            bit_index = match.group("bit")
            return f"get_bits(0x{can_id.upper()}, {bit_index}, {bit_index})"

        pattern_range = re.compile(r"0x(?P<id>[0-9a-fA-F]+)\[(?P<start>\d+):(?P<end>\d+)\]")
        pattern_single = re.compile(r"0x(?P<id>[0-9a-fA-F]+)\[(?P<bit>\d+)\]")
        pattern_ternary = re.compile(r"(?P<cond>[^?:]+?)\?\s*(?P<true>[^:]+?)\s*:\s*(?P<false>.+)")

        def replace_ternary(match: re.Match[str]) -> str:
            condition = match.group("cond").strip()
            true_expr = match.group("true").strip()
            false_expr = match.group("false").strip()
            return f"(({true_expr}) if ({condition}) else ({false_expr}))"

        converted = pattern_range.sub(replace_range, expression)
        converted = pattern_single.sub(replace_single, converted)
        if "?" in converted and ":" in converted:
            converted = pattern_ternary.sub(replace_ternary, converted)
        return converted

    def _evaluate_conf_signals(self) -> Dict[str, Any]:
        with self.conf_lock:
            if not self.conf_signal_exprs:
                return {}
            frames_snapshot = dict(self.latest_frames)
            expr_items = list(self.conf_signal_exprs.items())

        def get_bits(can_id: int, start: int, end: int) -> Optional[int]:
            data = frames_snapshot.get(can_id)
            if data is None:
                return None
            bit_length = len(data) * 8
            start_idx = int(start)
            end_idx = int(end)
            if start_idx < 0 or end_idx < start_idx or end_idx >= bit_length:
                return None
            value = int.from_bytes(data, byteorder="little", signed=False)
            mask = (1 << (end_idx - start_idx + 1)) - 1
            return (value >> start_idx) & mask

        results: Dict[str, Any] = {}
        for name, compiled in expr_items:
            try:
                value = eval(compiled, {"__builtins__": {}}, {"get_bits": get_bits})
            except Exception:  # pragma: no cover - defensive
                value = 0
            else:
                if value is None:
                    value = 0
                elif isinstance(value, bool):
                    value = int(value)
                elif isinstance(value, (int, float)):
                    pass
                else:
                    try:
                        value = float(value)
                    except (TypeError, ValueError):
                        value = 0
            results[name] = value

        with self.conf_lock:
            self.conf_signal_values = results
        return results

    def _build_conf_payload(self, values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self.conf_lock:
            payload: Dict[str, Any] = {}
            for key in self.conf_settings_order:
                payload[key] = self.conf_settings.get(key, 0)
            signal_values = values if values is not None else self.conf_signal_values
            for key in self.conf_signal_order:
                payload[key] = signal_values.get(key, 0)
        self._apply_gear_override(payload)
        return payload

    @staticmethod
    def _infer_type_from_expression(expression: str) -> str:
        lowered = expression.lower()
        if any(token in lowered for token in (".", "*", "/", "get_bits")):
            return "float"
        if "?" in expression and ":" in expression:
            return "byte"
        return "int"

    @staticmethod
    def _normalize_numeric(value: Any) -> float:
        if value is None:
            return 0.0
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _pack_value(self, value: Any, type_str: str) -> Tuple[bytes, Any]:
        type_key = (type_str or "float").lower()
        if type_key == "byte":
            normalized = int(round(self._normalize_numeric(value))) & 0xFF
            return struct.pack("<B", normalized), normalized
        if type_key == "short":
            normalized = int(round(self._normalize_numeric(value))) & 0xFFFF
            return struct.pack("<H", normalized), normalized
        if type_key == "int":
            normalized = int(round(self._normalize_numeric(value)))
            return struct.pack("<i", normalized), normalized
        normalized = float(self._normalize_numeric(value))
        return struct.pack("<f", normalized), normalized

    def _pack_conf_payload(self, payload: Dict[str, Any]) -> Tuple[bytes, str]:
        with self.conf_lock:
            ordered_names = list(self.conf_settings_order) + list(self.conf_signal_order)
            type_map = {**self.conf_settings_types, **self.conf_signal_types}
        data = bytearray()
        parts: List[str] = []
        for name in ordered_names:
            packed_bytes, normalized = self._pack_value(payload.get(name, 0), type_map.get(name, "float"))
            data.extend(packed_bytes)
            parts.append(f"{name}={normalized}")
        hex_repr = " ".join(f"{byte:02X}" for byte in data)
        text_repr = ", ".join(parts)
        if text_repr:
            text_repr = f"{text_repr}"
        self._last_payload_hex = hex_repr
        return bytes(data), text_repr

    def refresh_devices(self) -> None:
        try:
            ensure_lib_initialized()
            count = c_int32(0)
            result = TSCAN.tscan_scan_devices(byref(count))
            if result not in (0, 5) and count.value == 0:
                raise RuntimeError(f"Device scan failed (error code {result}).")

            device_entries: List[Dict[str, str]] = []
            labels: List[str] = []
            for idx in range(max(0, count.value)):
                manufacturer_ptr = c_char_p()
                product_ptr = c_char_p()
                serial_ptr = c_char_p()

                device_result = TSCAN.tscan_get_device_info(
                    idx,
                    byref(manufacturer_ptr),
                    byref(product_ptr),
                    byref(serial_ptr),
                )
                if device_result != 0:
                    continue

                manufacturer_value = manufacturer_ptr.value.decode("utf-8", errors="ignore") if manufacturer_ptr.value else ""
                product_value = product_ptr.value.decode("utf-8", errors="ignore") if product_ptr.value else ""
                serial_value = serial_ptr.value.decode("utf-8", errors="ignore") if serial_ptr.value else ""

                label = f"{serial_value or 'N/A'} ({product_value or 'Unknown'} / {manufacturer_value or 'Unknown'})"
                device_entries.append({"serial": serial_value, "label": label})
                labels.append(label)

            self.devices = device_entries
            self.device_combo.configure(values=labels)
            if labels:
                self.device_combo.current(0)
                self.serial_var.set(self.devices[0].get("serial", ""))
            else:
                self.device_combo.set("")
                self.serial_var.set("")

            self.status_var.set(f"Found {len(self.devices)} device(s).")
        except Exception as exc:
            self.status_var.set(f"Device refresh failed: {exc}")

    def start_bridge(self) -> None:
        if self.reader_active:
            return

        try:
            channel = CHANNEL_INDEX(self.channel_var.get()).value
        except Exception:
            messagebox.showerror("Invalid channel", "Channel must be between 0 and 31.")
            return

        try:
            arb_bitrate = float(self.arb_bitrate_var.get())
        except ValueError:
            messagebox.showerror("Invalid bitrate", "Arbitration bitrate must be a number.")
            return

        use_fd = self.use_fd_var.get()
        try:
            data_bitrate = float(self.data_bitrate_var.get()) if use_fd else arb_bitrate
        except ValueError:
            messagebox.showerror("Invalid bitrate", "Data bitrate must be a number.")
            return

        udp_ip = self.udp_ip_var.get().strip()
        if not udp_ip:
            messagebox.showerror("Invalid IP", "Enter a UDP destination IP address.")
            return

        try:
            udp_port = int(self.udp_port_var.get())
            if not (0 <= udp_port <= 65535):
                raise ValueError()
        except ValueError:
            messagebox.showerror("Invalid port", "UDP port must be an integer between 0 and 65535.")
            return

        try:
            period_text = self.send_period_var.get().strip()
            send_period_ms = float(period_text) if period_text else 0.0
            if send_period_ms < 0:
                raise ValueError()
        except ValueError:
            messagebox.showerror("Invalid period", "Send period must be zero or a positive number (milliseconds).")
            return

        serial_override = self.serial_var.get().strip()

        try:
            self.interface = TosunCANInterface()
            self.interface.connect(
                serial_override,
                channel,
                arb_bitrate,
                data_bitrate,
                use_fd,
                self.termination_var.get(),
            )
        except Exception as exc:
            self.interface = None
            messagebox.showerror("Connection failed", str(exc))
            return

        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_target = (udp_ip, udp_port)
        except OSError as err:
            self.interface.disconnect()
            self.interface = None
            messagebox.showerror("UDP error", f"Failed to open UDP socket: {err}")
            return

        self.frame_counter = 0
        self.reader_active = True
        self.active_channel = channel
        self.send_period_sec = send_period_ms / 1000.0
        self._next_udp_send_time = time.monotonic() if self.send_period_sec > 0 else 0.0
        self._last_payload_bytes = None
        self._last_payload_text = ""
        with self.conf_lock:
            self.latest_frames.clear()
            self.conf_signal_values = {}
        self._set_controls_running(True)

        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
            args=(channel, self.include_tx_var.get()),
        )
        self.reader_thread.start()
        self._start_listener()
        if self.send_period_sec > 0:
            self._ensure_placeholder_payload()
            self._send_last_payload(force=True)
        self.status_var.set(f"Streaming to UDP {udp_ip}:{udp_port}")

    def stop_bridge(self) -> None:
        if not self.reader_active:
            return

        self.reader_active = False
        self._stop_listener()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.5)
        self.reader_thread = None

        self._next_udp_send_time = 0.0
        self._last_payload_bytes = None
        self._last_payload_text = ""
        self.active_channel = None

        if self.interface:
            try:
                self.interface.disconnect()
            except Exception:
                pass
            self.interface = None

        if self.udp_socket:
            try:
                self.udp_socket.close()
            except Exception:
                pass
            self.udp_socket = None
            self.udp_target = None

        self._set_controls_running(False)
        self.status_var.set("Stopped.")

    def clear_log(self) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _set_controls_running(self, running: bool) -> None:
        state = tk.DISABLED if running else tk.NORMAL
        for control in [
            self.device_combo,
            self.refresh_button,
            self.serial_entry,
            self.channel_spin,
            self.arb_bitrate_entry,
            self.data_bitrate_entry,
            self.use_fd_check,
            self.udp_ip_entry,
            self.udp_port_entry,
            self.send_period_entry,
        ]:
            control.configure(state=state)

        self.start_button.configure(state=tk.DISABLED if running else tk.NORMAL)
        self.stop_button.configure(state=tk.NORMAL if running else tk.DISABLED)

    def _reader_loop(self, channel: int, include_tx: bool) -> None:
        assert self.interface is not None
        assert self.udp_socket is not None
        assert self.udp_target is not None

        frame_count = 0
        try:
            while self.reader_active:
                messages = self.interface.receive_messages(channel, include_tx, batch_size=32)
                if not messages:
                    if self.send_period_sec > 0:
                        self._send_last_payload()
                        if not self.reader_active:
                            break
                    time.sleep(0.01)
                    continue

                for message in messages:
                    frame_info, log_line, data_bytes = format_canfd_message(message)
                    if self.conf_signal_exprs:
                        with self.conf_lock:
                            self.latest_frames[frame_info["id"]] = data_bytes
                        conf_values = self._evaluate_conf_signals()
                    else:
                        conf_values = {}
                    payload_dict = self._build_conf_payload(conf_values)
                    payload_bytes, payload_text = self._encode_udp_payload(payload_dict)
                    self._store_last_payload(payload_bytes, payload_text)
                    self.log_queue.put(("log", log_line))
                    frame_count += 1
                    if frame_count % 10 == 0:
                        self.log_queue.put(("counter", frame_count))
                    if self.send_period_sec <= 0:
                        self._send_last_payload(force=True)
                        if not self.reader_active:
                            break
                if not self.reader_active:
                    break
                if self.send_period_sec > 0:
                    self._send_last_payload()
                    if not self.reader_active:
                        break
        except Exception as exc:  # pylint: disable=broad-except
            self.log_queue.put(("error", f"Reader error: {exc}"))
        finally:
            self.log_queue.put(("stopped", None))

    def _encode_udp_payload(self, payload: Dict[str, Any]) -> Tuple[bytes, str]:
        return self._pack_conf_payload(payload)

    def _store_last_payload(self, payload_bytes: bytes, payload_text: str) -> None:
        self._last_payload_bytes = payload_bytes
        self._last_payload_text = payload_text

    def _build_zero_payload(self) -> Optional[Dict[str, Any]]:
        return self._build_conf_payload()

    def _ensure_placeholder_payload(self) -> bool:
        if self._last_payload_bytes is not None:
            return True
        payload = self._build_zero_payload()
        if payload is None:
            return False
        payload_bytes, payload_text = self._encode_udp_payload(payload)
        self._store_last_payload(payload_bytes, payload_text)
        return True

    def _send_last_payload(self, force: bool = False) -> None:
        if self.udp_socket is None or self.udp_target is None:
            return
        if self._last_payload_bytes is None and not self._ensure_placeholder_payload():
            return

        should_send = False
        if force or self.send_period_sec <= 0:
            should_send = True
        else:
            now = time.monotonic()
            if self._next_udp_send_time <= 0:
                self._next_udp_send_time = now
            if now >= self._next_udp_send_time:
                should_send = True
                self._next_udp_send_time = now + self.send_period_sec

        if not should_send:
            return

        try:
            self.udp_socket.sendto(self._last_payload_bytes, self.udp_target)
        except OSError as err:
            self.log_queue.put(("error", f"UDP send failed: {err}"))
            self.reader_active = False
            return

        self.log_queue.put(("udp_payload", self._last_payload_text))
        if self.send_period_sec > 0:
            self._next_udp_send_time = time.monotonic() + self.send_period_sec
    def _update_payload_display(self, text: str) -> None:
        display = text.strip() if isinstance(text, str) else str(text)
        if not display:
            display = "(empty message)"
        truncated = display
        if len(truncated) > 800:
            truncated = truncated[:800] + " ..."
        self.last_udp_payload_var.set(truncated)
        if hasattr(self, "payload_text"):
            try:
                self.payload_text.configure(state=tk.NORMAL)
                self.payload_text.delete("1.0", tk.END)
                self.payload_text.insert(tk.END, display)
                self.payload_text.see(tk.END)
            finally:
                self.payload_text.configure(state=tk.DISABLED)

        hex_text = getattr(self, "_last_payload_hex", "")
        if hasattr(self, "payload_hex_text"):
            try:
                self.payload_hex_text.configure(state=tk.NORMAL)
                self.payload_hex_text.delete("1.0", tk.END)
                if hex_text:
                    self.payload_hex_text.insert(tk.END, hex_text)
                self.payload_hex_text.see("1.0")
            finally:
                self.payload_hex_text.configure(state=tk.DISABLED)

    def _schedule_queue_processing(self) -> None:
        self.root.after(100, self._process_queue)

    def _process_queue(self) -> None:
        try:
            while True:
                event, payload = self.log_queue.get_nowait()
                if event == "log":
                    self.log_text.configure(state=tk.NORMAL)
                    self.log_text.insert(tk.END, payload + "\n")
                    self.log_text.see(tk.END)
                    self.log_text.configure(state=tk.DISABLED)
                elif event == "counter":
                    self.frame_counter = int(payload)
                    self.status_var.set(f"Receiving... {self.frame_counter} frame(s)")
                elif event == "udp_payload":
                    self._update_payload_display(str(payload))
                elif event == "listen":
                    self.status_var.set(str(payload))
                elif event == "listen_log":
                    self.log_text.configure(state=tk.NORMAL)
                    self.log_text.insert(tk.END, "[LISTEN] " + str(payload) + "\n")
                    self.log_text.see(tk.END)
                    self.log_text.configure(state=tk.DISABLED)
                elif event == "error":
                    messagebox.showerror("Reader error", str(payload))
                    self.stop_bridge()
                elif event == "warning":
                    self.status_var.set(str(payload))
                elif event == "stopped":
                    self.stop_bridge()
                self.log_queue.task_done()
        except queue.Empty:
            pass
        finally:
            self._schedule_queue_processing()

    def on_close(self) -> None:
        self.stop_bridge()
        finalize_library()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = CANToUDPApp(root)
    try:
        root.mainloop()
    finally:
        app.stop_bridge()
        finalize_library()


if __name__ == "__main__":
    main()
