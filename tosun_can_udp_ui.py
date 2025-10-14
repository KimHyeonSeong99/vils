#!/usr/bin/env python3
"""Simple UI to bridge TOSUN CAN USB frames to UDP packets."""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
from ctypes import byref, c_char_p, c_int32
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

try:
    import cantools  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    cantools = None

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
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.devices: List[Dict[str, str]] = []
        self.interface: Optional[TosunCANInterface] = None
        self.udp_socket: Optional[socket.socket] = None
        self.udp_target: Optional[Tuple[str, int]] = None

        self.reader_thread: Optional[threading.Thread] = None
        self.reader_active = False
        self.log_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self.frame_counter = 0

        self.dbc_path_var = tk.StringVar()
        self.dbc_status_var = tk.StringVar(value="No DBC loaded.")
        self.dbc_database: Optional[Any] = None
        self.dbc_messages_by_id: Dict[Tuple[int, bool], Any] = {}
        self.message_display_to_key: Dict[str, Tuple[int, bool]] = {}
        self.message_key_to_display: Dict[Tuple[int, bool], str] = {}
        self.signal_mappings: Dict[Tuple[int, str], Dict[str, Any]] = {}
        self.signal_values: Dict[int, float] = {}
        self.signal_tree_items: Dict[Tuple[int, str], str] = {}
        self.signal_lock = threading.Lock()

        self.last_udp_payload_var = tk.StringVar(value="(no data)")
        self.condition_rules: List[Dict[str, Any]] = []
        self.expression_rules: List[Dict[str, Any]] = []
        self.condition_tree_items: Dict[str, Dict[str, Any]] = {}
        self.expression_tree_items: Dict[str, Dict[str, Any]] = {}
        self.condition_rule_counter = 0
        self.expression_rule_counter = 0
        self.latest_signals: Dict[Tuple[Tuple[int, bool], str], float] = {}
        self.message_names_by_key: Dict[Tuple[int, bool], str] = {}
        try:
            script_dir = Path(__file__).resolve().parent
        except (NameError, OSError):
            script_dir = Path.cwd()
        self.default_config_paths = [script_dir / name for name in CONFIG_FILE_NAMES]
        self.last_config_path: Optional[Path] = None

        self._build_layout()
        ensure_lib_initialized()
        self.refresh_devices()
        self._schedule_queue_processing()
        self._load_default_config()

    def _build_layout(self) -> None:
        self.root.grid_columnconfigure(0, weight=1)

        device_frame = ttk.LabelFrame(self.root, text="Device")
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

        can_frame = ttk.LabelFrame(self.root, text="CAN Configuration")
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
        self.use_fd_check = ttk.Checkbutton(can_frame, text="Use CAN FD", variable=self.use_fd_var, command=self._update_fd_state)
        self.use_fd_check.grid(row=1, column=0, padx=4, pady=4, sticky="w")

        self.termination_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(can_frame, text="Enable 120 Ohm termination", variable=self.termination_var).grid(
            row=1, column=1, padx=4, pady=4, sticky="w"
        )

        self.include_tx_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(can_frame, text="Include TX frames", variable=self.include_tx_var).grid(
            row=2, column=0, padx=4, pady=4, sticky="w"
        )

        dbc_frame = ttk.LabelFrame(self.root, text="DBC File")
        dbc_frame.grid(row=2, column=0, sticky="ew", padx=8, pady=4)
        dbc_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(dbc_frame, text="Path:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.dbc_entry = ttk.Entry(dbc_frame, textvariable=self.dbc_path_var, state="readonly")
        self.dbc_entry.grid(row=0, column=1, padx=4, pady=4, sticky="ew")

        ttk.Button(dbc_frame, text="Browse", command=self._browse_dbc).grid(row=0, column=2, padx=4, pady=4)
        self.load_dbc_button = ttk.Button(dbc_frame, text="Load", command=self._load_dbc)
        self.load_dbc_button.grid(row=0, column=3, padx=4, pady=4)

        ttk.Label(dbc_frame, textvariable=self.dbc_status_var).grid(
            row=1, column=0, columnspan=4, padx=4, pady=(0, 4), sticky="w"
        )

        mapping_frame = ttk.LabelFrame(self.root, text="Signal to UDP Index")
        mapping_frame.grid(row=3, column=0, sticky="nsew", padx=8, pady=4)
        mapping_frame.grid_columnconfigure(0, weight=1)
        mapping_frame.grid_rowconfigure(0, weight=1)

        columns = ("index", "message", "signal", "type", "value")
        self.signal_tree = ttk.Treeview(
            mapping_frame,
            columns=columns,
            show="headings",
            height=6,
            selectmode="browse",
        )
        self.signal_tree.heading("index", text="Type+Index")
        self.signal_tree.heading("message", text="Message")
        self.signal_tree.heading("signal", text="Signal")
        self.signal_tree.heading("type", text="Type")
        self.signal_tree.heading("value", text="Last Value")
        self.signal_tree.column("index", width=90, anchor="center")
        self.signal_tree.column("message", width=200, anchor="w")
        self.signal_tree.column("signal", width=160, anchor="w")
        self.signal_tree.column("type", width=80, anchor="center")
        self.signal_tree.column("value", width=120, anchor="center")
        self.signal_tree.grid(row=0, column=0, sticky="nsew")
        self.signal_tree.bind("<<TreeviewSelect>>", self._on_signal_selected)

        tree_scroll = ttk.Scrollbar(mapping_frame, orient="vertical", command=self.signal_tree.yview)
        tree_scroll.grid(row=0, column=1, sticky="ns")
        self.signal_tree.configure(yscrollcommand=tree_scroll.set)

        control_frame = ttk.Frame(mapping_frame)
        control_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=(4, 0))
        control_frame.grid_columnconfigure(1, weight=1)
        control_frame.grid_columnconfigure(3, weight=1)
        control_frame.grid_columnconfigure(5, weight=1)
        control_frame.grid_columnconfigure(7, weight=1)

        self.mapping_index_var = tk.StringVar()
        self.mapping_message_var = tk.StringVar()
        self.mapping_signal_var = tk.StringVar()
        self.mapping_type_var = tk.StringVar(value="float")

        ttk.Label(control_frame, text="Index:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.mapping_index_entry = ttk.Entry(control_frame, textvariable=self.mapping_index_var, width=6)
        self.mapping_index_entry.grid(row=0, column=1, padx=4, pady=4, sticky="w")

        ttk.Label(control_frame, text="Message:").grid(row=0, column=2, padx=4, pady=4, sticky="e")
        self.mapping_message_combo = ttk.Combobox(
            control_frame,
            textvariable=self.mapping_message_var,
            state="disabled",
            width=28,
        )
        self.mapping_message_combo.grid(row=0, column=3, padx=4, pady=4, sticky="ew")
        self.mapping_message_combo.bind("<<ComboboxSelected>>", self._on_mapping_message_selected)

        ttk.Label(control_frame, text="Signal:").grid(row=0, column=4, padx=4, pady=4, sticky="e")
        self.mapping_signal_combo = ttk.Combobox(
            control_frame,
            textvariable=self.mapping_signal_var,
            state="disabled",
            width=20,
        )
        self.mapping_signal_combo.grid(row=0, column=5, padx=4, pady=4, sticky="ew")

        ttk.Label(control_frame, text="Type:").grid(row=0, column=6, padx=4, pady=4, sticky="e")
        self.mapping_type_combo = ttk.Combobox(
            control_frame,
            textvariable=self.mapping_type_var,
            values=("float", "int"),
            state="disabled",
            width=10,
        )
        self.mapping_type_combo.grid(row=0, column=7, padx=4, pady=4, sticky="ew")

        self.add_mapping_button = ttk.Button(
            control_frame,
            text="Add / Update",
            command=self._add_or_update_mapping,
            state=tk.DISABLED,
        )
        self.add_mapping_button.grid(row=0, column=8, padx=4, pady=4)

        self.remove_mapping_button = ttk.Button(
            control_frame,
            text="Remove Selected",
            command=self._remove_selected_mapping,
            state=tk.DISABLED,
        )
        self.remove_mapping_button.grid(row=0, column=9, padx=4, pady=4)

        # Conditional rules frame
        condition_frame = ttk.LabelFrame(self.root, text="Conditional Rules")
        condition_frame.grid(row=4, column=0, sticky="nsew", padx=8, pady=4)
        condition_frame.grid_columnconfigure(0, weight=1)
        condition_frame.grid_rowconfigure(0, weight=1)

        condition_columns = ("signal", "operator", "compare", "target", "true", "false")
        self.condition_tree = ttk.Treeview(
            condition_frame,
            columns=condition_columns,
            show="headings",
            height=5,
            selectmode="browse",
        )
        for col, title, width, anchor in [
            ("signal", "Signal", 220, "w"),
            ("operator", "Op", 60, "center"),
            ("compare", "Compare", 100, "e"),
            ("target", "Target", 90, "center"),
            ("true", "True", 80, "center"),
            ("false", "False", 80, "center"),
        ]:
            self.condition_tree.heading(col, text=title)
            self.condition_tree.column(col, width=width, anchor=anchor)
        self.condition_tree.grid(row=0, column=0, sticky="nsew")
        self.condition_tree.bind("<<TreeviewSelect>>", self._on_condition_selected)

        condition_scroll = ttk.Scrollbar(condition_frame, orient="vertical", command=self.condition_tree.yview)
        condition_scroll.grid(row=0, column=1, sticky="ns")
        self.condition_tree.configure(yscrollcommand=condition_scroll.set)

        condition_controls = ttk.Frame(condition_frame)
        condition_controls.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=(4, 0))
        for col in range(0, 12):
            condition_controls.grid_columnconfigure(col, weight=1 if col % 2 == 1 else 0)

        self.condition_message_var = tk.StringVar()
        self.condition_signal_var = tk.StringVar()
        self.condition_operator_var = tk.StringVar(value="==")
        self.condition_compare_var = tk.StringVar(value="0")
        self.condition_target_type_var = tk.StringVar(value="int")
        self.condition_target_index_var = tk.StringVar(value="0")
        self.condition_true_value_var = tk.StringVar(value="1")
        self.condition_false_value_var = tk.StringVar(value="0")

        ttk.Label(condition_controls, text="Message:").grid(row=0, column=0, padx=4, pady=4, sticky="e")
        self.condition_message_combo = ttk.Combobox(
            condition_controls, textvariable=self.condition_message_var, state="disabled", width=26
        )
        self.condition_message_combo.grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        self.condition_message_combo.bind("<<ComboboxSelected>>", self._on_condition_message_selected)

        ttk.Label(condition_controls, text="Signal:").grid(row=0, column=2, padx=4, pady=4, sticky="e")
        self.condition_signal_combo = ttk.Combobox(
            condition_controls, textvariable=self.condition_signal_var, state="disabled", width=18
        )
        self.condition_signal_combo.grid(row=0, column=3, padx=4, pady=4, sticky="ew")

        ttk.Label(condition_controls, text="Operator:").grid(row=0, column=4, padx=4, pady=4, sticky="e")
        self.condition_operator_combo = ttk.Combobox(
            condition_controls,
            textvariable=self.condition_operator_var,
            state="readonly",
            values=("==", "!=", ">", "<", ">=", "<="),
            width=6,
        )
        self.condition_operator_combo.grid(row=0, column=5, padx=4, pady=4, sticky="w")

        ttk.Label(condition_controls, text="Compare:").grid(row=0, column=6, padx=4, pady=4, sticky="e")
        self.condition_compare_entry = ttk.Entry(condition_controls, textvariable=self.condition_compare_var, width=10)
        self.condition_compare_entry.grid(row=0, column=7, padx=4, pady=4, sticky="w")

        ttk.Label(condition_controls, text="Target:").grid(row=0, column=8, padx=4, pady=4, sticky="e")
        self.condition_target_type_combo = ttk.Combobox(
            condition_controls,
            textvariable=self.condition_target_type_var,
            state="readonly",
            values=("float", "int"),
            width=6,
        )
        self.condition_target_type_combo.grid(row=0, column=9, padx=4, pady=4, sticky="w")

        self.condition_target_index_entry = ttk.Entry(condition_controls, textvariable=self.condition_target_index_var, width=6)
        self.condition_target_index_entry.grid(row=0, column=10, padx=4, pady=4, sticky="w")

        ttk.Label(condition_controls, text="True/False:").grid(row=0, column=11, padx=4, pady=4, sticky="e")
        true_false_frame = ttk.Frame(condition_controls)
        true_false_frame.grid(row=0, column=12, padx=4, pady=4, sticky="ew")
        true_false_frame.grid_columnconfigure(1, weight=1)
        self.condition_true_entry = ttk.Entry(true_false_frame, textvariable=self.condition_true_value_var, width=6)
        self.condition_true_entry.grid(row=0, column=0, padx=(0, 4), pady=0)
        self.condition_false_entry = ttk.Entry(true_false_frame, textvariable=self.condition_false_value_var, width=6)
        self.condition_false_entry.grid(row=0, column=1, padx=(0, 4), pady=0)

        self.add_condition_button = ttk.Button(
            condition_controls,
            text="Add / Update",
            command=self._add_or_update_condition_rule,
            state=tk.DISABLED,
        )
        self.add_condition_button.grid(row=0, column=13, padx=4, pady=4)

        self.remove_condition_button = ttk.Button(
            condition_controls,
            text="Remove",
            command=self._remove_selected_condition_rule,
            state=tk.DISABLED,
        )
        self.remove_condition_button.grid(row=0, column=14, padx=4, pady=4)

        # Expression rules frame
        expression_frame = ttk.LabelFrame(self.root, text="Computed Expressions")
        expression_frame.grid(row=5, column=0, sticky="nsew", padx=8, pady=4)
        expression_frame.grid_columnconfigure(0, weight=1)
        expression_frame.grid_rowconfigure(0, weight=1)

        expression_columns = ("target", "expression")
        self.expression_tree = ttk.Treeview(
            expression_frame,
            columns=expression_columns,
            show="headings",
            height=4,
            selectmode="browse",
        )
        self.expression_tree.heading("target", text="Target")
        self.expression_tree.heading("expression", text="Expression")
        self.expression_tree.column("target", width=120, anchor="center")
        self.expression_tree.column("expression", width=500, anchor="w")
        self.expression_tree.grid(row=0, column=0, sticky="nsew")
        self.expression_tree.bind("<<TreeviewSelect>>", self._on_expression_selected)

        expression_scroll = ttk.Scrollbar(expression_frame, orient="vertical", command=self.expression_tree.yview)
        expression_scroll.grid(row=0, column=1, sticky="ns")
        self.expression_tree.configure(yscrollcommand=expression_scroll.set)

        expression_controls = ttk.Frame(expression_frame)
        expression_controls.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=(4, 0))
        expression_controls.grid_columnconfigure(3, weight=1)

        self.expression_target_type_var = tk.StringVar(value="float")
        self.expression_target_index_var = tk.StringVar(value="0")
        self.expression_text_var = tk.StringVar()

        ttk.Label(expression_controls, text="Target:").grid(row=0, column=0, padx=4, pady=4, sticky="e")
        self.expression_target_type_combo = ttk.Combobox(
            expression_controls,
            textvariable=self.expression_target_type_var,
            values=("float", "int"),
            state="readonly",
            width=6,
        )
        self.expression_target_type_combo.grid(row=0, column=1, padx=4, pady=4, sticky="w")
        self.expression_target_index_entry = ttk.Entry(expression_controls, textvariable=self.expression_target_index_var, width=6)
        self.expression_target_index_entry.grid(row=0, column=2, padx=4, pady=4, sticky="w")

        ttk.Label(expression_controls, text="Expression:").grid(row=0, column=3, padx=4, pady=4, sticky="e")
        self.expression_entry = ttk.Entry(expression_controls, textvariable=self.expression_text_var)
        self.expression_entry.grid(row=0, column=4, padx=4, pady=4, sticky="ew")

        self.add_expression_button = ttk.Button(
            expression_controls,
            text="Add / Update",
            command=self._add_or_update_expression_rule,
            state=tk.DISABLED,
        )
        self.add_expression_button.grid(row=0, column=5, padx=4, pady=4)

        self.remove_expression_button = ttk.Button(
            expression_controls,
            text="Remove",
            command=self._remove_selected_expression_rule,
            state=tk.DISABLED,
        )
        self.remove_expression_button.grid(row=0, column=6, padx=4, pady=4)

        ttk.Label(
            expression_controls,
            text="Use MessageName.SignalName (sanitized: non-alphanumerics -> '_'). Allowed ops: +, -, *, /, comparisons, ternary (a if cond else b), abs/min/max/round.",
        ).grid(row=1, column=0, columnspan=7, padx=4, pady=(0, 4), sticky="w")

        udp_frame = ttk.LabelFrame(self.root, text="UDP Target")
        udp_frame.grid(row=6, column=0, sticky="ew", padx=8, pady=4)
        udp_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(udp_frame, text="IP address:").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.udp_ip_var = tk.StringVar(value="127.0.0.1")
        self.udp_ip_entry = ttk.Entry(udp_frame, textvariable=self.udp_ip_var)
        self.udp_ip_entry.grid(row=0, column=1, padx=4, pady=4, sticky="ew")

        ttk.Label(udp_frame, text="Port:").grid(row=0, column=2, padx=4, pady=4, sticky="e")
        self.udp_port_var = tk.StringVar(value="5000")
        self.udp_port_entry = ttk.Entry(udp_frame, textvariable=self.udp_port_var, width=8)
        self.udp_port_entry.grid(row=0, column=3, padx=4, pady=4, sticky="w")

        button_frame = ttk.Frame(self.root)
        button_frame.grid(row=7, column=0, sticky="ew", padx=8, pady=4)
        button_frame.grid_columnconfigure(1, weight=1)

        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_bridge)
        self.start_button.grid(row=0, column=0, padx=4, pady=4)

        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_bridge, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1, padx=4, pady=4)

        ttk.Button(button_frame, text="Clear Log", command=self.clear_log).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(button_frame, text="Load Config", command=self._browse_config).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(button_frame, text="Save Config", command=self._save_config).grid(row=0, column=4, padx=4, pady=4)

        log_frame = ttk.LabelFrame(self.root, text="Received Frames")
        log_frame.grid(row=8, column=0, sticky="nsew", padx=8, pady=4)
        self.root.grid_rowconfigure(8, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=18,
            width=120,
            state=tk.DISABLED,
            font=("Consolas", 10),
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")

        payload_frame = ttk.LabelFrame(self.root, text="Last UDP Payload")
        payload_frame.grid(row=9, column=0, sticky="ew", padx=8, pady=4)
        payload_frame.grid_columnconfigure(0, weight=1)
        ttk.Label(
            payload_frame,
            textvariable=self.last_udp_payload_var,
            anchor="w",
            justify="left",
            wraplength=780,
        ).grid(row=0, column=0, sticky="ew", padx=6, pady=4)

        status_frame = ttk.Frame(self.root)
        status_frame.grid(row=10, column=0, sticky="ew", padx=8, pady=(0, 8))
        status_frame.grid_columnconfigure(0, weight=1)
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(status_frame, textvariable=self.status_var).grid(row=0, column=0, sticky="w")

        self._update_fd_state()
        if cantools is None:
            self.dbc_status_var.set("cantools not available. Install cantools to decode DBC signals.")
            self.load_dbc_button.configure(state=tk.DISABLED)
        self._refresh_signal_control_state()

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
        self._refresh_signal_control_state()

    def _refresh_signal_control_state(self) -> None:
        has_dbc = self.dbc_database is not None and cantools is not None
        message_state = "readonly" if has_dbc else "disabled"
        signal_state = "readonly" if has_dbc and self.mapping_signal_combo["values"] else "disabled"
        button_state = tk.NORMAL if has_dbc else tk.DISABLED
        type_state = "readonly" if has_dbc else "disabled"
        self.mapping_message_combo.configure(state=message_state)
        self.mapping_signal_combo.configure(state=signal_state)
        self.mapping_type_combo.configure(state=type_state)
        self.add_mapping_button.configure(state=button_state)
        remove_state = tk.NORMAL if has_dbc and self.signal_tree_items else tk.DISABLED
        self.remove_mapping_button.configure(state=remove_state)
        if not has_dbc:
            self.mapping_type_var.set("float")

        # Condition controls
        condition_signal_state = "readonly" if has_dbc and self.condition_signal_combo["values"] else "disabled"
        self.condition_message_combo.configure(state=message_state)
        self.condition_signal_combo.configure(state=condition_signal_state)
        self.condition_operator_combo.configure(state="readonly" if has_dbc else "disabled")
        self.condition_target_type_combo.configure(state="readonly" if has_dbc else "disabled")
        self.condition_target_index_entry.configure(state=tk.NORMAL if has_dbc else tk.DISABLED)
        self.condition_compare_entry.configure(state=tk.NORMAL if has_dbc else tk.DISABLED)
        self.condition_true_entry.configure(state=tk.NORMAL if has_dbc else tk.DISABLED)
        self.condition_false_entry.configure(state=tk.NORMAL if has_dbc else tk.DISABLED)
        self.add_condition_button.configure(state=button_state)
        self.remove_condition_button.configure(
            state=tk.NORMAL if self.condition_tree_items and has_dbc else tk.DISABLED
        )
        if not has_dbc:
            self.condition_target_type_var.set("int")

        # Expression controls
        self.expression_target_type_combo.configure(state="readonly" if has_dbc else "disabled")
        self.expression_target_index_entry.configure(state=tk.NORMAL if has_dbc else tk.DISABLED)
        self.expression_entry.configure(state=tk.NORMAL if has_dbc else tk.DISABLED)
        self.add_expression_button.configure(state=button_state)
        self.remove_expression_button.configure(
            state=tk.NORMAL if self.expression_tree_items and has_dbc else tk.DISABLED
        )
        if not has_dbc:
            self.expression_target_type_var.set("float")

    def _browse_dbc(self) -> None:
        file_path = filedialog.askopenfilename(
            title="Select DBC file",
            filetypes=[("DBC files", "*.dbc"), ("All files", "*.*")],
        )
        if file_path:
            self._load_dbc(file_path)

    def _load_dbc(self, path: Optional[str] = None) -> bool:
        if cantools is None:
            messagebox.showerror("DBC error", "cantools package is required to load DBC files.")
            return False

        dbc_path = path or self.dbc_path_var.get()
        if not dbc_path:
            messagebox.showerror("DBC error", "Please choose a DBC file.")
            return False

        file_path = Path(dbc_path).expanduser()
        if not file_path.exists():
            messagebox.showerror("DBC error", f"DBC file not found: {file_path}")
            return False

        try:
            database = cantools.database.load_file(str(file_path))
        except Exception as exc:  # pragma: no cover - user feedback path
            messagebox.showerror("DBC load failed", str(exc))
            self.dbc_status_var.set(f"Failed to load DBC: {exc}")
            return False

        self.dbc_database = database
        self.dbc_path_var.set(str(file_path))
        self.dbc_messages_by_id.clear()
        self.message_display_to_key.clear()
        self.message_key_to_display.clear()
        self.message_names_by_key.clear()
        with self.signal_lock:
            self.latest_signals.clear()

        message_entries: List[Tuple[str, Tuple[int, bool]]] = []
        for message in sorted(database.messages, key=lambda m: (m.frame_id, m.is_extended_frame, m.name)):
            key = (message.frame_id, bool(message.is_extended_frame))
            self.dbc_messages_by_id[key] = message
            self.message_names_by_key[key] = message.name
            display = self._format_message_display(message)
            self.message_key_to_display[key] = display
            message_entries.append((display, key))

        self._clear_signal_mappings()
        self._clear_condition_rules()
        self._clear_expression_rules()
        message_values = [entry[0] for entry in message_entries]
        self.mapping_message_combo.configure(values=message_values)
        self.mapping_message_combo.set("")
        self.mapping_signal_combo.configure(values=())
        self.mapping_signal_combo.set("")
        for display, key in message_entries:
            self.message_display_to_key[display] = key

        self.condition_message_combo.configure(values=message_values)
        self.condition_message_combo.set("")
        self.condition_signal_combo.configure(values=())
        self.condition_signal_combo.set("")
        self.dbc_status_var.set(f"Loaded {file_path.name} ({len(message_entries)} messages).")
        self.status_var.set(f"DBC loaded: {file_path}")
        self._refresh_signal_control_state()
        return True

    def _format_message_display(self, message: Any) -> str:
        display = f"{message.name} (0x{message.frame_id:X})"
        if getattr(message, "is_extended_frame", False):
            display += " EXT"
        return display

    def _get_message_display(self, message_key: Tuple[int, bool], message_obj: Any) -> str:
        display = self.message_key_to_display.get(message_key)
        if display is None:
            display = self._format_message_display(message_obj)
            self.message_key_to_display[message_key] = display
            self.message_display_to_key[display] = message_key
        return display

    def _store_signal_mapping(
        self,
        index: int,
        value_type: str,
        message_key: Tuple[int, bool],
        message_obj: Any,
        signal_name: str,
    ) -> str:
        normalized_type = "int" if value_type == "int" else "float"
        mapping_key = (index, normalized_type)
        message_display = self._get_message_display(message_key, message_obj)
        with self.signal_lock:
            self.signal_mappings[mapping_key] = {
                "key": message_key,
                "message": message_obj.name,
                "signal": signal_name,
                "value_type": normalized_type,
            }
            # reset cached value so newly configured mapping updates on first frame
            self.signal_values.pop(index, None)

        display_index = f"{normalized_type} {index}"
        row_values = (
            display_index,
            message_display,
            signal_name,
            normalized_type,
            "-",
        )
        item_id = self.signal_tree_items.get(mapping_key)
        if item_id:
            self.signal_tree.item(item_id, values=row_values)
        else:
            item_id = self.signal_tree.insert("", "end", values=row_values)
            self.signal_tree_items[mapping_key] = item_id
        self.remove_mapping_button.configure(state=tk.NORMAL)
        return normalized_type

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

    def _browse_config(self) -> None:
        initial_dir = None
        if self.last_config_path and self.last_config_path.parent.exists():
            initial_dir = str(self.last_config_path.parent)
        else:
            for path in self.default_config_paths:
                if path.exists():
                    initial_dir = str(path.parent)
                    break
            if initial_dir is None:
                try:
                    initial_dir = str(self.default_config_paths[0].parent)
                except IndexError:
                    initial_dir = str(Path.cwd())

        file_path = filedialog.askopenfilename(
            title="Select config file",
            filetypes=[
                ("Config files", "*.json *.conf *.config *.cofig"),
                ("JSON files", "*.json"),
                ("Conf files", "*.conf"),
                ("Config files", "*.config *.cofig"),
                ("All files", "*.*"),
            ],
            initialdir=initial_dir,
        )
        if file_path:
            self._load_config_from_path(Path(file_path))

    def _load_config_from_path(self, path: Path, display_dialogs: bool = True) -> bool:
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
        config_dir = path.parent
        warnings: List[str] = []
        applied: List[str] = []

        def parse_int(value: Any) -> Optional[int]:
            if isinstance(value, int):
                return value
            if isinstance(value, str):
                text = value.strip()
                if not text:
                    return None
                base = 10
                lower = text.lower()
                if lower.startswith("-0x") or lower.startswith("0x"):
                    base = 16
                return int(text, base)
            return None

        dbc_entry = data.get("dbc_path")
        if isinstance(dbc_entry, str) and dbc_entry.strip():
            resolved_path = Path(dbc_entry.strip()).expanduser()
            if not resolved_path.is_absolute():
                resolved_path = (config_dir / resolved_path).resolve()
            if not resolved_path.exists():
                warnings.append(f"DBC file not found: {resolved_path}")
            elif cantools is None:
                warnings.append("cantools is not installed; skipping DBC load from config.")
            else:
                load_success = self._load_dbc(str(resolved_path))
                if load_success:
                    applied.append("dbc")
                else:
                    warnings.append(f"Failed to load DBC file: {resolved_path}")

        udp_section = data.get("udp")
        udp_ip: Optional[str] = None
        udp_port_raw: Any = None
        if isinstance(udp_section, dict):
            udp_ip = udp_section.get("ip") or udp_section.get("address")
            udp_port_raw = udp_section.get("port")
        else:
            udp_ip = data.get("udp_ip")
            udp_port_raw = data.get("udp_port")

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

        def resolve_message(entry: Dict[str, Any], context: str) -> Optional[Tuple[Tuple[int, bool], Any, str]]:
            message_name = entry.get("message") or entry.get("message_name")
            if message_name is not None:
                message_name = str(message_name)
            frame_id_raw = entry.get("frame_id")
            is_extended_value = entry.get("is_extended")
            if isinstance(is_extended_value, bool):
                is_extended: Optional[bool] = is_extended_value
            elif isinstance(is_extended_value, str):
                lowered = is_extended_value.strip().lower()
                if lowered in ("true", "1", "yes", "y"):
                    is_extended = True
                elif lowered in ("false", "0", "no", "n"):
                    is_extended = False
                else:
                    is_extended = None
            else:
                is_extended = None

            frame_id: Optional[int] = None
            if frame_id_raw is not None:
                try:
                    frame_id = parse_int(frame_id_raw)
                except (TypeError, ValueError):
                    warnings.append(f"{context}: invalid frame_id value: {frame_id_raw}")
                    return None
                if frame_id is None:
                    warnings.append(f"{context}: frame_id is empty.")
                    return None

            message_key: Optional[Tuple[int, bool]] = None
            if frame_id is not None:
                for key in self.dbc_messages_by_id:
                    if key[0] == frame_id and (is_extended is None or key[1] == is_extended):
                        if message_name is None or self.message_names_by_key.get(key) == message_name:
                            message_key = key
                            break
            if message_key is None and message_name:
                for key, name in self.message_names_by_key.items():
                    if name == message_name and (frame_id is None or key[0] == frame_id):
                        message_key = key
                        break

            if message_key is None:
                warnings.append(f"{context}: message not found.")
                return None

            message_obj = self.dbc_messages_by_id.get(message_key)
            if message_obj is None:
                warnings.append(f"{context}: message data missing.")
                return None

            message_display = self._get_message_display(message_key, message_obj)
            return message_key, message_obj, message_display

        mappings_data = data.get("mappings")
        if mappings_data is None:
            mappings_data = data.get("signal_mappings")

        successful_mappings = 0
        if mappings_data is not None:
            if self.dbc_database is None:
                warnings.append("No DBC loaded; skipping signal mappings from config.")
            elif isinstance(mappings_data, list):
                self._clear_signal_mappings()

                for entry in mappings_data:
                    if not isinstance(entry, dict):
                        warnings.append(f"Invalid mapping entry (expected object): {entry}")
                        continue

                    index_raw = entry.get("index")
                    try:
                        index_parsed = parse_int(index_raw)
                    except (TypeError, ValueError):
                        warnings.append(f"Invalid mapping index: {index_raw}")
                        continue
                    if index_parsed is None:
                        warnings.append("Mapping entry is missing an index value.")
                        continue
                    if index_parsed < 0:
                        warnings.append(f"Mapping index must be non-negative: {index_parsed}")
                        continue
                    index = index_parsed

                    value_type = str(entry.get("type", "float")).strip().lower()
                    if value_type not in ("float", "int"):
                        warnings.append(f"Invalid mapping type for index {index}: {value_type}")
                        value_type = "float"

                    resolved = resolve_message(entry, f"Mapping index {index}")
                    if not resolved:
                        continue
                    message_key, message_obj, _message_display = resolved

                    signal_name = entry.get("signal") or entry.get("signal_name")
                    if not signal_name:
                        warnings.append(f"Mapping index {index}: signal not provided.")
                        continue
                    signal_name = str(signal_name)

                    if signal_name not in [signal.name for signal in message_obj.signals]:
                        warnings.append(
                            f"Signal '{signal_name}' not found in message '{message_obj.name}' for mapping index {index}."
                        )
                        continue

                    self._store_signal_mapping(index, value_type, message_key, message_obj, signal_name)
                    successful_mappings += 1

                self._refresh_signal_control_state()
            else:
                warnings.append("Signal mappings must be a list of mapping objects.")

        if successful_mappings:
            applied.append(f"{successful_mappings} mapping(s)")

        condition_entries = data.get("conditions") or data.get("condition_rules")
        loaded_conditions = 0
        if condition_entries is not None:
            if self.dbc_database is None:
                warnings.append("No DBC loaded; skipping condition rules from config.")
            elif isinstance(condition_entries, list):
                self._clear_condition_rules()
                for idx, entry in enumerate(condition_entries, start=1):
                    if not isinstance(entry, dict):
                        warnings.append(f"Condition rule #{idx}: invalid entry (expected object).")
                        continue

                    resolved = resolve_message(entry, f"Condition rule #{idx}")
                    if not resolved:
                        continue
                    message_key, message_obj, message_display = resolved

                    signal_name = entry.get("signal") or entry.get("signal_name")
                    if not signal_name:
                        warnings.append(f"Condition rule #{idx}: signal not provided.")
                        continue
                    signal_name = str(signal_name)
                    if signal_name not in [signal.name for signal in message_obj.signals]:
                        warnings.append(
                            f"Condition rule #{idx}: signal '{signal_name}' not found in message '{message_obj.name}'."
                        )
                        continue

                    operator = str(entry.get("operator") or entry.get("op") or "").strip()
                    if operator not in ("==", "!=", ">", "<", ">=", "<="):
                        warnings.append(f"Condition rule #{idx}: invalid operator '{operator}'.")
                        continue

                    compare_raw = entry.get("compare")
                    if compare_raw is None:
                        compare_raw = entry.get("compare_value")
                    try:
                        compare_value = float(compare_raw)
                    except (TypeError, ValueError):
                        warnings.append(f"Condition rule #{idx}: invalid compare value {compare_raw}.")
                        continue

                    target_section = entry.get("target") if isinstance(entry.get("target"), dict) else {}
                    target_type_raw = entry.get("target_type")
                    if target_type_raw is None:
                        target_type_raw = target_section.get("type")
                    target_type = str(target_type_raw or "int").strip().lower()
                    if target_type not in ("float", "int"):
                        warnings.append(f"Condition rule #{idx}: invalid target type '{target_type}'.")
                        continue

                    target_index_raw = entry.get("target_index")
                    if target_index_raw is None:
                        target_index_raw = target_section.get("index")
                    try:
                        target_index = parse_int(target_index_raw)
                    except (TypeError, ValueError):
                        warnings.append(f"Condition rule #{idx}: invalid target index {target_index_raw}.")
                        continue
                    if target_index is None:
                        warnings.append(f"Condition rule #{idx}: target index is missing.")
                        continue
                    if target_index < 0:
                        warnings.append(f"Condition rule #{idx}: target index must be non-negative.")
                        continue

                    true_raw = entry.get("true_value")
                    if true_raw is None:
                        true_raw = entry.get("true")
                    try:
                        true_value = float(true_raw)
                    except (TypeError, ValueError):
                        warnings.append(f"Condition rule #{idx}: invalid true value {true_raw}.")
                        continue

                    false_raw = entry.get("false_value")
                    if false_raw is None:
                        false_raw = entry.get("false")
                    if false_raw is None or false_raw == "":
                        false_value = None
                    else:
                        try:
                            false_value = float(false_raw)
                        except (TypeError, ValueError):
                            warnings.append(f"Condition rule #{idx}: invalid false value {false_raw}.")
                            continue

                    signal_display = f"{message_display}.{signal_name}"
                    target_display = f"{target_type} {target_index}"
                    self.condition_rule_counter += 1
                    rule = {
                        "id": self.condition_rule_counter,
                        "message_key": message_key,
                        "message_display": message_display,
                        "signal": signal_name,
                        "operator": operator,
                        "compare_value": compare_value,
                        "target_type": target_type,
                        "target_index": target_index,
                        "true_value": true_value,
                        "false_value": false_value,
                    }
                    tree_values = (
                        signal_display,
                        operator,
                        compare_value,
                        target_display,
                        true_value,
                        "" if false_value is None else false_value,
                    )
                    tree_id = self.condition_tree.insert("", "end", values=tree_values)
                    rule["tree_id"] = tree_id
                    self.condition_tree_items[tree_id] = rule
                    self.condition_rules.append(rule)
                    loaded_conditions += 1

                if loaded_conditions:
                    applied.append(f"{loaded_conditions} condition rule(s)")
                    self.remove_condition_button.configure(state=tk.NORMAL)
                    self._refresh_signal_control_state()
            else:
                warnings.append("Condition rules must be a list of rule objects.")

        expression_entries = data.get("expressions") or data.get("expression_rules")
        loaded_expressions = 0
        if expression_entries is not None:
            if self.dbc_database is None:
                warnings.append("No DBC loaded; skipping expression rules from config.")
            elif isinstance(expression_entries, list):
                self._clear_expression_rules()
                for idx, entry in enumerate(expression_entries, start=1):
                    if not isinstance(entry, dict):
                        warnings.append(f"Expression rule #{idx}: invalid entry (expected object).")
                        continue

                    target_section = entry.get("target") if isinstance(entry.get("target"), dict) else {}
                    target_type_raw = entry.get("target_type")
                    if target_type_raw is None:
                        target_type_raw = target_section.get("type")
                    target_type = str(target_type_raw or "float").strip().lower()
                    if target_type not in ("float", "int"):
                        warnings.append(f"Expression rule #{idx}: invalid target type '{target_type}'.")
                        continue

                    target_index_raw = entry.get("target_index")
                    if target_index_raw is None:
                        target_index_raw = target_section.get("index")
                    try:
                        target_index = parse_int(target_index_raw)
                    except (TypeError, ValueError):
                        warnings.append(f"Expression rule #{idx}: invalid target index {target_index_raw}.")
                        continue
                    if target_index is None:
                        warnings.append(f"Expression rule #{idx}: target index is missing.")
                        continue
                    if target_index < 0:
                        warnings.append(f"Expression rule #{idx}: target index must be non-negative.")
                        continue

                    expression_text = entry.get("expression") or entry.get("expr")
                    if not isinstance(expression_text, str):
                        warnings.append(f"Expression rule #{idx}: expression must be a string.")
                        continue
                    expression_text = expression_text.strip()
                    if not expression_text:
                        warnings.append(f"Expression rule #{idx}: expression cannot be empty.")
                        continue

                    self.expression_rule_counter += 1
                    rule = {
                        "id": self.expression_rule_counter,
                        "target_type": target_type,
                        "target_index": target_index,
                        "expression": expression_text,
                    }
                    target_display = f"{target_type} {target_index}"
                    tree_id = self.expression_tree.insert("", "end", values=(target_display, expression_text))
                    rule["tree_id"] = tree_id
                    self.expression_tree_items[tree_id] = rule
                    self.expression_rules.append(rule)
                    loaded_expressions += 1

                if loaded_expressions:
                    applied.append(f"{loaded_expressions} expression rule(s)")
                    self.remove_expression_button.configure(state=tk.NORMAL)
                    self._refresh_signal_control_state()
            else:
                warnings.append("Expression rules must be a list of rule objects.")

        if applied:
            summary = "; ".join(dict.fromkeys(applied))  # preserve order, remove duplicates
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

    def _build_config_snapshot(self) -> Optional[Dict[str, Any]]:
        config: Dict[str, Any] = {}

        dbc_path = self.dbc_path_var.get().strip()
        if dbc_path:
            config["dbc_path"] = dbc_path

        udp_ip = self.udp_ip_var.get().strip()
        udp_port_str = self.udp_port_var.get().strip()
        udp_settings: Dict[str, Any] = {}
        if udp_ip:
            udp_settings["ip"] = udp_ip
        if udp_port_str:
            try:
                udp_port = int(udp_port_str, 0)
            except ValueError:
                messagebox.showerror("Config save", "UDP port must be an integer before saving a config file.")
                return None
            if not (0 <= udp_port <= 65535):
                messagebox.showerror("Config save", "UDP port must be between 0 and 65535 before saving.")
                return None
            udp_settings["port"] = udp_port
        if udp_settings:
            config["udp"] = udp_settings

        mapping_entries: List[Dict[str, Any]] = []
        with self.signal_lock:
            mapping_items = list(self.signal_mappings.items())
        for (index, value_type), mapping in sorted(mapping_items, key=lambda item: (item[0][0], item[0][1])):
            signal_name = str(mapping.get("signal", "")).strip()
            if not signal_name:
                continue
            entry: Dict[str, Any] = {
                "index": index,
                "type": value_type,
                "signal": signal_name,
            }
            message_name = mapping.get("message")
            if message_name:
                entry["message"] = message_name
            message_key = mapping.get("key")
            if isinstance(message_key, tuple) and len(message_key) == 2:
                frame_id, is_extended = message_key
                try:
                    frame_id_int = int(frame_id)
                except (TypeError, ValueError):
                    frame_id_int = None
                if frame_id_int is not None:
                    entry["frame_id"] = f"0x{frame_id_int:X}"
                entry["is_extended"] = bool(is_extended)
                if not entry.get("message"):
                    entry["message"] = self.message_names_by_key.get(message_key)
            mapping_entries.append(entry)
        if mapping_entries:
            config["mappings"] = mapping_entries

        condition_entries: List[Dict[str, Any]] = []
        for rule in self.condition_rules:
            message_key = rule.get("message_key")
            signal_name = rule.get("signal")
            operator = rule.get("operator")
            compare_value = rule.get("compare_value")
            target_type = rule.get("target_type")
            target_index = rule.get("target_index")
            true_value = rule.get("true_value")
            if (
                message_key is None
                or signal_name is None
                or operator is None
                or compare_value is None
                or target_type is None
                or target_index is None
                or true_value is None
            ):
                continue

            entry = {
                "signal": signal_name,
                "operator": operator,
                "compare": compare_value,
                "target_type": target_type,
                "target_index": target_index,
                "true_value": true_value,
            }

            false_value = rule.get("false_value")
            if false_value is not None:
                entry["false_value"] = false_value

            if isinstance(message_key, tuple) and len(message_key) == 2:
                frame_id, is_extended = message_key
                try:
                    entry["frame_id"] = f"0x{int(frame_id):X}"
                except (TypeError, ValueError):
                    pass
                entry["is_extended"] = bool(is_extended)
                message_name = self.message_names_by_key.get(message_key)
                if message_name:
                    entry["message"] = message_name
            else:
                message_name = self.message_names_by_key.get(message_key) or rule.get("message_display")
                if message_name:
                    entry["message"] = message_name

            condition_entries.append(entry)
        if condition_entries:
            config["conditions"] = condition_entries

        expression_entries: List[Dict[str, Any]] = []
        for rule in self.expression_rules:
            target_type = rule.get("target_type")
            target_index = rule.get("target_index")
            expression_text = rule.get("expression")
            if target_type not in ("float", "int") or target_index is None or expression_text is None:
                continue
            entry = {
                "target_type": target_type,
                "target_index": target_index,
                "expression": expression_text,
            }
            expression_entries.append(entry)
        if expression_entries:
            config["expressions"] = expression_entries

        return config

    def _save_config(self) -> None:
        snapshot = self._build_config_snapshot()
        if snapshot is None:
            return

        initial_dir: Optional[str] = None
        initial_file: Optional[str] = None
        if self.last_config_path and self.last_config_path.parent.exists():
            initial_dir = str(self.last_config_path.parent)
            initial_file = self.last_config_path.name
        else:
            for path in self.default_config_paths:
                if path.exists():
                    initial_dir = str(path.parent)
                    initial_file = path.name
                    break
            if initial_dir is None and self.default_config_paths:
                initial_dir = str(self.default_config_paths[0].parent)
                initial_file = self.default_config_paths[0].name

        file_path = filedialog.asksaveasfilename(
            title="Save config file",
            defaultextension=".config",
            filetypes=[
                ("Config files", "*.config *.cofig *.json *.conf"),
                ("Config files (*.config)", "*.config *.cofig"),
                ("JSON files", "*.json"),
                ("Conf files", "*.conf"),
                ("All files", "*.*"),
            ],
            initialdir=initial_dir,
            initialfile=initial_file,
        )
        if not file_path:
            return

        target_path = Path(file_path).expanduser()
        try:
            target_path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            messagebox.showerror("Config save", f"Unable to create directory for config file: {exc}")
            return

        try:
            text = json.dumps(snapshot, indent=2, sort_keys=True, ensure_ascii=True)
            target_path.write_text(text + "\n", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("Config save", f"Failed to save config file: {exc}")
            return

        self.last_config_path = target_path
        self.status_var.set(f"Config saved to {target_path}")
        messagebox.showinfo("Config save", f"Config saved to:\n{target_path}")

    def _clear_signal_mappings(self) -> None:
        with self.signal_lock:
            self.signal_mappings.clear()
            self.signal_values.clear()
        for item in self.signal_tree.get_children():
            self.signal_tree.delete(item)
        self.signal_tree_items.clear()
        self.mapping_index_var.set("")
        self.mapping_signal_var.set("")
        self.mapping_message_var.set("")
        self.mapping_type_var.set("float")
        self.remove_mapping_button.configure(state=tk.DISABLED)
        self._refresh_signal_control_state()

    def _clear_condition_rules(self) -> None:
        self.condition_rules.clear()
        self.condition_tree_items.clear()
        for item in self.condition_tree.get_children():
            self.condition_tree.delete(item)
        self.condition_rule_counter = 0
        self.remove_condition_button.configure(state=tk.DISABLED)

    def _clear_expression_rules(self) -> None:
        self.expression_rules.clear()
        self.expression_tree_items.clear()
        for item in self.expression_tree.get_children():
            self.expression_tree.delete(item)
        self.expression_rule_counter = 0
        self.remove_expression_button.configure(state=tk.DISABLED)

    def _on_mapping_message_selected(self, _event: Any) -> None:
        display = self.mapping_message_var.get()
        key = self.message_display_to_key.get(display)
        if not key:
            self.mapping_signal_combo.configure(values=(), state="disabled")
            self.mapping_signal_var.set("")
            return

        message = self.dbc_messages_by_id.get(key)
        if not message:
            self.mapping_signal_combo.configure(values=(), state="disabled")
            self.mapping_signal_var.set("")
            return

        signals = [signal.name for signal in message.signals]
        if signals:
            self.mapping_signal_combo.configure(values=signals, state="readonly")
            if self.mapping_signal_var.get() not in signals:
                self.mapping_signal_var.set(signals[0])
        else:
            self.mapping_signal_combo.configure(values=(), state="disabled")
            self.mapping_signal_var.set("")

    def _add_or_update_mapping(self) -> None:
        if self.dbc_database is None or cantools is None:
            messagebox.showerror("Signal mapping", "Load a DBC file before adding mappings.")
            return

        index_str = self.mapping_index_var.get().strip()
        if not index_str:
            messagebox.showerror("Signal mapping", "Provide a UDP index (e.g. 0).")
            return

        try:
            index = int(index_str)
        except ValueError:
            messagebox.showerror("Signal mapping", "UDP index must be an integer.")
            return

        if index < 0:
            messagebox.showerror("Signal mapping", "UDP index must be non-negative.")
            return

        message_display = self.mapping_message_var.get()
        key = self.message_display_to_key.get(message_display)
        if not key:
            messagebox.showerror("Signal mapping", "Select a DBC message.")
            return

        message_obj = self.dbc_messages_by_id.get(key)
        if message_obj is None:
            messagebox.showerror("Signal mapping", "Selected message is not available.")
            return

        signal_name = self.mapping_signal_var.get().strip()
        if not signal_name:
            messagebox.showerror("Signal mapping", "Select a signal.")
            return

        if signal_name not in [signal.name for signal in message_obj.signals]:
            messagebox.showerror("Signal mapping", f"Signal '{signal_name}' not found in message '{message_obj.name}'.")
            return

        selected_type = self.mapping_type_var.get().strip().lower()
        if selected_type not in ("float", "int"):
            selected_type = "float"
            self.mapping_type_var.set("float")

        normalized_type = self._store_signal_mapping(index, selected_type, key, message_obj, signal_name)
        self.status_var.set(f"Mapping set: UDP {normalized_type} {index} -> {message_obj.name}.{signal_name}")
        self._refresh_signal_control_state()

    def _remove_selected_mapping(self) -> None:
        selection = self.signal_tree.selection()
        if not selection:
            return

        for item_id in selection:
            values = self.signal_tree.item(item_id, "values")
            if not values:
                continue
            display_index = str(values[0])
            parts = display_index.split(maxsplit=1)
            if len(parts) == 2:
                value_type, index_str = parts[0].lower(), parts[1]
            else:
                value_type = str(values[3]).lower()
                index_str = str(values[0])
            try:
                index = int(index_str)
            except (TypeError, ValueError):
                continue
            mapping_key = (index, value_type)
            with self.signal_lock:
                self.signal_mappings.pop(mapping_key, None)
                if not any(k[0] == index for k in self.signal_mappings):
                    self.signal_values.pop(index, None)
            self.signal_tree.delete(item_id)
            self.signal_tree_items.pop(mapping_key, None)

        if not self.signal_tree_items:
            self.remove_mapping_button.configure(state=tk.DISABLED)
        self.status_var.set("Mapping removed.")
        self._refresh_signal_control_state()

    def _on_signal_selected(self, _event: Any) -> None:
        selection = self.signal_tree.selection()
        if not selection:
            if not self.signal_tree_items:
                self.remove_mapping_button.configure(state=tk.DISABLED)
            return

        self.remove_mapping_button.configure(state=tk.NORMAL)
        item_id = selection[0]
        values = self.signal_tree.item(item_id, "values")
        if not values:
            return

        display_index, message_display, signal_name, value_type, _value = values
        display_index = str(display_index)
        parts = display_index.split(maxsplit=1)
        if len(parts) == 2:
            value_type = parts[0].lower()
            index_str = parts[1]
        else:
            index_str = str(display_index)
            value_type = str(value_type).lower()
        self.mapping_index_var.set(index_str)
        self.mapping_message_var.set(message_display)
        self.mapping_signal_var.set(signal_name)
        self.mapping_message_combo.set(message_display)
        self._on_mapping_message_selected(None)
        self.mapping_signal_combo.set(signal_name)
        if value_type not in ("float", "int"):
            value_type = "float"
        self.mapping_type_var.set(value_type)
        self.mapping_type_combo.set(value_type)

    def _on_condition_message_selected(self, _event: Any) -> None:
        display = self.condition_message_var.get()
        key = self.message_display_to_key.get(display)
        if not key:
            self.condition_signal_combo.configure(values=(), state="disabled")
            self.condition_signal_var.set("")
            return

        message = self.dbc_messages_by_id.get(key)
        if not message:
            self.condition_signal_combo.configure(values=(), state="disabled")
            self.condition_signal_var.set("")
            return

        signals = [signal.name for signal in message.signals]
        if signals:
            self.condition_signal_combo.configure(values=signals, state="readonly")
            if self.condition_signal_var.get() not in signals:
                self.condition_signal_var.set(signals[0])
        else:
            self.condition_signal_combo.configure(values=(), state="disabled")
            self.condition_signal_var.set("")

    def _parse_float(self, value: str, field_name: str) -> Optional[float]:
        try:
            return float(value)
        except ValueError:
            messagebox.showerror("Invalid value", f"{field_name} must be a number.")
            return None

    def _parse_optional_float(self, value: str) -> Optional[float]:
        value = value.strip()
        if value == "":
            return None
        try:
            return float(value)
        except ValueError:
            messagebox.showerror("Invalid value", "False case value must be numeric or blank.")
            return None

    def _add_or_update_condition_rule(self) -> None:
        if self.dbc_database is None or cantools is None:
            messagebox.showerror("Conditional rule", "Load a DBC file before adding rules.")
            return

        message_display = self.condition_message_var.get()
        key = self.message_display_to_key.get(message_display)
        if not key:
            messagebox.showerror("Conditional rule", "Select a message.")
            return
        message_obj = self.dbc_messages_by_id.get(key)
        if message_obj is None:
            messagebox.showerror("Conditional rule", "Selected message is not available.")
            return

        signal_name = self.condition_signal_var.get().strip()
        if not signal_name:
            messagebox.showerror("Conditional rule", "Select a signal.")
            return
        if signal_name not in [signal.name for signal in message_obj.signals]:
            messagebox.showerror("Conditional rule", f"Signal '{signal_name}' not found in message '{message_obj.name}'.")
            return

        operator = self.condition_operator_var.get().strip()
        if operator not in ("==", "!=", ">", "<", ">=", "<="):
            messagebox.showerror("Conditional rule", "Operator is invalid.")
            return

        compare_value = self._parse_float(self.condition_compare_var.get().strip(), "Compare value")
        if compare_value is None:
            return

        target_type = self.condition_target_type_var.get().strip().lower()
        if target_type not in ("float", "int"):
            messagebox.showerror("Conditional rule", "Target type must be float or int.")
            return

        try:
            target_index = int(self.condition_target_index_var.get().strip())
        except ValueError:
            messagebox.showerror("Conditional rule", "Target index must be an integer.")
            return
        if target_index < 0:
            messagebox.showerror("Conditional rule", "Target index must be non-negative.")
            return

        true_value = self._parse_float(self.condition_true_value_var.get().strip(), "True value")
        if true_value is None:
            return
        false_value = self._parse_optional_float(self.condition_false_value_var.get())
        target_display = f"{target_type} {target_index}"
        signal_display = f"{message_display}.{signal_name}"

        selected = self.condition_tree.selection()
        if selected:
            tree_id = selected[0]
            rule = self.condition_tree_items.get(tree_id)
        else:
            tree_id = None
            rule = None

        if rule is None:
            self.condition_rule_counter += 1
            rule = {"id": self.condition_rule_counter}
            tree_id = self.condition_tree.insert(
                "", "end", values=(signal_display, operator, compare_value, target_display, true_value, false_value if false_value is not None else "")
            )
            self.condition_tree_items[tree_id] = rule
            self.condition_rules.append(rule)
        else:
            self.condition_tree.item(
                tree_id,
                values=(signal_display, operator, compare_value, target_display, true_value, false_value if false_value is not None else ""),
            )

        rule.update(
            {
                "message_key": key,
                "message_display": message_display,
                "signal": signal_name,
                "operator": operator,
                "compare_value": compare_value,
                "target_type": target_type,
                "target_index": target_index,
                "true_value": true_value,
                "false_value": false_value,
                "tree_id": tree_id,
            }
        )

        self.remove_condition_button.configure(state=tk.NORMAL)
        self.status_var.set(
            f"Condition rule set: if {signal_display} {operator} {compare_value} then UDP {target_display} <- {true_value}"
        )
        self._refresh_signal_control_state()

    def _remove_selected_condition_rule(self) -> None:
        selection = self.condition_tree.selection()
        if not selection:
            return
        for item_id in selection:
            rule = self.condition_tree_items.pop(item_id, None)
            if rule:
                if rule in self.condition_rules:
                    self.condition_rules.remove(rule)
            self.condition_tree.delete(item_id)
        if not self.condition_tree_items:
            self.remove_condition_button.configure(state=tk.DISABLED)
        self.status_var.set("Condition rule removed.")
        self._refresh_signal_control_state()

    def _on_condition_selected(self, _event: Any) -> None:
        selection = self.condition_tree.selection()
        if not selection:
            if not self.condition_tree_items:
                self.remove_condition_button.configure(state=tk.DISABLED)
            return
        self.remove_condition_button.configure(state=tk.NORMAL)
        item_id = selection[0]
        rule = self.condition_tree_items.get(item_id)
        if not rule:
            return
        self.condition_message_var.set(rule.get("message_display", ""))
        self.condition_message_combo.set(rule.get("message_display", ""))
        self._on_condition_message_selected(None)
        self.condition_signal_var.set(rule.get("signal", ""))
        self.condition_signal_combo.set(rule.get("signal", ""))
        self.condition_operator_var.set(rule.get("operator", "=="))
        self.condition_operator_combo.set(rule.get("operator", "=="))
        self.condition_compare_var.set(str(rule.get("compare_value", 0)))
        self.condition_target_type_var.set(rule.get("target_type", "int"))
        self.condition_target_type_combo.set(rule.get("target_type", "int"))
        self.condition_target_index_var.set(str(rule.get("target_index", 0)))
        self.condition_true_value_var.set(str(rule.get("true_value", 1)))
        false_val = rule.get("false_value")
        self.condition_false_value_var.set("" if false_val is None else str(false_val))

    def _add_or_update_expression_rule(self) -> None:
        if self.dbc_database is None or cantools is None:
            messagebox.showerror("Expression rule", "Load a DBC file before adding expressions.")
            return

        target_type = self.expression_target_type_var.get().strip().lower()
        if target_type not in ("float", "int"):
            messagebox.showerror("Expression rule", "Target type must be float or int.")
            return
        try:
            target_index = int(self.expression_target_index_var.get().strip())
        except ValueError:
            messagebox.showerror("Expression rule", "Target index must be an integer.")
            return
        if target_index < 0:
            messagebox.showerror("Expression rule", "Target index must be non-negative.")
            return

        expression = self.expression_text_var.get().strip()
        if not expression:
            messagebox.showerror("Expression rule", "Expression cannot be empty.")
            return

        target_display = f"{target_type} {target_index}"
        selected = self.expression_tree.selection()
        if selected:
            tree_id = selected[0]
            rule = self.expression_tree_items.get(tree_id)
        else:
            tree_id = None
            rule = None

        if rule is None:
            self.expression_rule_counter += 1
            rule = {"id": self.expression_rule_counter}
            tree_id = self.expression_tree.insert("", "end", values=(target_display, expression))
            self.expression_tree_items[tree_id] = rule
            self.expression_rules.append(rule)
        else:
            self.expression_tree.item(tree_id, values=(target_display, expression))

        rule.update(
            {
                "target_type": target_type,
                "target_index": target_index,
                "expression": expression,
                "tree_id": tree_id,
            }
        )

        self.remove_expression_button.configure(state=tk.NORMAL)
        self.status_var.set(f"Expression rule set: UDP {target_display} = {expression}")
        self._refresh_signal_control_state()

    def _remove_selected_expression_rule(self) -> None:
        selection = self.expression_tree.selection()
        if not selection:
            return
        for item_id in selection:
            rule = self.expression_tree_items.pop(item_id, None)
            if rule and rule in self.expression_rules:
                self.expression_rules.remove(rule)
            self.expression_tree.delete(item_id)
        if not self.expression_tree_items:
            self.remove_expression_button.configure(state=tk.DISABLED)
        self.status_var.set("Expression rule removed.")
        self._refresh_signal_control_state()

    def _on_expression_selected(self, _event: Any) -> None:
        selection = self.expression_tree.selection()
        if not selection:
            if not self.expression_tree_items:
                self.remove_expression_button.configure(state=tk.DISABLED)
            return
        self.remove_expression_button.configure(state=tk.NORMAL)
        item_id = selection[0]
        rule = self.expression_tree_items.get(item_id)
        if not rule:
            return
        self.expression_target_type_var.set(rule.get("target_type", "float"))
        self.expression_target_type_combo.set(rule.get("target_type", "float"))
        self.expression_target_index_var.set(str(rule.get("target_index", 0)))
        self.expression_text_var.set(rule.get("expression", ""))

    def _handle_dbc_signals(
        self,
        identifier: int,
        is_extended: bool,
        data_bytes: bytes,
    ) -> Tuple[Optional[Dict[int, float]], Dict[int, float]]:
        key = (identifier, bool(is_extended))
        message_obj = self.dbc_messages_by_id.get(key)
        if message_obj is None:
            return None, {}

        try:
            decoded = message_obj.decode(
                data_bytes,
                decode_choices=False,
                scaling=True,
                allow_truncated=True,
            )
        except TypeError:
            decoded = message_obj.decode(
                data_bytes,
                decode_choices=False,
                scaling=True,
            )
        except Exception as exc:  # pragma: no cover - runtime feedback
            self.log_queue.put(("warning", f"DBC decode failed for {message_obj.name}: {exc}"))
            return None, {}

        snapshot, updated = self._update_signal_values(key, decoded)
        return snapshot if snapshot else None, updated

    def _get_signal_value(self, message_key: Tuple[int, bool], signal_name: str) -> Optional[float]:
        with self.signal_lock:
            return self.latest_signals.get((message_key, signal_name))

    def _evaluate_condition(self, value: float, compare: float, operator: str) -> bool:
        if operator == "==":
            return value == compare
        if operator == "!=":
            return value != compare
        if operator == ">":
            return value > compare
        if operator == "<":
            return value < compare
        if operator == ">=":
            return value >= compare
        if operator == "<=":
            return value <= compare
        return False

    def _apply_condition_rules(self, slot_values: Dict[str, Dict[int, float]]) -> None:
        if not self.condition_rules:
            return
        for rule in self.condition_rules:
            message_key = rule.get("message_key")
            signal_name = rule.get("signal")
            if message_key is None or not signal_name:
                continue
            value = self._get_signal_value(message_key, signal_name)
            if value is None:
                continue
            compare_value = rule.get("compare_value")
            operator = rule.get("operator", "==")
            if compare_value is None or operator is None:
                continue
            if self._evaluate_condition(value, compare_value, operator):
                target_value = rule.get("true_value")
            else:
                target_value = rule.get("false_value")
                if target_value is None:
                    continue
            target_type = rule.get("target_type", "int")
            target_index = rule.get("target_index")
            if target_type not in ("float", "int") or not isinstance(target_index, int):
                continue
            slot_values.setdefault(target_type, {})[target_index] = float(target_value)

    def _sanitize_name(self, name: str) -> str:
        sanitized = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in name)
        if not sanitized:
            sanitized = "value"
        if sanitized[0].isdigit():
            sanitized = f"_{sanitized}"
        return sanitized

    def _build_expression_context(self) -> Dict[str, Any]:
        context: Dict[str, Any] = {}
        message_groups: Dict[str, Dict[str, float]] = {}
        with self.signal_lock:
            for (message_key, signal_name), value in self.latest_signals.items():
                message_name = self.message_names_by_key.get(message_key, f"MSG_{message_key[0]:X}")
                message_var = self._sanitize_name(message_name)
                signal_var = self._sanitize_name(signal_name)
                if signal_var not in context:
                    context[signal_var] = value
                msg_dict = message_groups.setdefault(message_var, {})
                msg_dict[signal_var] = value
        from types import SimpleNamespace

        for message_var, signals in message_groups.items():
            context[message_var] = SimpleNamespace(**signals)
        return context

    def _safe_eval_expression(self, expression: str, context: Dict[str, Any]) -> Optional[float]:
        if not expression:
            return None
        try:
            import ast

            tree = ast.parse(expression, mode="eval")
        except SyntaxError as exc:
            self.log_queue.put(("warning", f"Expression parse error '{expression}': {exc}"))
            return None

        allowed_nodes = (
            ast.Expression,
            ast.BinOp,
            ast.UnaryOp,
            ast.BoolOp,
            ast.Compare,
            ast.IfExp,
            ast.Call,
            ast.Name,
            ast.Load,
            ast.Constant,
        )
        allowed_bin_ops = (ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Mod, ast.Pow)
        allowed_unary_ops = (ast.UAdd, ast.USub)
        allowed_bool_ops = (ast.And, ast.Or)
        allowed_cmp_ops = (ast.Eq, ast.NotEq, ast.Gt, ast.GtE, ast.Lt, ast.LtE)
        allowed_functions = {"abs": abs, "min": min, "max": max, "round": round}

        for node in ast.walk(tree):
            if not isinstance(node, allowed_nodes):
                self.log_queue.put(("warning", f"Expression contains unsupported syntax: {expression}"))
                return None
            if isinstance(node, ast.BinOp) and not isinstance(node.op, allowed_bin_ops):
                self.log_queue.put(("warning", f"Expression contains unsupported operator: {expression}"))
                return None
            if isinstance(node, ast.UnaryOp) and not isinstance(node.op, allowed_unary_ops):
                self.log_queue.put(("warning", f"Expression contains unsupported unary operator: {expression}"))
                return None
            if isinstance(node, ast.BoolOp) and not isinstance(node.op, allowed_bool_ops):
                self.log_queue.put(("warning", f"Expression contains unsupported boolean operator: {expression}"))
                return None
            if isinstance(node, ast.Compare):
                if any(not isinstance(op, allowed_cmp_ops) for op in node.ops):
                    self.log_queue.put(("warning", f"Expression contains unsupported comparison: {expression}"))
                    return None
            if isinstance(node, ast.Call):
                if not isinstance(node.func, ast.Name) or node.func.id not in allowed_functions:
                    self.log_queue.put(("warning", f"Expression contains unsupported function: {expression}"))
                    return None
        eval_context = dict(context)
        eval_context.update(allowed_functions)
        try:
            result = eval(compile(tree, "<expression>", "eval"), {"__builtins__": {}}, eval_context)
        except Exception as exc:
            self.log_queue.put(("warning", f"Expression evaluation error '{expression}': {exc}"))
            return None
        try:
            return float(result)
        except (TypeError, ValueError):
            return None

    def _apply_expression_rules(self, slot_values: Dict[str, Dict[int, float]]) -> None:
        if not self.expression_rules:
            return
        context = self._build_expression_context()
        for rule in self.expression_rules:
            expression = rule.get("expression", "")
            target_type = rule.get("target_type", "float")
            target_index = rule.get("target_index")
            if target_type not in ("float", "int") or not isinstance(target_index, int):
                continue
            result = self._safe_eval_expression(expression, context)
            if result is None:
                continue
            slot_values.setdefault(target_type, {})[target_index] = result

    def _update_signal_values(
        self,
        key: Tuple[int, bool],
        decoded: Dict[str, Any],
    ) -> Tuple[Dict[int, float], Dict[int, float]]:
        updated: Dict[int, float] = {}
        with self.signal_lock:
            for signal_name, value in decoded.items():
                if isinstance(value, bool):
                    numeric_signal_value = float(int(value))
                elif isinstance(value, (int, float)):
                    numeric_signal_value = float(value)
                else:
                    try:
                        numeric_signal_value = float(value)
                    except (TypeError, ValueError):
                        continue
                self.latest_signals[(key, signal_name)] = numeric_signal_value

            for (index, value_type), mapping in list(self.signal_mappings.items()):
                if mapping["key"] != key:
                    continue
                signal_name = mapping["signal"]
                if signal_name not in decoded:
                    continue
                value = decoded[signal_name]
                numeric_value: Optional[float]
                if isinstance(value, (int, float)):
                    numeric_value = float(value)
                elif isinstance(value, bool):
                    numeric_value = float(int(value))
                else:
                    try:
                        numeric_value = float(value)
                    except (TypeError, ValueError):
                        continue
                self.signal_values[index] = numeric_value
                updated[index] = numeric_value
            snapshot = dict(self.signal_values)
        return snapshot, updated

    def _update_signal_tree_with_values(self, values: Dict[int, float]) -> None:
        for (index, value_type), item_id in list(self.signal_tree_items.items()):
            if index not in values:
                continue
            numeric_value = values[index]
            with self.signal_lock:
                mapping = self.signal_mappings.get((index, value_type))
            current = list(self.signal_tree.item(item_id, "values"))
            if not current:
                continue
            display_index = f"{value_type} {index}"
            if value_type == "int":
                try:
                    formatted = str(int(numeric_value))
                except (TypeError, ValueError, OverflowError):
                    formatted = "-"
            else:
                formatted = f"{numeric_value:.6f}".rstrip("0").rstrip(".")
                formatted = formatted or "0"
            current[0] = display_index
            current[3] = value_type
            current[4] = formatted
            self.signal_tree.item(item_id, values=current)

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
        self._set_controls_running(True)

        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
            args=(channel, self.include_tx_var.get()),
        )
        self.reader_thread.start()
        self.status_var.set(f"Streaming to UDP {udp_ip}:{udp_port}")

    def stop_bridge(self) -> None:
        if not self.reader_active:
            return

        self.reader_active = False
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.5)
        self.reader_thread = None

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
                    time.sleep(0.01)
                    continue

                for message in messages:
                    payload, log_line, data_bytes = format_canfd_message(message)
                    signals_snapshot = None
                    if self.dbc_messages_by_id and (
                        self.signal_mappings or self.condition_rules or self.expression_rules
                    ):
                        signals_snapshot, updated_indices = self._handle_dbc_signals(
                            payload["id"],
                            payload["is_extended_id"],
                            data_bytes,
                        )
                        if updated_indices:
                            self.log_queue.put(("signal_values", updated_indices))
                    else:
                        updated_indices = {}
                    if not signals_snapshot:
                        with self.signal_lock:
                            if self.signal_values:
                                signals_snapshot = dict(self.signal_values)
                    signals_snapshot = signals_snapshot or {}
                    with self.signal_lock:
                        mapping_snapshot = {key: dict(mapping) for key, mapping in self.signal_mappings.items()}
                    slot_values: Dict[str, Dict[int, float]] = {"float": {}, "int": {}}
                    for (index, value_type), mapping in mapping_snapshot.items():
                        if index not in signals_snapshot:
                            continue
                        try:
                            numeric_value = float(signals_snapshot[index])
                        except (TypeError, ValueError):
                            continue
                        if value_type == "int":
                            slot_values["int"][index] = numeric_value
                        else:
                            slot_values["float"][index] = numeric_value
                    self._apply_expression_rules(slot_values)
                    self._apply_condition_rules(slot_values)
                    if signals_snapshot:
                        payload["signals"] = signals_snapshot
                    float_map = {
                        f"udp float value {index}": float(value)
                        for index, value in slot_values["float"].items()
                    }
                    int_map = {}
                    for index, value in slot_values["int"].items():
                        try:
                            int_map[f"udp int value {index}"] = int(value)
                        except (TypeError, ValueError, OverflowError):
                            continue
                    if float_map:
                        payload["udp_float_values"] = float_map
                    if int_map:
                        payload["udp_int_values"] = int_map
                    try:
                        udp_payload = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("ascii")
                    except (UnicodeEncodeError, TypeError):
                        udp_payload = json.dumps(payload, separators=(",", ":")).encode("utf-8")

                    try:
                        self.udp_socket.sendto(udp_payload, self.udp_target)
                    except OSError as err:
                        self.log_queue.put(("error", f"UDP send failed: {err}"))
                        self.reader_active = False
                        return

                    self.log_queue.put(("udp_payload", udp_payload))
                    self.log_queue.put(("log", log_line))
                    frame_count += 1
                    if frame_count % 10 == 0:
                        self.log_queue.put(("counter", frame_count))
        except Exception as exc:  # pylint: disable=broad-except
            self.log_queue.put(("error", f"Reader error: {exc}"))
        finally:
            self.log_queue.put(("stopped", None))

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
                    if isinstance(payload, (bytes, bytearray)):
                        display = payload.decode("utf-8", errors="replace")
                    else:
                        display = str(payload)
                    display = display.strip()
                    if not display:
                        display = "(empty message)"
                    if len(display) > 800:
                        display = display[:800] + " ..."
                    self.last_udp_payload_var.set(display)
                elif event == "signal_values":
                    if isinstance(payload, dict):
                        self._update_signal_tree_with_values(payload)
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
