"""CAN to UDP bridge for TOSUN CAN USB devices.

This script listens to frames on a CAN bus and forwards configured signals
as JSON payloads over UDP.  It is designed for use with TOSUN CAN USB
interfaces that expose a SocketCAN or compatible interface via python-can.
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

try:
    import can
except ImportError as exc:  # pragma: no cover - informative error
    raise SystemExit(
        "python-can is required to run this script. Install it with 'pip install python-can'."
    ) from exc


@dataclass(frozen=True)
class SignalConfig:
    """Configuration describing how to extract a signal from a CAN frame."""

    name: str
    can_id: int
    start_bit: int
    length: int
    byte_order: str = "little"
    signed: bool = False
    scale: float = 1.0
    offset: float = 0.0

    def __post_init__(self) -> None:  # type: ignore[override]
        if self.length <= 0:
            raise ValueError(f"Signal '{self.name}' length must be positive")
        if self.start_bit < 0:
            raise ValueError(f"Signal '{self.name}' start_bit cannot be negative")
        if self.byte_order not in {"little", "big"}:
            raise ValueError(
                f"Signal '{self.name}' byte_order must be either 'little' or 'big', got {self.byte_order!r}"
            )


def _load_config(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as fh:
            return json.load(fh)
    except FileNotFoundError as exc:  # pragma: no cover - configuration error
        raise SystemExit(f"Configuration file not found: {path}") from exc
    except json.JSONDecodeError as exc:  # pragma: no cover - configuration error
        raise SystemExit(f"Failed to parse configuration file {path}: {exc}") from exc


def _build_signal_map(signal_configs: Iterable[SignalConfig]) -> Dict[int, List[SignalConfig]]:
    signal_map: Dict[int, List[SignalConfig]] = {}
    for cfg in signal_configs:
        signal_map.setdefault(cfg.can_id, []).append(cfg)
    return signal_map


def _extract_signal(data: bytes, cfg: SignalConfig) -> float:
    total_bits = len(data) * 8
    if cfg.start_bit + cfg.length > total_bits:
        raise ValueError(
            f"Signal '{cfg.name}' (start_bit={cfg.start_bit}, length={cfg.length}) exceeds frame size ({total_bits} bits)"
        )

    mask = (1 << cfg.length) - 1
    if cfg.byte_order == "big":
        value = int.from_bytes(data, byteorder="big", signed=False)
        shift = total_bits - cfg.start_bit - cfg.length
    else:
        value = int.from_bytes(data, byteorder="little", signed=False)
        shift = cfg.start_bit

    raw = (value >> shift) & mask
    if cfg.signed and cfg.length > 0 and (raw & (1 << (cfg.length - 1))):
        raw -= 1 << cfg.length

    scaled = raw * cfg.scale + cfg.offset
    return scaled


def _create_bus(can_config: dict) -> can.Bus:
    bus_kwargs = {
        key: value
        for key, value in can_config.items()
        if key
        not in {
            "filters",
        }
    }
    try:
        bus = can.Bus(**bus_kwargs)
    except TypeError as exc:  # pragma: no cover - configuration error
        raise SystemExit(f"Invalid CAN configuration: {exc}") from exc

    filters = can_config.get("filters")
    if filters:
        try:
            bus.set_filters(filters)
        except can.CanError as exc:  # pragma: no cover - runtime error
            raise SystemExit(f"Failed to set CAN filters: {exc}") from exc

    return bus


def _send_udp(sock: socket.socket, destination: Tuple[str, int], payload: dict) -> None:
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    sock.sendto(data, destination)


def _process_frame(
    message: can.Message,
    signal_map: Dict[int, List[SignalConfig]],
    sock: socket.socket,
    destination: Tuple[str, int],
    include_raw_frame: bool,
) -> None:
    configs = signal_map.get(message.arbitration_id)
    if not configs:
        return

    extracted = {}
    for cfg in configs:
        try:
            value = _extract_signal(message.data, cfg)
        except ValueError as exc:
            logging.warning("Failed to extract signal %s: %s", cfg.name, exc)
            continue
        extracted[cfg.name] = value

    if not extracted:
        return

    payload = {
        "timestamp": message.timestamp,
        "can_id": message.arbitration_id,
        "extended": message.is_extended_id,
        "signals": extracted,
    }

    if include_raw_frame:
        payload["raw_data"] = message.data.hex()

    _send_udp(sock, destination, payload)
    logging.debug("Forwarded CAN ID 0x%X with signals %s", message.arbitration_id, extracted)


def _parse_signal_configs(config: dict) -> List[SignalConfig]:
    signals_cfg = config.get("signals", [])
    signal_configs: List[SignalConfig] = []
    for entry in signals_cfg:
        try:
            signal_configs.append(
                SignalConfig(
                    name=entry["name"],
                    can_id=int(entry["can_id"], 0)
                    if isinstance(entry["can_id"], str)
                    else int(entry["can_id"]),
                    start_bit=int(entry.get("start_bit", 0)),
                    length=int(entry["length"]),
                    byte_order=entry.get("byte_order", "little"),
                    signed=bool(entry.get("signed", False)),
                    scale=float(entry.get("scale", 1.0)),
                    offset=float(entry.get("offset", 0.0)),
                )
            )
        except KeyError as exc:  # pragma: no cover - configuration error
            raise SystemExit(f"Signal configuration is missing required field: {exc}") from exc
        except ValueError as exc:  # pragma: no cover - configuration error
            raise SystemExit(f"Invalid signal configuration: {exc}") from exc
    return signal_configs


def _setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(message)s",
    )


def run(config_path: Path) -> None:
    config = _load_config(config_path)

    can_config = config.get("can")
    if not isinstance(can_config, dict):  # pragma: no cover - configuration error
        raise SystemExit("Configuration must define a 'can' section")

    udp_config = config.get("udp")
    if not isinstance(udp_config, dict):  # pragma: no cover - configuration error
        raise SystemExit("Configuration must define a 'udp' section")

    include_raw_frame = bool(config.get("include_raw_frame", False))

    signal_map = _build_signal_map(_parse_signal_configs(config))
    if not signal_map:
        logging.warning("No signals configured; frames will not be forwarded")

    destination = (udp_config.get("host", "127.0.0.1"), int(udp_config.get("port", 5000)))

    with _create_bus(can_config) as bus, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        logging.info(
            "Forwarding CAN frames to UDP %s:%s", destination[0], destination[1]
        )
        while True:
            try:
                message = bus.recv(timeout=1.0)
            except can.CanError as exc:  # pragma: no cover - runtime error
                logging.error("CAN bus error: %s", exc)
                time.sleep(1.0)
                continue

            if message is None:
                continue

            _process_frame(message, signal_map, sock, destination, include_raw_frame)


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Bridge CAN frames to UDP packets")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/example_config.json"),
        help="Path to the configuration JSON file",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Logging level (e.g. DEBUG, INFO, WARNING)",
    )
    args = parser.parse_args(argv)

    _setup_logging(args.log_level)

    try:
        run(args.config)
    except KeyboardInterrupt:
        logging.info("Interrupted by user; shutting down")


if __name__ == "__main__":
    main()
