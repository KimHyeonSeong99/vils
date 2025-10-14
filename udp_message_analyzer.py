#!/usr/bin/env python3
"""Simple console tool to inspect UDP payloads produced by the TOSUN CAN -> UDP bridge."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from collections import Counter
from typing import Any, Dict, Optional, Tuple


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Listen on a UDP port and pretty-print incoming CAN payloads from the bridge."
    )
    parser.add_argument(
        "--bind",
        default="127.0.0.1",
        help="IP address to bind (default: %(default)s).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
        help="UDP port to listen on (default: %(default)s).",
    )
    parser.add_argument(
        "--buffer-size",
        type=int,
        default=4096,
        help="Receive buffer size in bytes (default: %(default)s).",
    )
    parser.add_argument(
        "--filter-id",
        type=int,
        help="Only display messages with the specified CAN ID (decimal).",
    )
    parser.add_argument(
        "--filter-extended",
        action="store_true",
        help="Only display messages flagged as extended (is_extended_id == True).",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Show raw payload alongside parsed view.",
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="Print summary statistics every 100 messages.",
    )
    return parser.parse_args()


def _format_signal_summary(payload: Dict[str, Any]) -> str:
    signals = payload.get("signals") or {}
    if not isinstance(signals, dict):
        return ""
    parts = []
    for name, value in list(signals.items())[:6]:
        parts.append(f"{name}:{value}")
    extra = len(signals) - len(parts)
    if extra > 0:
        parts.append(f"...(+{extra})")
    return ", ".join(parts)


def _format_udp_values(prefix: str, values: Dict[str, Any]) -> str:
    parts = []
    for key, value in sorted(values.items()):
        parts.append(f"{key.split()[-1]}={value}")
    return f"{prefix}({', '.join(parts)})" if parts else ""


def _print_payload(
    decoded: Dict[str, Any],
    addr: Tuple[str, int],
    raw_text: Optional[str],
    show_raw: bool,
) -> None:
    can_id = decoded.get("id")
    direction = decoded.get("direction")
    is_extended = decoded.get("is_extended_id")
    timestamp = decoded.get("timestamp_seconds", time.time())
    channel = decoded.get("channel")
    dlc = decoded.get("dlc")
    data_hex = decoded.get("data")

    signal_summary = _format_signal_summary(decoded)
    float_summary = _format_udp_values("float", decoded.get("udp_float_values") or {})
    int_summary = _format_udp_values("int", decoded.get("udp_int_values") or {})
    extras = ", ".join(part for part in [signal_summary, float_summary, int_summary] if part)

    header = (
        f"[{time.strftime('%H:%M:%S', time.localtime(timestamp))}"
        f".{int((timestamp % 1)*1000):03d}]"
        f" CH{channel} ID={can_id} ({'EXT' if is_extended else 'STD'})"
        f" DIR={direction} DLC={dlc}"
    )
    print(header)
    print(f"  DATA: {data_hex}")
    if extras:
        print(f"  {extras}")
    print(f"  FROM: {addr[0]}:{addr[1]}")
    if show_raw and raw_text is not None:
        print("  RAW :", raw_text)
    print("-")


def main() -> None:
    args = _parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        print(f"Failed to bind UDP socket on {args.bind}:{args.port}: {exc}", file=sys.stderr)
        sys.exit(1)

    print(f"Listening on {args.bind}:{args.port} ... (Ctrl+C to stop)")
    sock.settimeout(1.0)

    counter = 0
    per_id = Counter()
    start_time = time.monotonic()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(args.buffer_size)
            except socket.timeout:
                continue
            counter += 1
            text: Optional[str]
            try:
                text = data.decode("utf-8", errors="replace")
            except Exception:
                text = None

            if text is None:
                print(f"Received non-text payload ({len(data)} bytes) from {addr}")
                continue

            try:
                decoded = json.loads(text)
            except json.JSONDecodeError:
                print(f"[WARN] Invalid JSON payload from {addr}: {text[:200]} ...")
                continue

            if args.filter_id is not None and decoded.get("id") != args.filter_id:
                continue
            if args.filter_extended and not decoded.get("is_extended_id"):
                continue

            can_id = decoded.get("id")
            if isinstance(can_id, int):
                per_id[can_id] += 1

            _print_payload(decoded, addr, text if args.raw else None, args.raw)

            if args.stats and counter % 100 == 0:
                elapsed = max(time.monotonic() - start_time, 1e-6)
                rate = counter / elapsed
                most_common = per_id.most_common(5)
                summary = ", ".join(f"{can_id}:{count}" for can_id, count in most_common)
                print(f"== Stats == messages={counter} rate={rate:.1f} msg/s top_ids={summary or 'n/a'}")
    except KeyboardInterrupt:
        print("\nStopping analyzer...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
