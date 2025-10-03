"""Helper script to bundle can_udp_bridge.py into a standalone executable."""

from __future__ import annotations

import os
from pathlib import Path


def _require_pyinstaller() -> None:
    try:
        import PyInstaller.__main__  # type: ignore[attr-defined]
    except ModuleNotFoundError as exc:  # pragma: no cover - runtime dependency check
        raise SystemExit(
            "PyInstaller is required to build the executable. Install it with 'pip install pyinstaller'."
        ) from exc


def build() -> None:
    _require_pyinstaller()

    import PyInstaller.__main__  # type: ignore[attr-defined]

    project_root = Path(__file__).resolve().parents[1]
    entry_script = project_root / "src" / "can_udp_bridge.py"

    if not entry_script.exists():  # pragma: no cover - sanity check
        raise SystemExit(f"Entry script not found: {entry_script}")

    add_data_sep = ";" if os.name == "nt" else ":"
    config_example = project_root / "config" / "example_config.json"
    add_data_arg = f"{config_example}{add_data_sep}config"

    PyInstaller.__main__.run(
        [
            str(entry_script),
            "--onefile",
            "--name",
            "can_udp_bridge",
            "--add-data",
            add_data_arg,
        ]
    )


def main() -> None:
    build()


if __name__ == "__main__":
    main()
