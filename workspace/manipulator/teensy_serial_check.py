"""Non-motion serial sanity check for the Teensy bagger controller.

Examples:
    python -m workspace.manipulator.teensy_serial_check --list
    python -m workspace.manipulator.teensy_serial_check --port COM5
    python -m workspace.manipulator.teensy_serial_check --port COM5 --dry-run

This script never sends a move command. It only uses:
    PING, STAT, POS
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

SERIAL_IMPORT_ERROR: str | None = None


def _import_pyserial():  # noqa: ANN202 - imported module objects are intentionally dynamic
    """Import the external pyserial package even when this folder has serial.py."""
    local_dir = Path(__file__).resolve().parent
    original_path = list(sys.path)
    old_serial = sys.modules.get("serial")
    old_serial_file = Path(getattr(old_serial, "__file__", "") or "").resolve() if old_serial else None

    if old_serial_file and old_serial_file == local_dir / "serial.py":
        sys.modules.pop("serial", None)

    try:
        sys.path = [p for p in sys.path if Path(p or ".").resolve() != local_dir]
        import serial as pyserial  # type: ignore
        from serial.tools import list_ports as pyserial_list_ports  # type: ignore

        if not hasattr(pyserial, "Serial"):
            raise ImportError(f"Imported serial module is not pyserial: {getattr(pyserial, '__file__', 'unknown')}")
        return pyserial, pyserial_list_ports, None
    except ImportError as exc:  # pragma: no cover - user environment dependent
        return None, None, str(exc)
    finally:
        sys.path = original_path


serial, list_ports, SERIAL_IMPORT_ERROR = _import_pyserial()


@dataclass(frozen=True)
class CheckResult:
    port: str
    baud: int
    ok: bool
    lines: list[str]


def available_ports() -> list[str]:
    if list_ports is None:
        return []
    return [p.device for p in list_ports.comports()]


def describe_ports() -> list[str]:
    if list_ports is None:
        return []
    out = []
    for p in list_ports.comports():
        desc = getattr(p, "description", "") or ""
        hwid = getattr(p, "hwid", "") or ""
        out.append(f"{p.device:>12}  {desc}  {hwid}")
    return out


def auto_select_port() -> str | None:
    if list_ports is None:
        return None
    ports = list(list_ports.comports())
    if not ports:
        return None

    preferred_keywords = ("teensy", "usb serial", "usb-serial", "usb", "serial")
    for keyword in preferred_keywords:
        for p in ports:
            text = f"{getattr(p, 'device', '')} {getattr(p, 'description', '')} {getattr(p, 'hwid', '')}".lower()
            if keyword in text:
                return p.device
    return ports[0].device


def drain_lines(ser, duration_s: float) -> list[str]:  # noqa: ANN001 - serial object type varies
    deadline = time.time() + duration_s
    lines: list[str] = []
    while time.time() < deadline:
        try:
            line = ser.readline().decode(errors="replace").strip()
        except Exception as exc:  # pragma: no cover - hardware dependent
            lines.append(f"RX_ERROR: {exc}")
            break
        if line:
            lines.append(line)
        else:
            time.sleep(0.02)
    return lines


def send_line(ser, line: str) -> None:  # noqa: ANN001 - serial object type varies
    ser.write((line + "\n").encode())
    ser.flush()


def run_check(port: str, baud: int, startup_wait: float, response_wait: float, dry_run: bool = False) -> CheckResult:
    commands = ["PING", "STAT", "POS"]

    if dry_run:
        lines = [
            f"DRY_RUN would open {port} at {baud} baud",
            *(f"DRY_RUN TX: {cmd}" for cmd in commands),
        ]
        return CheckResult(port=port, baud=baud, ok=True, lines=lines)

    if serial is None:
        detail = f" ({SERIAL_IMPORT_ERROR})" if SERIAL_IMPORT_ERROR else ""
        raise RuntimeError(f"pyserial is not available{detail}. Run: python -m pip install pyserial")

    lines: list[str] = []
    with serial.Serial(port, baud, timeout=0.10) as ser:
        time.sleep(startup_wait)
        lines.extend(drain_lines(ser, 0.75))

        for cmd in commands:
            lines.append(f"TX: {cmd}")
            send_line(ser, cmd)
            rx = drain_lines(ser, response_wait)
            lines.extend(rx)

    ok = any("PONG" in line for line in lines) or any(line.startswith("READY") for line in lines)
    return CheckResult(port=port, baud=baud, ok=ok, lines=lines)


def print_lines(lines: Iterable[str]) -> None:
    for line in lines:
        print(line)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Non-motion serial check for the Teensy bagger controller.")
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM5 or /dev/ttyACM0. If omitted, tries to auto-select.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate. Default: 115200")
    parser.add_argument("--list", action="store_true", help="List available serial ports and exit.")
    parser.add_argument("--dry-run", action="store_true", help="Do not open serial. Only print what would be checked.")
    parser.add_argument("--startup-wait", type=float, default=1.75, help="Seconds to wait after opening port.")
    parser.add_argument("--response-wait", type=float, default=0.35, help="Seconds to wait after each command.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.list:
        ports = describe_ports()
        if not ports:
            print("No serial ports found.")
            return 1
        print("Available serial ports:")
        print_lines(ports)
        return 0

    port = args.port or auto_select_port()
    if not port:
        print("No serial port selected/found. Try: python -m workspace.manipulator.teensy_serial_check --list")
        return 2

    try:
        result = run_check(
            port=port,
            baud=args.baud,
            startup_wait=args.startup_wait,
            response_wait=args.response_wait,
            dry_run=args.dry_run,
        )
    except Exception as exc:
        print(f"FAIL: {exc}")
        return 3

    print(f"Port: {result.port}")
    print(f"Baud: {result.baud}")
    print_lines(result.lines)

    if result.ok:
        print("PASS: Teensy serial link responded.")
        return 0

    print("FAIL: Port opened, but no READY/PONG response was detected.")
    print("Check the selected COM port, baud rate, USB cable, and whether the Teensy sketch is running.")
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
