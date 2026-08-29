"""Command-line input helpers and the interactive command loop."""
from __future__ import annotations

import logging
import os
from typing import List, Set

from .logging_utils import print_full_exception
from .tuner import get_robstride_sdk

LOG = logging.getLogger("robstride_gain_tuner")


def parse_motor_ids_or_scan() -> List[int]:
    """Ask for motor IDs on stdin, or scan the CAN bus when empty."""
    print("Enter motor IDs (space-separated, e.g. '1 2 3')")
    print("Or press Enter to scan CAN bus.")
    s = input("Motor IDs: ").strip()

    if s:
        try:
            ids = [int(x) for x in s.split()]
        except ValueError:
            raise SystemExit("Invalid ID list. Use space-separated integers, e.g. '1 2 3'.")
        if not ids:
            raise SystemExit("No ids provided.")
        if len(set(ids)) != len(ids):
            raise SystemExit("Duplicate ids.")
        if any(i < 1 or i > 255 for i in ids):
            raise SystemExit("Ids must be 1..255.")
        return ids

    channel = "can0"
    print(f"Scanning {channel} for motors (IDs 1..255) ...")
    try:
        RobstrideBus = get_robstride_sdk()["bus"]
    except ImportError as exc:
        raise SystemExit(f"RobStride SDK is required for scanning: {exc}")

    found = RobstrideBus.scan_channel(channel, start_id=1, end_id=255)
    if not found:
        raise SystemExit("No motors found.")
    ids = sorted(found.keys())
    print(f"Found motors: {ids}")
    return ids


def command_loop(tuner) -> None:
    """Interactive command prompt.

    Runs in a daemon thread; the main thread owns the matplotlib window and
    control loop.
    """
    print("\nCommands:")
    print("  select <id> | select all")
    print("  invert <id...> | invert all")
    print("  kp <value> | kd <value>")
    print("  hold")
    print("  step <deg>")
    print("  goto <deg>")
    print("  sine <amp_deg> <freq_hz> [duration_s]")
    print("  stop")
    print("  status")
    print("  q\n")

    while True:
        try:
            if len(tuner.selected) == len(tuner.motor_states):
                sel_str = "ALL"
            else:
                sel_str = ",".join(str(x) for x in sorted(tuner.selected))
            cmd = input(f"[{sel_str}] >> ").strip().lower()
            if not cmd:
                continue

            if cmd in ("q", "quit", "exit"):
                tuner.shutdown()
                os._exit(0)

            if cmd == "status":
                tuner.status()
                continue

            if cmd == "hold":
                tuner.hold()
                continue

            if cmd == "stop":
                tuner.stop_excitation()
                continue

            if cmd.startswith("select "):
                parts = cmd.split()
                if len(parts) != 2:
                    print("Usage: select <id> or select all")
                    continue
                if parts[1] == "all":
                    tuner.select(None)
                else:
                    tuner.select(int(parts[1]))
                continue

            if cmd.startswith("invert"):
                parts = cmd.split()
                if len(parts) < 2:
                    print("Usage: invert <id...> or invert all")
                    continue
                if "all" in parts[1:]:
                    tuner.invert(set(tuner.motor_states.keys()))
                else:
                    ids: Set[int] = set()
                    for p in parts[1:]:
                        try:
                            ids.add(int(p))
                        except ValueError:
                            pass
                    tuner.invert(ids)
                continue

            if cmd.startswith("kp "):
                tuner.set_kp(float(cmd.split()[1]))
                continue

            if cmd.startswith("kd "):
                tuner.set_kd(float(cmd.split()[1]))
                continue

            if cmd.startswith("step "):
                tuner.step(float(cmd.split()[1]))
                continue

            if cmd.startswith("goto "):
                tuner.goto(float(cmd.split()[1]))
                continue

            if cmd.startswith("sine "):
                parts = cmd.split()
                if len(parts) not in (3, 4):
                    print("Usage: sine <amp_deg> <freq_hz> [duration_s]")
                    continue
                amp = float(parts[1])
                freq = float(parts[2])
                dur = float(parts[3]) if len(parts) == 4 else None
                tuner.sine(amp, freq, dur)
                continue

            print("Unknown command. Try: status, hold, step, goto, sine, kp, kd, select, invert, stop, q")

        except KeyboardInterrupt:
            print("keyboard interrupt")
            tuner.shutdown()
            os._exit(0)
        except Exception as exc:
            print_full_exception("[CLI] Exception while processing command", exc, LOG)
