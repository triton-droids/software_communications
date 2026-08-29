"""Math helpers and process-wide debug logging setup.

The original script duplicated imports and exception-hook logic.  Keeping
those concerns here means the tuner/plotter/CLI modules only import what
they use.
"""
from __future__ import annotations

import faulthandler
import logging
import math
import os
import sys
import threading
import traceback

LOG_PATH = os.path.join(os.getcwd(), "robstride_gain_tuner_debug.log")


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` to the inclusive range [low, high]."""
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to the interval (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def offset_to_pi(angle: float) -> float:
    """Return the multiple of 2*pi that brings ``angle`` closest to zero."""
    return angle - wrap_to_pi(angle)


def setup_logging() -> logging.Logger:
    """Configure console + file logging and return the module logger."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(threadName)s | %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(LOG_PATH, mode="a", encoding="utf-8"),
        ],
    )
    return logging.getLogger("robstride_gain_tuner")


def print_full_exception(prefix: str, exc: BaseException, log: logging.Logger) -> None:
    """Print a full traceback to both the console and the debug log."""
    tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    msg = f"{prefix}\n--- TRACEBACK START ---\n{tb}--- TRACEBACK END ---\nLog file: {LOG_PATH}"
    print(msg)
    try:
        log.error(msg)
    except Exception:
        pass


def install_excepthooks(log: logging.Logger) -> None:
    """Install main-thread and worker-thread exception hooks for noisy failures."""
    try:
        faulthandler.enable(all_threads=True)
    except Exception as exc:
        print(f"[WARN] faulthandler.enable failed: {exc}")

    def _sys_excepthook(exc_type, exc, tb):
        msg = "".join(traceback.format_exception(exc_type, exc, tb))
        print(f"\n[FATAL] Unhandled exception on main thread:\n{msg}\nLog file: {LOG_PATH}\n")
        try:
            log.critical(msg)
        except Exception:
            pass

    sys.excepthook = _sys_excepthook

    if hasattr(threading, "excepthook"):
        def _thread_excepthook(args):
            msg = "".join(traceback.format_exception(args.exc_type, args.exc_value, args.exc_traceback))
            print(
                f"\n[FATAL] Unhandled exception in thread '{args.thread.name}':\n"
                f"{msg}\nLog file: {LOG_PATH}\n"
            )
            try:
                log.critical(msg)
            except Exception:
                pass

        threading.excepthook = _thread_excepthook
