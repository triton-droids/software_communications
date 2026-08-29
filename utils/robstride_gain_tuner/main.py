"""Program entry point."""
from __future__ import annotations

import os
import signal
import sys
import threading

from .cli import command_loop, parse_motor_ids_or_scan
from .logging_utils import install_excepthooks, setup_logging
from .plotter import LivePlotter
from .tuner import GainTunerMIT


def main() -> None:
    log = setup_logging()
    install_excepthooks(log)

    motor_ids = parse_motor_ids_or_scan()

    tuner = GainTunerMIT(
        motor_ids=motor_ids,
        channel="can0",
        bitrate=1_000_000,
        model="rs-03",   # fallback only; per-ID models are used automatically
        hz=60.0,
        ramp_deg_s=30.0,
    )

    def _signal_handler(_signum=None, _frame=None):
        tuner.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    if not tuner.connect():
        sys.exit(1)

    threading.Thread(target=command_loop, args=(tuner,), daemon=True).start()

    plotter = LivePlotter(tuner, window_s=30.0, ui_hz=60.0)
    plotter.show()


if __name__ == "__main__":
    main()
