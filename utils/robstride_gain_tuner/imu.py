"""IMU fall detection with compatibility imports.

The original script imported ``imu_stream`` directly.  In this repo the same
functionality lives in ``Attitude_Sensing/.../imu_read.py`` (and on older
branches as ``utils/imu_read.py``), so the import is resolved lazily and
falls back across all known locations.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

_REPO_ROOT = Path(__file__).resolve().parents[2]


def _add_imu_search_paths() -> None:
    candidates = (
        _REPO_ROOT,
        _REPO_ROOT / "Attitude_Sensing" / "src" / "attitude_sensing_pkg" / "attitude_sensing_pkg",
        _REPO_ROOT / "utils",
    )
    for path in candidates:
        if path.is_dir() and str(path) not in sys.path:
            sys.path.insert(0, str(path))


def _import_imu():
    """Return ``(iter_imu_samples, RK4DeadReckoner)`` from any available module."""
    _add_imu_search_paths()
    errors = []
    for module_name in ("imu_stream", "utils.imu_read", "imu_read"):
        try:
            module = __import__(module_name, fromlist=["iter_imu_samples", "RK4DeadReckoner"])
            return module.iter_imu_samples, module.RK4DeadReckoner
        except Exception as exc:  # noqa: BLE001 - import fallback is intentionally broad
            errors.append(f"{module_name}: {exc}")
    raise ImportError("Could not import IMU helpers. Tried: " + "; ".join(errors))


def create_integrator(rk4_cls=None):
    """Create an RK4 integrator.

    The newer firmware helper accepts gating constants; older versions only
    accept ``gravity_world``.  Try the rich signature first and fall back.
    """
    if rk4_cls is None:
        _iter_imu_samples, rk4_cls = _import_imu()

    try:
        return rk4_cls(
            gravity_world=(0.0, 0.0, 9.80665),
            kp_acc=2.0,
            acc_gate_g=0.08,
            jerk_gate_ms3=10.0,
            gyro_gate_dps=20.0,
            gate_min_count=3,
            stationary_min_count=3,
        )
    except TypeError:
        return rk4_cls(gravity_world=(0.0, 0.0, 9.80665))


class IMUFallDetector:
    """Detect falls from chest-chassis IMU data and call ``halt_fn``.

    Falls back to firmware roll/pitch when the integrator is not used.
    Trips ``halt_fn`` if |roll| or |pitch| exceeds the configured thresholds
    for ``confirm_count`` consecutive samples.
    """

    def __init__(
        self,
        port: str,
        halt_fn: Callable[[str], None],
        baud: int = 115200,
        rate_hz: float = 100.0,
        fall_roll_deg: float = 40.0,
        fall_pitch_deg: float = 40.0,
        confirm_count: int = 3,
        use_integrator: bool = True,
    ):
        self.port = port
        self.baud = baud
        self.halt_fn = halt_fn
        self.rate_hz = rate_hz
        self.fall_roll_rad = math.radians(fall_roll_deg)
        self.fall_pitch_rad = math.radians(fall_pitch_deg)
        self.confirm_count = max(1, int(confirm_count))
        self.use_integrator = use_integrator

        # Latest orientation (updated by reader thread)
        self._roll: float = 0.0
        self._pitch: float = 0.0
        self._yaw: float = 0.0
        self._lock = threading.Lock()

        self._tripped = False
        self._trip_count = 0
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Diagnostics
        self.samples_read = 0
        self.errors = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._tripped = False
        self._trip_count = 0
        self._thread = threading.Thread(
            target=self._loop,
            name="imu_fall_detector",
            daemon=True,
        )
        self._thread.start()
        print(
            f"[IMU] Fall detector started | port={self.port} "
            f"roll_thresh={math.degrees(self.fall_roll_rad):.1f}deg "
            f"pitch_thresh={math.degrees(self.fall_pitch_rad):.1f}deg "
            f"confirm={self.confirm_count} "
            f"integrator={'RK4' if self.use_integrator else 'firmware_rpy'}"
        )

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    @property
    def tripped(self) -> bool:
        return self._tripped

    def get_rpy(self):
        """Return (roll, pitch, yaw) in radians. Thread-safe."""
        with self._lock:
            return self._roll, self._pitch, self._yaw

    def _check_fall(self, roll: float, pitch: float) -> None:
        if self._tripped:
            return

        over_threshold = (
            abs(roll) > self.fall_roll_rad or abs(pitch) > self.fall_pitch_rad
        )

        if over_threshold:
            self._trip_count += 1
        else:
            self._trip_count = 0  # must be consecutive

        if self._trip_count >= self.confirm_count:
            self._tripped = True
            reason = (
                f"[IMU] FALL DETECTED - "
                f"roll={math.degrees(roll):+.1f}deg "
                f"pitch={math.degrees(pitch):+.1f}deg "
                f"(thresh +/-roll={math.degrees(self.fall_roll_rad):.1f}deg "
                f"+/-pitch={math.degrees(self.fall_pitch_rad):.1f}deg "
                f"over {self._trip_count} samples)"
            )
            print(reason)
            try:
                self.halt_fn(reason)
            except Exception as exc:
                print(f"[IMU] halt_fn raised: {exc}")

    def _loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                iter_imu_samples, rk4_cls = _import_imu()
                integrator = None
                if self.use_integrator:
                    integrator = create_integrator(rk4_cls)

                # iter_imu_samples handles serial open, read, parse, rate control
                for sample in iter_imu_samples(
                    source="serial",
                    port=self.port,
                    baud=self.baud,
                    rate_hz=self.rate_hz,
                    integrator=integrator,
                    include_all=True,
                ):
                    if self._stop_evt.is_set():
                        return

                    # Prefer RK4 fused RPY if integrator is running; fall back
                    # to firmware roll/pitch from BNO085.
                    rpy_deg = sample.get("rpy_deg")
                    if rpy_deg is not None and self.use_integrator:
                        roll_rad = math.radians(rpy_deg[0])
                        pitch_rad = math.radians(rpy_deg[1])
                        yaw_rad = math.radians(rpy_deg[2])
                    else:
                        roll_raw = sample.get("roll_deg")
                        pitch_raw = sample.get("pitch_deg")
                        if roll_raw is None or pitch_raw is None:
                            continue
                        roll_rad = math.radians(roll_raw)
                        pitch_rad = math.radians(pitch_raw)
                        yaw_rad = 0.0

                    with self._lock:
                        self._roll = roll_rad
                        self._pitch = pitch_rad
                        self._yaw = yaw_rad

                    self.samples_read += 1
                    self._check_fall(roll_rad, pitch_rad)

            except Exception as exc:  # noqa: BLE001 - reconnect on any serial/IMU error
                self.errors += 1
                print(f"[IMU] Error in reader loop: {exc}, reconnecting in 2s...")
                time.sleep(2.0)
