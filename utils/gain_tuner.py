#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride MIT Gain Tuner (Mode 0) + LIVE PLOTS (mac-safe main-thread control)
+ inversion array + joint limits + temperature telemetry (thermal safety disabled)
+ per-motor model map (RS02/RS03/RS04 mixed)

Key design:
- Control loop runs in matplotlib animation callback (main thread) to avoid macOS GUI starvation.
- Inversion array pre-sets st.direction for CAN IDs 1..10.
- Joint limits (radians) are enforced on target commands and during ramping (logical space).
- Temperature safety supervisor:
    OK -> DERATE (slow ramp) -> HOLD (freeze) -> DISABLED (disable motor)
  Gains are never scaled.
- Motor model is configured per CAN ID in MOTOR_MODEL_BY_ID.
- NOTE: To hold motor 2 or 7 at zero, select it and use `goto 0` or `hold`.

Run:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set up can0
  python3 robstride_gain_tuner_liveplot_mac_safe.py
"""

import sys
import os
import time
import math
import struct
import threading
import signal
from dataclasses import dataclass, field
from typing import Optional, Dict, Set, List, Tuple, Callable
from collections import deque
from imu_stream import iter_imu_samples, RK4DeadReckoner
import traceback
import faulthandler
import logging
import numpy as np
from time import perf_counter


import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -------------------- RobStride SDK imports --------------------
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
    from utils.actuation_safety import ActuationSafetyMonitor
except ImportError:
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType, CommunicationType
        from actuation_safety import ActuationSafetyMonitor
    except ImportError as e:
        print(f"Failed to import RobStride SDK: {e}")
        sys.exit(1)


# -------------------- Debug / logging --------------------
LOG_PATH = os.path.join(os.getcwd(), "robstride_gain_tuner_debug.log")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(threadName)s | %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(LOG_PATH, mode="a", encoding="utf-8"),
    ],
)
log = logging.getLogger("robstride_gain_tuner")

# Print tracebacks even on "silent" crashes / segfaults (and include all threads)
try:
    faulthandler.enable(all_threads=True)
except Exception as e:
    print(f"[WARN] faulthandler.enable failed: {e}")

def _print_full_exception(prefix: str, exc: BaseException):
    """Prints a full traceback to console + log file."""
    tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    msg = f"{prefix}\n--- TRACEBACK START ---\n{tb}--- TRACEBACK END ---\nLog file: {LOG_PATH}"
    print(msg)
    try:
        log.error(msg)
    except Exception:
        pass

def _sys_excepthook(exc_type, exc, tb):
    # This catches exceptions on the main thread that would otherwise just exit.
    msg = "".join(traceback.format_exception(exc_type, exc, tb))
    print(f"\n[FATAL] Unhandled exception on main thread:\n{msg}\nLog file: {LOG_PATH}\n")
    try:
        log.critical(msg)
    except Exception:
        pass

sys.excepthook = _sys_excepthook

# Python 3.8+ thread exception hook (very useful here)
if hasattr(threading, "excepthook"):
    def _thread_excepthook(args):
        msg = "".join(traceback.format_exception(args.exc_type, args.exc_value, args.exc_traceback))
        print(f"\n[FATAL] Unhandled exception in thread '{args.thread.name}':\n{msg}\nLog file: {LOG_PATH}\n")
        try:
            log.critical(msg)
        except Exception:
            pass
    threading.excepthook = _thread_excepthook


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def offset_to_pi(x: float) -> float:
    return x - wrap_to_pi(x)


# -------------------- Your provided config --------------------
# Inversion array interpreted sequentially for CAN IDs 1-10
INVERSION_ARRAY = [-1, -1, -1, 1, 1, 1, -1, 1, -1, -1]
INVERSION_BY_ID: Dict[int, int] = {i + 1: INVERSION_ARRAY[i] for i in range(len(INVERSION_ARRAY))}

# Joint limits in radians (logical joint space)
JOINT_LIMITS: Dict[int, Tuple[float, float]] = {
    1: (-1.57, 1.57),            # left_hip1_joint
    2: (-1.57, 0.436332),        # left_hip2_joint
    3: (-0.785398, 0.785398),    # left_thigh_joint
    4: (-2.0944, 0.0),           # left_knee_joint
    5: (-0.6, 0.6),              # left_ankle_joint
    6: (-1.57, 1.57),            # right_hip1_joint
    7: (-0.436332, 1.57),        # right_hip2_joint
    8: (-0.785398, 0.785398),    # right_thigh_joint
    9: (-2.0944, 0.0),           # right_knee_joint
    10: (-0.6, 0.6),             # right_ankle_joint
}

# Per-motor model mapping (from your list)
MOTOR_MODEL_BY_ID: Dict[int, str] = {
    1: "rs-04",
    2: "rs-03",
    3: "rs-03",
    4: "rs-04",
    5: "rs-02",
    6: "rs-04",
    7: "rs-03",
    8: "rs-03",
    9: "rs-04",
    10: "rs-02",
}

# Shared actuation safety monitor (joint-limit/jump trips)
ACTUATION_SAFETY_ENABLED = False  # temporary: disable joint-limit safety trips for tuner testing

# -------------------- Torque Spike Detection --------------------
TORQUE_SAFETY_ENABLED = True

# Per-model torque limits in Nm
# Set conservatively below the motor's rated peak torque
TORQUE_LIMITS_NM: Dict[str, float] = {
    "rs-04": 80.0,   # rated 40Nm, peak 120Nm — trip at ~67% of peak
    "rs-03": 40.0,   # rated 21Nm, peak 60Nm  — trip at ~67% of peak
    "rs-02": 12.0,   # rated 7Nm,  peak 17Nm  — trip at ~70% of peak
}
# ^ this should be accurate based on RobStride product info but can be adjusted based on actual torque data

TORQUE_DEFAULT_LIMIT_NM = 30.0  # fallback if model not in dict

# -------------------- IMU Fall Detection --------------------
IMU_ENABLED           = True
IMU_SERIAL_PORT       = "/dev/ttyACM0"   # your ESP32 port
IMU_BAUD              = 115200
IMU_RATE_HZ           = 100.0
IMU_FALL_ROLL_DEG     = 40.0
IMU_FALL_PITCH_DEG    = 40.0
IMU_CONFIRM_COUNT     = 3
IMU_USE_INTEGRATOR    = True   # False = use BNO085 firmware roll/pitch directly

# Temperature telemetry filter (thermal safety disabled)
TEMP_SAFETY_ENABLED = False
TEMP_DERATE_START_C = 65.0   # start slowing motion
TEMP_HOLD_C = 75.0           # freeze at current pose
TEMP_DISABLE_C = 85.0        # disable motor
TEMP_REENABLE_C = 70.0       # must cool below this to re-enable (hysteresis)
TEMP_VALID_MIN_C = -40.0     # reject implausible telemetry low values
TEMP_VALID_MAX_C = 200.0     # reject implausible telemetry high values
TEMP_MAX_STEP_C = 40.0       # reject one-sample temperature spikes

DERATE_MIN_SCALE = 0.20      # minimum motion scale at/above disable threshold
DISABLE_COOLDOWN_S = 2.0     # how long to stay disabled before trying to re-enable

# -------------------- Delay measurement knobs --------------------
STEP_CMD_EPS_RAD = 0.005     # detect "command started moving" (~0.29 deg)
STEP_POS_EPS_RAD = 0.010     # detect "position started moving" (~0.57 deg)
STEP_TIMEOUT_S = 2.0         # give up if no motion seen in this time


@dataclass
class Excitation:
    mode: str = "none"          # "none" | "sine"
    amp_rad: float = 0.0
    freq_hz: float = 0.0
    t0: float = 0.0
    duration_s: Optional[float] = None
    center_rad: float = 0.0


@dataclass
class MotorState:
    id: int
    name: str
    model: str

    # Telemetry
    raw_position: float = 0.0   # raw physical motor angle (rad)
    position: float = 0.0       # adjusted logical angle (rad)
    velocity: float = 0.0       # adjusted logical velocity (rad/s)
    torque: float = 0.0         # Nm
    temperature: float = 0.0    # C

    # Gains
    kp: float = 10.0
    kd: float = 0.2

    # Mount direction (1 normal, -1 inverted)
    direction: int = 1

    # Joint limits (logical space)
    limit_lo: float = -math.inf
    limit_hi: float = math.inf

    # Targets (logical, pre-direction)
    target_rad: float = 0.0
    commanded_target_rad: float = 0.0
    hold_center_rad: float = 0.0
    startup_motor_offset_rad: float = 0.0
    bypass_limit_clamp: bool = False  # hold out-of-limits startup pose until user commands motion

    # Excitation state
    excitation: Excitation = field(default_factory=Excitation)

    # Temp safety state
    temp_state: str = "OK"           # "OK" | "DERATE" | "HOLD" | "DISABLED"
    enabled: bool = True
    last_disable_t: float = 0.0

    last_error: Optional[str] = None

    # -------------------- Timing / delay metrics --------------------
    loop_dt_ms: float = 0.0        # wall-time between control_step calls (jitter indicator)
    write_dt_ms: float = 0.0       # duration of write_operation_frame() call
    read_dt_ms: float = 0.0        # duration of read_operation_frame() call
    io_gap_ms: float = 0.0         # (read_end - write_end) in ms (same-cycle IO gap)
    last_write_end_t: float = 0.0  # epoch seconds (time.time()) when last write finished
    last_read_end_t: float = 0.0   # epoch seconds when last read finished
    prev_sent_phys_target: float = 0.0
    step_pending: bool = False
    step_cmd_t: float = 0.0        # epoch seconds when first changed command was sent
    step_pos0: float = 0.0         # position at that time
    last_step_delay_s: float = math.nan
    consecutive_torque_spikes: int = 0

import threading
import math
import time
from typing import Callable, Optional

# Import from the existing imu pipeline
from imu_stream import iter_imu_samples, RK4DeadReckoner


class IMUFallDetector:
    """
    detects falls using imu data from the chest chassis and calls halt_fn
    this class requires input data from imu_read.py, and imu_read.py
    requires imu data to be CSV format. upload imu_read_firmware.cpp to
    the microcontroller for output data to be in the correct format for imu_read.py

    Falls back to firmware roll/pitch if integrator is not used.
    Trips halt_fn if |roll| or |pitch| exceeds thresholds for
    confirm_count consecutive samples.
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
        self.fall_roll_rad  = math.radians(fall_roll_deg)
        self.fall_pitch_rad = math.radians(fall_pitch_deg)
        self.confirm_count  = max(1, int(confirm_count))
        self.use_integrator = use_integrator

        # Latest orientation (updated by reader thread)
        self._roll:  float = 0.0
        self._pitch: float = 0.0
        self._yaw:   float = 0.0
        self._lock = threading.Lock()

        self._tripped    = False
        self._trip_count = 0
        self._stop_evt   = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Diagnostics
        self.samples_read = 0
        self.errors       = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._tripped    = False
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
        """Returns (roll, pitch, yaw) in radians. Thread-safe."""
        with self._lock:
            return self._roll, self._pitch, self._yaw

    def _check_fall(self, roll: float, pitch: float) -> None:
        if self._tripped:
            return

        over_threshold = (
            abs(roll)  > self.fall_roll_rad or
            abs(pitch) > self.fall_pitch_rad
        )

        if over_threshold:
            self._trip_count += 1
        else:
            self._trip_count = 0  # must be consecutive

        if self._trip_count >= self.confirm_count:
            self._tripped = True
            reason = (
                f"[IMU] FALL DETECTED — "
                f"roll={math.degrees(roll):+.1f}deg "
                f"pitch={math.degrees(pitch):+.1f}deg "
                f"(thresh ±roll={math.degrees(self.fall_roll_rad):.1f}deg "
                f"±pitch={math.degrees(self.fall_pitch_rad):.1f}deg "
                f"over {self._trip_count} samples)"
            )
            print(reason)
            try:
                self.halt_fn(reason)
            except Exception as e:
                print(f"[IMU] halt_fn raised: {e}")

    def _loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                # Create integrator if requested
                integrator = None
                if self.use_integrator:
                    integrator = RK4DeadReckoner(
                        gravity_world=(0.0, 0.0, 9.80665),
                        kp_acc=2.0,
                        acc_gate_g=0.08,
                        jerk_gate_ms3=10.0,
                        gyro_gate_dps=20.0,
                        gate_min_count=3,
                        stationary_min_count=3,
                    )

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

                    # Prefer RK4 fused RPY if integrator is running
                    # Fall back to firmware roll/pitch from BNO085
                    rpy_deg = sample.get("rpy_deg")
                    if rpy_deg is not None and self.use_integrator:
                        roll_rad  = math.radians(rpy_deg[0])
                        pitch_rad = math.radians(rpy_deg[1])
                        yaw_rad   = math.radians(rpy_deg[2])
                    else:
                        # Use firmware roll/pitch directly from BNO085 output
                        roll_raw  = sample.get("roll_deg")
                        pitch_raw = sample.get("pitch_deg")
                        if roll_raw is None or pitch_raw is None:
                            continue
                        roll_rad  = math.radians(roll_raw)
                        pitch_rad = math.radians(pitch_raw)
                        yaw_rad   = 0.0

                    with self._lock:
                        self._roll  = roll_rad
                        self._pitch = pitch_rad
                        self._yaw   = yaw_rad

                    self.samples_read += 1
                    self._check_fall(roll_rad, pitch_rad)

            except Exception as e:
                self.errors += 1
                print(f"[IMU] Error in reader loop: {e}, reconnecting in 2s...")
                time.sleep(2.0)

class GainTunerMIT:
    def __init__(
        self,
        motor_ids: List[int],
        channel: str = "can0",
        bitrate: int = 1_000_000,
        model: str = "rs-03",   # fallback if ID not in MOTOR_MODEL_BY_ID
        hz: float = 60.0,
        ramp_deg_s: float = 30.0,
    ):
        self.channel = channel
        self.bitrate = bitrate
        self.model = model.lower()

        self.hz = float(hz)
        self.dt = 1.0 / self.hz
        self.ramp_rad_s_nominal = math.radians(float(ramp_deg_s))

        self.motor_states: Dict[int, MotorState] = {}
        for mid in motor_ids:
            mmodel = MOTOR_MODEL_BY_ID.get(mid, self.model)
            st = MotorState(id=mid, name=f"motor_{mid}", model=mmodel)

            # Apply inversion array for IDs 1..10, otherwise default 1
            st.direction = int(INVERSION_BY_ID.get(mid, 1))

            # Apply joint limits if provided, else infinite
            if mid in JOINT_LIMITS:
                st.limit_lo, st.limit_hi = JOINT_LIMITS[mid]

            self.motor_states[mid] = st

        self.selected: Set[int] = set(motor_ids)

        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()

        self.running = True
        self.connected = False
        self._last_control_step_t = time.time()
        self.safety_monitor: Optional[ActuationSafetyMonitor] = None
        self.safety_tripped = False
        self.safety_reason: Optional[str] = None
        self.imu_detector: Optional[IMUFallDetector] = None

    def _trip_safety(self, reason: str):
        if self.safety_tripped:
            return
        self.safety_tripped = True
        self.safety_reason = reason
        print(reason)

    def _assert_safe(self):
        if not ACTUATION_SAFETY_ENABLED:
            return
        if self.safety_tripped:
            raise RuntimeError(self.safety_reason or "Safety monitor tripped")

    def _read_logical_for_safety(self, mid: int) -> float:
        if self.bus is None:
            raise RuntimeError("Bus not connected")
        acquired = self.lock.acquire(timeout=0.001)
        if not acquired:
            raise TimeoutError("safety read skipped: control lock busy")
        try:
            st = self.motor_states[mid]
            pos, vel, tq, temp = self.bus.read_operation_frame(st.name, timeout=0.005)
            self._update_telemetry_from_raw(st, pos, vel, tq, temp)
            return float(st.position)
        finally:
            self.lock.release()

    def _sanitize_temp_reading(self, st: MotorState, raw_temp: float) -> float:
        t = float(raw_temp)
        prev = float(st.temperature)
        if (not math.isfinite(t)) or t < TEMP_VALID_MIN_C or t > TEMP_VALID_MAX_C:
            st.last_error = f"ignored invalid temp reading: {t:.1f}C"
            return prev
        if math.isfinite(prev) and TEMP_VALID_MIN_C <= prev <= TEMP_VALID_MAX_C:
            if abs(t - prev) > TEMP_MAX_STEP_C:
                st.last_error = f"ignored temp spike: prev={prev:.1f}C new={t:.1f}C"
                return prev
        return t

    def _logical_from_raw(self, st: MotorState, raw_pos: float) -> float:
        return float(raw_pos) / float(st.direction) - st.startup_motor_offset_rad

    def _physical_from_logical(self, st: MotorState, logical_rad: float) -> float:
        return float((float(logical_rad) + st.startup_motor_offset_rad) * float(st.direction))

    def _update_telemetry_from_raw(self, st: MotorState, pos: float, vel: float, tq: float, temp: float) -> None:
        st.raw_position = float(pos)
        st.position = self._logical_from_raw(st, pos)
        st.velocity = float(vel) / float(st.direction)
        st.torque = float(tq)
        st.temperature = self._sanitize_temp_reading(st, temp)

    def _check_torque_spikes(self, st: MotorState) -> None:
    """
    compares motor torque to the limits of each motor model, torque limit constants defined at top of file
    Requires 3 consecutive over-limit readings to trip,
    preventing single noisy samples from causing false trips.
    """
    if not TORQUE_SAFETY_ENABLED:
        return

    limit = TORQUE_LIMITS_NM.get(st.model, TORQUE_DEFAULT_LIMIT_NM)

    if abs(st.torque) > limit:
        st.consecutive_torque_spikes += 1
        if st.consecutive_torque_spikes >= 3:
            self._trip_safety(
                f"[SAFETY] motor {st.id} ({st.model}) torque spike: "
                f"{st.torque:.2f} Nm exceeds limit of {limit:.1f} Nm "
                f"({st.consecutive_torque_spikes} consecutive readings)"
            )
    else:
        # Reset counter — must be consecutive to trip
        st.consecutive_torque_spikes = 0

    def _start_safety_monitor(self):
        if not ACTUATION_SAFETY_ENABLED:
            self.safety_monitor = None
            print("[SAFETY] actuation safety monitor disabled for tuner testing.")
            return
        joint_limits = {mid: (st.limit_lo, st.limit_hi) for mid, st in self.motor_states.items()}
        self.safety_monitor = ActuationSafetyMonitor(
            name="gain_tuner",
            motor_ids=list(self.motor_states.keys()),
            joint_limits_by_id=joint_limits,
            read_logical_pos_fn=self._read_logical_for_safety,
            halt_fn=self._trip_safety,
            control_hz=self.hz,
            read_hz=max(120.0, self.hz * 2.0),
            max_step_deg=90.0,
        )
        self.safety_monitor.start()
        per_motor_hz = self.safety_monitor.read_hz / max(1, len(self.motor_states))
        print(
            f"[SAFETY] monitor started: control_hz={self.hz:.1f}, "
            f"read_hz_total={self.safety_monitor.read_hz:.1f}, per_motor~{per_motor_hz:.1f}, "
            f"max_jump_deg=90.0"
        )

    def _start_imu_detector(self) -> None:
        if not IMU_ENABLED:
            print("[IMU] Fall detection disabled.")
            return
        self.imu_detector = IMUFallDetector(
            port=IMU_SERIAL_PORT,
            halt_fn=self._trip_safety,
            baud=IMU_BAUD,
            rate_hz=IMU_RATE_HZ,
            fall_roll_deg=IMU_FALL_ROLL_DEG,
            fall_pitch_deg=IMU_FALL_PITCH_DEG,
            confirm_count=IMU_CONFIRM_COUNT,
            use_integrator=IMU_USE_INTEGRATOR,
        )
        self.imu_detector.start()

    def _clamp_to_limits(self, st: MotorState, logical_rad: float) -> float:
        return clamp(logical_rad, st.limit_lo, st.limit_hi)

    def _within_limits(self, st: MotorState, logical_rad: float) -> bool:
        return st.limit_lo <= float(logical_rad) <= st.limit_hi

    def _set_mode_raw(self, mode: int, motor_id: int):
        motor_name = f"motor_{motor_id}"
        device_id = self.bus.motors[motor_name].id
        param_id, _, _ = ParameterType.MODE  # MODE is int8
        value_buffer = struct.pack("<bBH", int(mode), 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.1)

    def _motion_scale_from_temp(self, temp_c: float) -> float:
        """
        Motion derating scale for ramp rate (NOT gains).
        1.0 until TEMP_DERATE_START_C, then linearly down to DERATE_MIN_SCALE at TEMP_DISABLE_C.
        """
        if temp_c <= TEMP_DERATE_START_C:
            return 1.0
        if temp_c >= TEMP_DISABLE_C:
            return DERATE_MIN_SCALE
        frac = (temp_c - TEMP_DERATE_START_C) / (TEMP_DISABLE_C - TEMP_DERATE_START_C)
        return clamp(1.0 - frac * (1.0 - DERATE_MIN_SCALE), DERATE_MIN_SCALE, 1.0)

    def _update_temp_state_pre(self, st: MotorState, now: float):
        """
        Use last-known temperature to transition state BEFORE sending.
        (Fast reaction requires at least one-cycle latency unless your driver provides async temp.)
        """
        if not TEMP_SAFETY_ENABLED:
            st.temp_state = "OFF"
            return
        t = st.temperature
        if st.temp_state != "DISABLED":
            if t >= TEMP_DISABLE_C:
                st.temp_state = "DISABLED"
                st.last_disable_t = now
            elif t >= TEMP_HOLD_C:
                st.temp_state = "HOLD"
            elif t >= TEMP_DERATE_START_C:
                st.temp_state = "DERATE"
            else:
                st.temp_state = "OK"

        # Auto re-enable logic is checked in control_step()

    def connect(self) -> bool:
        # IMPORTANT: use per-motor model here
        motors = {
            f"motor_{mid}": Motor(id=mid, model=self.motor_states[mid].model)
            for mid in self.motor_states.keys()
        }
        calibration = {
            f"motor_{mid}": {"direction": 1, "homing_offset": 0.0}
            for mid in self.motor_states.keys()
        }

        try:
            try:
                self.bus = RobstrideBus(self.channel, motors, calibration, bitrate=self.bitrate)
            except TypeError:
                self.bus = RobstrideBus(self.channel, motors, calibration)

            print(f"Connecting to {self.channel} (bitrate={self.bitrate}) ...")
            self.bus.connect(handshake=True)

            with self.lock:
                for mid, st in self.motor_states.items():
                    print(
                        f"[ID {mid}] model={st.model} enable + MIT mode | dir={st.direction:+d} "
                        f"| limits=[{st.limit_lo:.4f},{st.limit_hi:.4f}] rad"
                    )
                    self.bus.enable(st.name)
                    st.enabled = True
                    time.sleep(0.25)

                    self._set_mode_raw(0, mid)

                    # Read once; set targets to current pose (no motion)
                    try:
                        pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
                        st.startup_motor_offset_rad = offset_to_pi(float(pos) / float(st.direction))
                        self._update_telemetry_from_raw(st, pos, vel, tq, temp)

                        logical = st.position
                        # IMPORTANT: do NOT clamp initial hold (could cause motion on connect).
                        st.target_rad = logical
                        st.commanded_target_rad = logical
                        st.hold_center_rad = logical
                        st.bypass_limit_clamp = not self._within_limits(st, logical)

                        # If out of limits, just warn (future commands will be clamped)
                        if st.bypass_limit_clamp:
                            print(
                                f"  WARN: current logical pos {logical:.4f} rad is outside limits; holding anyway (no motion)."
                            )

                        st.last_error = None
                    except Exception as e:
                        st.last_error = str(e)
                        st.target_rad = 0.0
                        st.commanded_target_rad = 0.0
                        st.hold_center_rad = 0.0
                        st.bypass_limit_clamp = False

                    # Send an initial "hold"
                    physical_target = self._physical_from_logical(st, st.commanded_target_rad)
                    self.bus.write_operation_frame(st.name, physical_target, st.kp, st.kd, 0.0, 0.0)

                    # --- delay-metrics init (avoid arming a fake step on first cycle) ---
                    st.prev_sent_phys_target = physical_target
                    st.step_pending = False
                    st.last_step_delay_s = math.nan

                    time.sleep(0.05)

            self.connected = True
            self.running = True
            self._start_safety_monitor()
            self._start_imu_detector()
            if not TEMP_SAFETY_ENABLED:
                print("[TEMP] thermal safety disabled; temperatures will still be displayed.")
            print("Connected. Motors are holding their current position (no motion).")
            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False

    def _disable_motor_locked(self, st: MotorState, now: float):
        if not st.enabled:
            return
        try:
            # Freeze command state before disable
            st.excitation = Excitation()
            logical = st.position
            st.target_rad = logical
            st.commanded_target_rad = logical
            st.hold_center_rad = logical
            st.bypass_limit_clamp = not self._within_limits(st, logical)

            self.bus.disable(st.name)
            st.enabled = False
            st.last_disable_t = now
            print(f"[TEMP] DISABLED motor {st.id} at {st.temperature:.1f}C")
        except Exception as e:
            st.last_error = f"disable failed: {e}"

    def _reenable_motor_locked(self, st: MotorState, now: float):
        if st.enabled:
            return
        try:
            self.bus.enable(st.name)
            time.sleep(0.05)
            self._set_mode_raw(0, st.id)

            # Re-sync hold to current position after re-enable
            try:
                pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
                self._update_telemetry_from_raw(st, pos, vel, tq, temp)
            except Exception:
                pass

            logical = st.position
            st.excitation = Excitation()
            st.target_rad = logical
            st.commanded_target_rad = logical
            st.hold_center_rad = logical
            st.bypass_limit_clamp = not self._within_limits(st, logical)

            physical_target = self._physical_from_logical(st, logical)
            self.bus.write_operation_frame(st.name, physical_target, float(st.kp), float(st.kd), 0.0, 0.0)

            # --- delay-metrics init (avoid arming a fake step on first cycle) ---
            st.prev_sent_phys_target = physical_target
            st.step_pending = False
            st.last_step_delay_s = math.nan

            st.enabled = True
            st.temp_state = "HOLD"  # come back in HOLD; user/policy can move again when cool
            print(f"[TEMP] RE-ENABLED motor {st.id} at {st.temperature:.1f}C (state=HOLD)")
        except Exception as e:
            st.last_error = f"reenable failed: {e}"

    def control_step(self, dt: float):
        """
        One control cycle (excitation+ramp+send+read) driven by plot callback.
        Safety order:
          - decide temp states based on last temperature (pre)
          - apply HOLD/DISABLE actions
          - compute target (with limit clamp) + ramp (with temp derate)
          - send (if enabled)
          - read telemetry
          - if temp now critical, disable immediately for next cycles
        """
        if not (self.running and self.connected and self.bus):
            return
        self._assert_safe()

        dt = float(clamp(dt, 0.0, 0.05))
        now = time.time()

        # --- loop timing (jitter indicator) ---
        loop_dt = now - self._last_control_step_t
        self._last_control_step_t = now

        with self.lock:
            for st in self.motor_states.values():
                st.loop_dt_ms = float(loop_dt) * 1000.0

            # --- temp state update (pre-send) ---
            for st in self.motor_states.values():
                self._update_temp_state_pre(st, now)

            # --- handle DISABLED transitions / auto re-enable ---
            if TEMP_SAFETY_ENABLED:
                for st in self.motor_states.values():
                    if st.temp_state == "DISABLED":
                        if st.enabled:
                            self._disable_motor_locked(st, now)
                    else:
                        if (not st.enabled) and (st.temperature <= TEMP_REENABLE_C) and (
                            (now - st.last_disable_t) >= DISABLE_COOLDOWN_S
                        ):
                            self._reenable_motor_locked(st, now)

            # --- handle HOLD state: freeze target at current pose (no excitation) ---
            if TEMP_SAFETY_ENABLED:
                for st in self.motor_states.values():
                    if st.temp_state == "HOLD":
                        st.excitation = Excitation()
                        logical = st.position
                        st.target_rad = logical
                        st.commanded_target_rad = logical
                        st.hold_center_rad = logical
                        st.bypass_limit_clamp = not self._within_limits(st, logical)

            # --- excitation + clamp + ramp ---
            for st in self.motor_states.values():
                if not st.enabled:
                    continue  # disabled: don't update commands

                ex = st.excitation
                if ex.mode == "sine" and (not TEMP_SAFETY_ENABLED or st.temp_state != "HOLD"):
                    if ex.duration_s is not None and (now - ex.t0) >= ex.duration_s:
                        st.excitation = Excitation()
                        st.target_rad = st.hold_center_rad
                    else:
                        st.target_rad = ex.center_rad + ex.amp_rad * math.sin(
                            2.0 * math.pi * ex.freq_hz * (now - ex.t0)
                        )

                # enforce joint limits in logical space for any motion command
                if not st.bypass_limit_clamp:
                    st.target_rad = self._clamp_to_limits(st, st.target_rad)

                # ramp (derate motion if hot)
                motion_scale = 1.0
                if TEMP_SAFETY_ENABLED and st.temp_state == "DERATE":
                    motion_scale = self._motion_scale_from_temp(st.temperature)

                max_step = self.ramp_rad_s_nominal * dt * motion_scale
                delta = st.target_rad - st.commanded_target_rad
                if abs(delta) <= max_step:
                    st.commanded_target_rad = st.target_rad
                else:
                    st.commanded_target_rad += math.copysign(max_step, delta)

                # also ensure commanded stays within limits
                if not st.bypass_limit_clamp:
                    st.commanded_target_rad = self._clamp_to_limits(st, st.commanded_target_rad)

            # --- send frames (fixed gains; RL-safe) + IO timing + step-delay arming ---
            for st in self.motor_states.values():
                if not st.enabled:
                    continue
                try:
                    physical_target = self._physical_from_logical(st, st.commanded_target_rad)
                    arm_step = (
                        st.excitation.mode != "sine"
                        and (not st.step_pending)
                        and abs(physical_target - st.prev_sent_phys_target) >= STEP_CMD_EPS_RAD
                    )

                    t0 = perf_counter()
                    self.bus.write_operation_frame(
                        st.name, physical_target, float(st.kp), float(st.kd), 0.0, 0.0
                    )
                    t1 = perf_counter()

                    st.write_dt_ms = (t1 - t0) * 1000.0
                    st.last_write_end_t = time.time()

                    if arm_step:
                        st.step_pending = True
                        st.step_cmd_t = st.last_write_end_t
                        st.step_pos0 = st.position
                        st.last_step_delay_s = math.nan

                    st.prev_sent_phys_target = physical_target

                except Exception as e:
                    if "No response" not in str(e):
                        st.last_error = str(e)

            # --- read frames + IO timing + step-delay completion ---
            for st in self.motor_states.values():
                if not st.enabled:
                    continue
                try:
                    t0 = perf_counter()
                    pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
                    t1 = perf_counter()

                    st.read_dt_ms = (t1 - t0) * 1000.0
                    st.last_read_end_t = time.time()

                    self._update_telemetry_from_raw(st, pos, vel, tq, temp)
                    self._check_torque_spikes(st)
                    st.last_error = None

                    if st.last_write_end_t > 0.0:
                        st.io_gap_ms = (st.last_read_end_t - st.last_write_end_t) * 1000.0

                    if st.step_pending:
                        if (st.last_read_end_t - st.step_cmd_t) > STEP_TIMEOUT_S:
                            st.step_pending = False
                        else:
                            if abs(st.position - st.step_pos0) >= STEP_POS_EPS_RAD:
                                st.last_step_delay_s = (st.last_read_end_t - st.step_cmd_t)
                                st.step_pending = False

                except Exception as e:
                    if "No response" not in str(e):
                        st.last_error = str(e)

            # --- immediate post-read critical check (affects next cycle) ---
            if TEMP_SAFETY_ENABLED:
                for st in self.motor_states.values():
                    if st.enabled and st.temperature >= TEMP_DISABLE_C:
                        st.temp_state = "DISABLED"
                        st.last_disable_t = now
                        self._disable_motor_locked(st, now)

    # -------------------- commands --------------------
    def _confirm_large_change(self, delta_deg: float) -> bool:
        if abs(delta_deg) <= 15.0:
            return True
        print("WARNING: Large change requested.")
        print(f"Delta: {delta_deg:+.1f} deg (>|15| requires confirmation)")
        ans = input("Proceed? (y/n): ").strip().lower()
        return ans in ("y", "yes")

    def select(self, motor_id: Optional[int]):
        if motor_id is None:
            self.selected = set(self.motor_states.keys())
            print(f"Selected all motors: {sorted(self.selected)}")
            return
        if motor_id not in self.motor_states:
            print(f"Motor {motor_id} not found. Available: {sorted(self.motor_states.keys())}")
            return
        self.selected = {motor_id}

        st = self.motor_states[motor_id]
        if st.last_error:
            print(f"[WARN] Motor {motor_id} has last_error: {st.last_error} (see also {LOG_PATH})")

        print(f"Selected motor: {motor_id}")

    def invert(self, ids: Set[int]):
        """
        Manual override toggle (in addition to initial INVERSION_BY_ID preset).
        This changes sign mapping immediately; we re-hold to avoid motion.
        """
        with self.lock:
            for mid in ids:
                if mid not in self.motor_states:
                    continue
                st = self.motor_states[mid]
                st.direction *= -1
                st.startup_motor_offset_rad = offset_to_pi(st.raw_position / float(st.direction))
                st.position = self._logical_from_raw(st, st.raw_position)
                st.velocity *= -1.0
                # keep logical target matching current physical pose to avoid motion
                logical = st.position
                st.target_rad = logical
                st.commanded_target_rad = logical
                st.hold_center_rad = logical
                st.bypass_limit_clamp = not self._within_limits(st, logical)
                st.excitation = Excitation()
        print(f"Toggled direction for motors: {sorted(ids)}")

    def set_kp(self, kp: float):
        kp = float(kp)
        if not (0.0 <= kp <= 5000.0):
            print("kp out of range (0..5000).")
            return
        with self.lock:
            for mid in self.selected:
                self.motor_states[mid].kp = kp
        print(f"Set kp={kp:.1f} for motors: {sorted(self.selected)}")

    def set_kd(self, kd: float):
        kd = float(kd)
        if not (0.0 <= kd <= 100.0):
            print("kd out of range (0..100).")
            return
        with self.lock:
            for mid in self.selected:
                self.motor_states[mid].kd = kd
        print(f"Set kd={kd:.2f} for motors: {sorted(self.selected)}")

    def hold(self):
        with self.lock:
            for mid in self.selected:
                st = self.motor_states[mid]
                st.excitation = Excitation()
                logical = st.position
                st.target_rad = logical
                st.commanded_target_rad = logical
                st.hold_center_rad = logical
                st.bypass_limit_clamp = not self._within_limits(st, logical)
        print(f"Hold set for motors: {sorted(self.selected)}")

    def step(self, delta_deg: float):
        delta_deg = float(clamp(delta_deg, -90.0, 90.0))
        if not self._confirm_large_change(delta_deg):
            print("Cancelled.")
            return
        with self.lock:
            for mid in self.selected:
                st = self.motor_states[mid]
                st.excitation = Excitation()
                st.bypass_limit_clamp = False
                st.target_rad = self._clamp_to_limits(st, st.target_rad + math.radians(delta_deg))
        print(f"Step {delta_deg:+.1f} deg (clamped to limits) for motors: {sorted(self.selected)}")

    def goto(self, angle_deg: float):
        angle_deg = float(clamp(angle_deg, -720.0, 720.0))
        mids = sorted(self.selected)
        if mids:
            cur_deg = math.degrees(self.motor_states[mids[0]].target_rad)
            if not self._confirm_large_change(angle_deg - cur_deg):
                print("Cancelled.")
                return
        with self.lock:
            for mid in self.selected:
                st = self.motor_states[mid]
                st.excitation = Excitation()
                st.bypass_limit_clamp = False
                st.target_rad = self._clamp_to_limits(st, math.radians(angle_deg))
        print(f"Goto {angle_deg:+.1f} deg (clamped to limits) for motors: {sorted(self.selected)}")

    def sine(self, amp_deg: float, freq_hz: float, duration_s: Optional[float]):
        amp_deg = float(clamp(amp_deg, 0.0, 90.0))
        freq_hz = float(clamp(freq_hz, 0.1, 5.0))
        if duration_s is not None:
            duration_s = float(clamp(duration_s, 0.2, 30.0))
        with self.lock:
            now = time.time()
            for mid in self.selected:
                st = self.motor_states[mid]
                st.bypass_limit_clamp = False
                center = self._clamp_to_limits(st, st.target_rad)
                st.target_rad = center
                st.hold_center_rad = center
                st.excitation = Excitation(
                    mode="sine",
                    amp_rad=math.radians(amp_deg),
                    freq_hz=freq_hz,
                    t0=now,
                    duration_s=duration_s,
                    center_rad=center,
                )
        dstr = f"{duration_s:.2f}s" if duration_s is not None else "infinite"
        print(
            f"Sine excite: amp={amp_deg:.2f}deg freq={freq_hz:.2f}Hz duration={dstr} "
            f"(limits enforced) for motors: {sorted(self.selected)}"
        )

    def stop_excitation(self):
        with self.lock:
            for mid in self.selected:
                self.motor_states[mid].excitation = Excitation()
        print(f"Stopped excitation for motors: {sorted(self.selected)}")

    def status(self):
        with self.lock:
            print("-" * 132)
            print(
                f"{'ID':<4} {'Sel':<4} {'Model':<6} {'Pos(deg)':<10} {'Cmd(deg)':<10} {'Vel':<10} {'Tq':<10} "
                f"{'Temp':<8} {'State':<9} {'Kp':<8} {'Kd':<8} {'Dir':<4} {'Lim(rad)':<22}"
            )
            print("-" * 132)
            for mid in sorted(self.motor_states.keys()):
                st = self.motor_states[mid]
                sel = "*" if mid in self.selected else ""
                pos_deg = math.degrees(st.position)
                cmd_deg = math.degrees(st.commanded_target_rad)
                dir_str = "INV" if st.direction == -1 else "NOR"
                lim_str = f"[{st.limit_lo:.3f},{st.limit_hi:.3f}]"
                print(
                    f"{mid:<4} {sel:<4} {st.model:<6} {pos_deg:<10.2f} {cmd_deg:<10.2f} {st.velocity:<10.3f} "
                    f"{st.torque:<10.3f} {st.temperature:<8.1f} {st.temp_state:<9} {st.kp:<8.1f} {st.kd:<8.2f} "
                    f"{dir_str:<4} {lim_str:<22}"
                )
                if st.last_error:
                    print(f"     error: {st.last_error}")
            print("-" * 132)
            if self.imu_detector is not None:
                roll, pitch, _ = self.imu_detector.get_rpy()
                print(
                    f"\n[IMU] port={IMU_SERIAL_PORT} "
                    f"roll={math.degrees(roll):+.1f}deg "
                    f"pitch={math.degrees(pitch):+.1f}deg | "
                    f"samples={self.imu_detector.samples_read} "
                    f"errors={self.imu_detector.errors} "
                    f"tripped={self.imu_detector.tripped}"
                )
            else:
                print("\n[IMU] Not running.")

    def shutdown(self):
        print("Shutting down...")
        self.running = False
        
        if self.imu_detector is not None:
            self.imu_detector.stop()
            self.imu_detector = None
        
        if self.safety_monitor is not None:
            self.safety_monitor.stop()
            self.safety_monitor = None

        if self.bus and self.connected:
            with self.lock:
                # Hold current pose briefly (only for enabled motors), then disable
                for st in self.motor_states.values():
                    if not st.enabled:
                        continue
                    try:
                        logical = st.position
                        physical_target = self._physical_from_logical(st, logical)
                        self.bus.write_operation_frame(st.name, physical_target, st.kp, st.kd, 0.0, 0.0)
                    except Exception:
                        pass
                time.sleep(0.2)

                for st in self.motor_states.values():
                    try:
                        if st.enabled:
                            self.bus.disable(st.name)
                    except Exception:
                        pass
            try:
                self.bus.disconnect()
            except Exception:
                pass

        self.connected = False
        print("Done.")


# -------------------- Motor ID parsing / scan --------------------
def parse_motor_ids_or_scan() -> List[int]:
    print("Enter motor IDs (space-separated, e.g. '1 2 3')")
    print("Or press Enter to scan CAN bus.")
    s = input("Motor IDs: ").strip()

    if s:
        ids = [int(x) for x in s.split()]
        if not ids:
            raise SystemExit("No ids provided.")
        if len(set(ids)) != len(ids):
            raise SystemExit("Duplicate ids.")
        if any(i < 1 or i > 255 for i in ids):
            raise SystemExit("Ids must be 1..255.")
        return ids

    channel = "can0"
    print(f"Scanning {channel} for motors (IDs 1..255) ...")
    found = RobstrideBus.scan_channel(channel, start_id=1, end_id=255)
    if not found:
        raise SystemExit("No motors found.")
    ids = sorted(found.keys())
    print(f"Found motors: {ids}")
    return ids


# -------------------- Live plotter (drives control in main thread) --------------------
class LivePlotter:
    def __init__(self, tuner: GainTunerMIT, window_s: float = 10.0, ui_hz: float = 30.0):
        self.tuner = tuner
        self.window_s = float(window_s)
        self.ui_interval_ms = int(1000.0 / float(ui_hz))

        maxlen = int(window_s * ui_hz) + 200
        self.t = deque(maxlen=maxlen)
        self.pos_deg = deque(maxlen=maxlen)
        self.cmd_deg = deque(maxlen=maxlen)
        self.err_deg = deque(maxlen=maxlen)
        self.vel_deg_s = deque(maxlen=maxlen)
        self.tq = deque(maxlen=maxlen)
        self.temp_c = deque(maxlen=maxlen)

        self.fig, self.ax = plt.subplots(5, 1, sharex=True, figsize=(10, 9))
        try:
            self.fig.canvas.manager.set_window_title("RobStride Gain Tuner - Live Plots")
        except Exception:
            pass

        (self.l_pos,) = self.ax[0].plot([], [], label="pos (deg)")
        (self.l_cmd,) = self.ax[0].plot([], [], label="cmd (deg)")
        self.ax[0].set_ylabel("deg")
        self.ax[0].legend(loc="upper right")

        (self.l_err,) = self.ax[1].plot([], [], label="err (deg)")
        self.ax[1].set_ylabel("deg")
        self.ax[1].legend(loc="upper right")

        (self.l_vel,) = self.ax[2].plot([], [], label="vel (deg/s)")
        self.ax[2].set_ylabel("deg/s")
        self.ax[2].legend(loc="upper right")

        (self.l_tq,) = self.ax[3].plot([], [], label="torque (Nm)")
        self.ax[3].set_ylabel("Nm")
        self.ax[3].legend(loc="upper right")

        (self.l_temp,) = self.ax[4].plot([], [], label="temp (C)")
        self.ax[4].set_ylabel("C")
        self.ax[4].set_xlabel("time (s)")
        self.ax[4].legend(loc="upper right")

        self._t0 = time.time()
        self._last_update_t = time.time()
        self._accum = 0.0
        self._max_control_iters_per_ui = 20

        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self._ani = None

        # header spacing
        self.fig.subplots_adjust(top=0.90)

    def _on_close(self, _evt):
        try:
            self.tuner.shutdown()
        finally:
            os._exit(0)

    def _choose_motor_to_plot(self) -> int:
        with self.tuner.lock:
            if len(self.tuner.selected) == 1:
                return next(iter(self.tuner.selected))
            if len(self.tuner.selected) > 0:
                return sorted(self.tuner.selected)[0]
            return sorted(self.tuner.motor_states.keys())[0]

    def _wrap_to_pi(self, x: float) -> float:
        return (x + math.pi) % (2.0 * math.pi) - math.pi

    def _estimate_sine_delay(self, freq_hz: float):
        """
        Estimate cmd->pos effective delay at freq_hz using phase lag (least-squares fit).
        Returns (delay_s, phase_lag_rad) or (None, None).
        """
        if freq_hz is None or freq_hz <= 0.0:
            return (None, None)
        if len(self.t) < 60:
            return (None, None)

        t = np.asarray(self.t, dtype=float)
        cmd = np.asarray(self.cmd_deg, dtype=float) * (math.pi / 180.0)
        pos = np.asarray(self.pos_deg, dtype=float) * (math.pi / 180.0)

        cmd = cmd - np.mean(cmd)
        pos = pos - np.mean(pos)

        w = 2.0 * math.pi * float(freq_hz)
        tt = t - t[0]

        S = np.sin(w * tt)
        C = np.cos(w * tt)
        A = np.stack([S, C], axis=1)

        try:
            (a_cmd, b_cmd), *_ = np.linalg.lstsq(A, cmd, rcond=None)
            (a_pos, b_pos), *_ = np.linalg.lstsq(A, pos, rcond=None)
        except Exception:
            return (None, None)

        amp_cmd = math.hypot(a_cmd, b_cmd)
        amp_pos = math.hypot(a_pos, b_pos)
        if amp_cmd < 1e-4 or amp_pos < 1e-4:
            return (None, None)

        phi_cmd = math.atan2(b_cmd, a_cmd)
        phi_pos = math.atan2(b_pos, a_pos)

        phase_lag = self._wrap_to_pi(phi_cmd - phi_pos)
        delay_s = phase_lag / w

        T = 1.0 / float(freq_hz)
        if delay_s < 0.0:
            delay_s += T

        return (delay_s, phase_lag)

    def _append_sample(self, mid: int):
        now_s = time.time() - self._t0

        with self.tuner.lock:
            st = self.tuner.motor_states[mid]
            pos = st.position
            vel = st.velocity
            tq = st.torque
            temp = st.temperature
            cmd = st.commanded_target_rad
            err = cmd - pos
            kp = st.kp
            kd = st.kd
            exmode = st.excitation.mode
            state = st.temp_state
            enabled = st.enabled
            lim_lo, lim_hi = st.limit_lo, st.limit_hi
            motion_scale = self.tuner._motion_scale_from_temp(temp) if state == "DERATE" else 1.0
            model = st.model
            write_ms = st.write_dt_ms
            read_ms = st.read_dt_ms
            iogap_ms = st.io_gap_ms
            loop_ms = st.loop_dt_ms
            step_delay_s = st.last_step_delay_s
            exfreq = st.excitation.freq_hz if st.excitation.mode == "sine" else None

        # ---- store samples for plotting + delay estimation ----
        self.t.append(now_s)
        self.pos_deg.append(math.degrees(pos))
        self.cmd_deg.append(math.degrees(cmd))
        self.err_deg.append(math.degrees(err))
        self.vel_deg_s.append(math.degrees(vel))
        self.tq.append(tq)
        self.temp_c.append(temp)

        step_ms_str = "N/A"
        if not math.isnan(step_delay_s):
            step_ms_str = f"{step_delay_s * 1000.0:.1f} ms"

        sine_ms_str = "N/A"
        phase_deg_str = "N/A"
        if exmode == "sine" and exfreq is not None:
            d_s, ph = self._estimate_sine_delay(exfreq)
            if d_s is not None:
                sine_ms_str = f"{d_s * 1000.0:.1f} ms"
                phase_deg_str = f"{math.degrees(ph):.1f} deg"

        en_str = "EN" if enabled else "DIS"
        self.fig.suptitle(
            f"Motor {mid} ({en_str}) model={model} | kp={kp:.2f} kd={kd:.2f} | ex={exmode}"
            + (f"@{exfreq:.2f}Hz" if exfreq is not None else "")
            + f" | IO: write={write_ms:.2f}ms read={read_ms:.2f}ms gap={iogap_ms:.2f}ms loop={loop_ms:.2f}ms"
            + f" | step_delay={step_ms_str} | sine_delay={sine_ms_str} phase_lag={phase_deg_str}"
            + f" | temp={temp:.1f}C state={state} | motion_scale={motion_scale:.2f}"
            + f" | limits=[{lim_lo:.3f},{lim_hi:.3f}] rad | dir={st.direction:+d}",
            y=0.985,
            fontsize=10.0,
        )

    def _update(self, _frame):
        try:
            # Drive control in main thread
            now = time.time()
            dt_wall = now - self._last_update_t
            self._last_update_t = now
            dt_wall = clamp(dt_wall, 0.0, 0.1)

            self._accum += dt_wall
            iters = 0
            while self._accum >= self.tuner.dt and iters < self._max_control_iters_per_ui:
                self.tuner.control_step(self.tuner.dt)
                self._accum -= self.tuner.dt
                iters += 1
            if iters >= self._max_control_iters_per_ui:
                self._accum = 0.0

            mid = self._choose_motor_to_plot()
            self._append_sample(mid)

            if len(self.t) < 2:
                return (self.l_pos, self.l_cmd, self.l_err, self.l_vel, self.l_tq, self.l_temp)

            x = list(self.t)
            self.l_pos.set_data(x, list(self.pos_deg))
            self.l_cmd.set_data(x, list(self.cmd_deg))
            self.l_err.set_data(x, list(self.err_deg))
            self.l_vel.set_data(x, list(self.vel_deg_s))
            self.l_tq.set_data(x, list(self.tq))
            self.l_temp.set_data(x, list(self.temp_c))

            xmax = x[-1]
            xmin = max(0.0, xmax - self.window_s)
            self.ax[-1].set_xlim(xmin, xmax)

            for a in self.ax:
                a.relim()
                a.autoscale_view(scalex=False, scaley=True)

            return (self.l_pos, self.l_cmd, self.l_err, self.l_vel, self.l_tq, self.l_temp)

        except Exception as e:
            _print_full_exception("[PLOT] Exception in LivePlotter._update (matplotlib callback)", e)
            try:
                self.tuner.shutdown()
            finally:
                # Exit non-silently with code 1 so you see the printed traceback.
                os._exit(1)


    def show(self):
        self._ani = FuncAnimation(
            self.fig,
            self._update,
            interval=self.ui_interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout(rect=[0, 0, 1, 0.94])
        plt.show()


# -------------------- CLI thread --------------------
def command_loop(tuner: GainTunerMIT):
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
                    ids = set()
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
        except Exception as e:
            _print_full_exception("[CLI] Exception while processing command", e)



def main():
    motor_ids = parse_motor_ids_or_scan()

    tuner = GainTunerMIT(
        motor_ids=motor_ids,
        channel="can0",
        bitrate=1_000_000,
        model="rs-03",   # fallback only (per-ID models used automatically)
        hz=60.0,
        ramp_deg_s=30.0,
    )

    def _sig(_signum=None, _frame=None):
        tuner.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    if not tuner.connect():
        sys.exit(1)

    threading.Thread(target=command_loop, args=(tuner,), daemon=True).start()

    plotter = LivePlotter(tuner, window_s=30.0, ui_hz=60.0)
    plotter.show()


if __name__ == "__main__":
    main()
