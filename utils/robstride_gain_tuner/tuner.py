"""RobStride MIT gain tuner core (no plotting or CLI)."""
from __future__ import annotations

import math
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set

from . import config
from .imu import IMUFallDetector
from .logging_utils import clamp, offset_to_pi
from .state import Excitation, MotorState

_REPO_ROOT = Path(__file__).resolve().parents[2]


def _add_sdk_search_paths() -> None:
    """Make the packaged RobStride SDK importable from rosenv or the repo root."""
    py_short = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = (
        _REPO_ROOT / "rosenv" / "lib" / py_short / "site-packages",
        _REPO_ROOT,
    )
    for path in candidates:
        if path.is_dir() and str(path) not in sys.path:
            sys.path.insert(0, str(path))


def get_robstride_sdk():
    """Import the RobStride SDK and return its relevant classes.

    Returns a dict with keys ``bus``, ``motor``, ``parameter_type``,
    ``communication_type``, and ``actuation_safety`` (the latter may be None
    when the optional safety monitor is not installed).
    """
    _add_sdk_search_paths()

    try:
        from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
    except ImportError:
        try:
            from bus import RobstrideBus, Motor
            from protocol import ParameterType, CommunicationType
        except ImportError as exc:
            raise ImportError(
                "Failed to import RobStride SDK. "
                "Run `source rosenv/bin/activate` or install robstride_dynamics."
            ) from exc

    try:
        from utils.actuation_safety import ActuationSafetyMonitor
    except ImportError:
        try:
            from actuation_safety import ActuationSafetyMonitor
        except ImportError:
            ActuationSafetyMonitor = None

    return {
        "bus": RobstrideBus,
        "motor": Motor,
        "parameter_type": ParameterType,
        "communication_type": CommunicationType,
        "actuation_safety": ActuationSafetyMonitor,
    }


class GainTunerMIT:
    """MIT-mode gain tuner with ramping, limits, safety supervisors, and live metrics."""

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
            motor_model = config.MOTOR_MODEL_BY_ID.get(mid, self.model)
            st = MotorState(id=mid, name=f"motor_{mid}", model=motor_model)
            st.direction = int(config.INVERSION_BY_ID.get(mid, 1))
            if mid in config.JOINT_LIMITS:
                st.limit_lo, st.limit_hi = config.JOINT_LIMITS[mid]
            self.motor_states[mid] = st

        self.selected: Set[int] = set(motor_ids)

        self.bus = None
        self.lock = threading.Lock()

        self.running = True
        self.connected = False
        self._last_control_step_t = time.time()
        self.safety_monitor = None
        self.safety_tripped = False
        self.safety_reason: Optional[str] = None
        self.imu_detector: Optional[IMUFallDetector] = None
        self._sdk = None

    # ------------------------------------------------------------------
    # Safety helpers
    # ------------------------------------------------------------------
    def _trip_safety(self, reason: str):
        if self.safety_tripped:
            return
        self.safety_tripped = True
        self.safety_reason = reason
        print(reason)

    def _assert_safe(self):
        if not config.ACTUATION_SAFETY_ENABLED:
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
            pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
            self._update_telemetry_from_raw(st, pos, vel, tq, temp)
            return float(st.position)
        finally:
            self.lock.release()

    def _sanitize_temp_reading(self, st: MotorState, raw_temp: float) -> float:
        """Reject implausible temperature readings."""
        t = float(raw_temp)
        prev = float(st.temperature)
        if (not math.isfinite(t)) or t < config.TEMP_VALID_MIN_C or t > config.TEMP_VALID_MAX_C:
            st.last_error = f"ignored invalid temp reading: {t:.1f}C"
            return prev
        if math.isfinite(prev) and config.TEMP_VALID_MIN_C <= prev <= config.TEMP_VALID_MAX_C:
            if abs(t - prev) > config.TEMP_MAX_STEP_C:
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
        """Trip when torque exceeds the per-model limit for 3 consecutive samples."""
        if not config.TORQUE_SAFETY_ENABLED:
            return

        limit = config.TORQUE_LIMITS_NM.get(st.model, config.TORQUE_DEFAULT_LIMIT_NM)

        if abs(st.torque) > limit:
            st.consecutive_torque_spikes += 1
            if st.consecutive_torque_spikes >= config.TORQUE_CONSECUTIVE_LIMIT:
                self._trip_safety(
                    f"[SAFETY] motor {st.id} ({st.model}) torque spike: "
                    f"{st.torque:.2f} Nm exceeds limit of {limit:.1f} Nm "
                    f"({st.consecutive_torque_spikes} consecutive readings)"
                )
        else:
            st.consecutive_torque_spikes = 0

    def _start_safety_monitor(self):
        if not config.ACTUATION_SAFETY_ENABLED:
            self.safety_monitor = None
            print("[SAFETY] actuation safety monitor disabled for tuner testing.")
            return

        sdk = self._sdk or get_robstride_sdk()
        monitor_cls = sdk["actuation_safety"]
        if monitor_cls is None:
            print("[SAFETY] actuation_safety module not found; joint-limit monitor unavailable.")
            return

        joint_limits = {mid: (st.limit_lo, st.limit_hi) for mid, st in self.motor_states.items()}
        self.safety_monitor = monitor_cls(
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
        if not config.IMU_ENABLED:
            print("[IMU] Fall detection disabled.")
            return
        try:
            from .imu import _import_imu
            _import_imu()
        except ImportError as exc:
            print(f"[IMU] Fall detection unavailable: {exc}")
            return
        self.imu_detector = IMUFallDetector(
            port=config.IMU_SERIAL_PORT,
            halt_fn=self._trip_safety,
            baud=config.IMU_BAUD,
            rate_hz=config.IMU_RATE_HZ,
            fall_roll_deg=config.IMU_FALL_ROLL_DEG,
            fall_pitch_deg=config.IMU_FALL_PITCH_DEG,
            confirm_count=config.IMU_CONFIRM_COUNT,
            use_integrator=config.IMU_USE_INTEGRATOR,
        )
        self.imu_detector.start()

    def _check_ground_contact(self) -> bool:
        """Check ankle/hip torques to verify the robot is weight-bearing."""
        if not config.GROUND_CHECK_ENABLED:
            return True

        warnings = []

        for ankle_id, label in ((config.LEFT_ANKLE_ID, "left"), (config.RIGHT_ANKLE_ID, "right")):
            st = self.motor_states.get(ankle_id)
            if st is None:
                continue
            if abs(st.torque) < config.ANKLE_CONTACT_MIN_TORQUE_NM:
                warnings.append(
                    f"  {label} ankle (ID {ankle_id}): "
                    f"torque={st.torque:.3f} Nm - below contact threshold "
                    f"({config.ANKLE_CONTACT_MIN_TORQUE_NM} Nm)"
                )

        for hip_id, label in ((config.LEFT_HIP_ID, "left"), (config.RIGHT_HIP_ID, "right")):
            st = self.motor_states.get(hip_id)
            if st is None:
                continue
            if abs(st.torque) < config.HIP_CONTACT_MIN_TORQUE_NM:
                warnings.append(
                    f"  {label} hip (ID {hip_id}): "
                    f"torque={st.torque:.3f} Nm - below contact threshold "
                    f"({config.HIP_CONTACT_MIN_TORQUE_NM} Nm)"
                )

        if warnings:
            print("\n[GROUND CHECK] WARNING: Robot may not be weight-bearing:")
            for warning in warnings:
                print(warning)
            print(
                "[GROUND CHECK] Ensure robot is standing on the ground "
                "before commanding motion. Continuing anyway.\n"
            )
            return False

        print("[GROUND CHECK] Ankle and hip torques look nominal - robot appears grounded.")
        return True

    def _clamp_to_limits(self, st: MotorState, logical_rad: float) -> float:
        return clamp(logical_rad, st.limit_lo, st.limit_hi)

    def _within_limits(self, st: MotorState, logical_rad: float) -> bool:
        return st.limit_lo <= float(logical_rad) <= st.limit_hi

    def _set_mode_raw(self, mode: int, motor_id: int):
        sdk = self._sdk or get_robstride_sdk()
        ParameterType = sdk["parameter_type"]
        CommunicationType = sdk["communication_type"]
        motor_name = f"motor_{motor_id}"
        device_id = self.bus.motors[motor_name].id
        param_id, _, _ = ParameterType.MODE  # MODE is int8
        value_buffer = struct.pack("<bBH", int(mode), 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.1)

    def _motion_scale_from_temp(self, temp_c: float) -> float:
        """Motion derating scale for ramp rate (NOT gains)."""
        if temp_c <= config.TEMP_DERATE_START_C:
            return 1.0
        if temp_c >= config.TEMP_DISABLE_C:
            return config.DERATE_MIN_SCALE
        frac = (temp_c - config.TEMP_DERATE_START_C) / (config.TEMP_DISABLE_C - config.TEMP_DERATE_START_C)
        return clamp(1.0 - frac * (1.0 - config.DERATE_MIN_SCALE), config.DERATE_MIN_SCALE, 1.0)

    def _update_temp_state_pre(self, st: MotorState, now: float):
        """Transition temperature state before sending a frame."""
        if not config.TEMP_SAFETY_ENABLED:
            st.temp_state = "OFF"
            return
        t = st.temperature
        if st.temp_state != "DISABLED":
            if t >= config.TEMP_DISABLE_C:
                st.temp_state = "DISABLED"
                st.last_disable_t = now
            elif t >= config.TEMP_HOLD_C:
                st.temp_state = "HOLD"
            elif t >= config.TEMP_DERATE_START_C:
                st.temp_state = "DERATE"
            else:
                st.temp_state = "OK"

    # ------------------------------------------------------------------
    # Connection / disconnection
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            self._sdk = get_robstride_sdk()
            RobstrideBus = self._sdk["bus"]
            Motor = self._sdk["motor"]

            motors = {
                f"motor_{mid}": Motor(id=mid, model=self.motor_states[mid].model)
                for mid in self.motor_states.keys()
            }
            calibration = {
                f"motor_{mid}": {"direction": 1, "homing_offset": 0.0}
                for mid in self.motor_states.keys()
            }

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
                        # Do NOT clamp the initial hold; that could cause motion on connect.
                        st.target_rad = logical
                        st.commanded_target_rad = logical
                        st.hold_center_rad = logical
                        st.bypass_limit_clamp = not self._within_limits(st, logical)

                        if st.bypass_limit_clamp:
                            print(
                                f"  WARN: current logical pos {logical:.4f} rad is outside limits; "
                                f"holding anyway (no motion)."
                            )

                        st.last_error = None
                    except Exception as exc:
                        st.last_error = str(exc)
                        st.target_rad = 0.0
                        st.commanded_target_rad = 0.0
                        st.hold_center_rad = 0.0
                        st.bypass_limit_clamp = False

                    physical_target = self._physical_from_logical(st, st.commanded_target_rad)
                    self.bus.write_operation_frame(st.name, physical_target, st.kp, st.kd, 0.0, 0.0)

                    # Delay-metrics init (avoid arming a fake step on first cycle)
                    st.prev_sent_phys_target = physical_target
                    st.step_pending = False
                    st.last_step_delay_s = math.nan

                    time.sleep(0.05)

            self._check_ground_contact()
            self.connected = True
            self.running = True
            self._start_safety_monitor()
            self._start_imu_detector()
            if not config.TEMP_SAFETY_ENABLED:
                print("[TEMP] thermal safety disabled; temperatures will still be displayed.")
            print("Connected. Motors are holding their current position (no motion).")
            return True

        except Exception as exc:
            print(f"Connection failed: {exc}")
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
        except Exception as exc:
            st.last_error = f"disable failed: {exc}"

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

            st.prev_sent_phys_target = physical_target
            st.step_pending = False
            st.last_step_delay_s = math.nan

            st.enabled = True
            st.temp_state = "HOLD"  # come back in HOLD; user/policy can move again when cool
            print(f"[TEMP] RE-ENABLED motor {st.id} at {st.temperature:.1f}C (state=HOLD)")
        except Exception as exc:
            st.last_error = f"reenable failed: {exc}"

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_step(self, dt: float):
        """One control cycle (excitation + ramp + send + read).

        Called from the matplotlib animation callback on the main thread.
        """
        if not (self.running and self.connected and self.bus):
            return
        self._assert_safe()

        dt = float(clamp(dt, 0.0, 0.05))
        now = time.time()

        # Loop timing (jitter indicator)
        loop_dt = now - self._last_control_step_t
        self._last_control_step_t = now

        with self.lock:
            for st in self.motor_states.values():
                st.loop_dt_ms = float(loop_dt) * 1000.0

            # Temperature state update (pre-send)
            for st in self.motor_states.values():
                self._update_temp_state_pre(st, now)

            # Handle DISABLED transitions / auto re-enable
            if config.TEMP_SAFETY_ENABLED:
                for st in self.motor_states.values():
                    if st.temp_state == "DISABLED":
                        if st.enabled:
                            self._disable_motor_locked(st, now)
                    else:
                        if (not st.enabled) and (st.temperature <= config.TEMP_REENABLE_C) and (
                            (now - st.last_disable_t) >= config.DISABLE_COOLDOWN_S
                        ):
                            self._reenable_motor_locked(st, now)

            # Handle HOLD state: freeze target at current pose (no excitation)
            if config.TEMP_SAFETY_ENABLED:
                for st in self.motor_states.values():
                    if st.temp_state == "HOLD":
                        st.excitation = Excitation()
                        logical = st.position
                        st.target_rad = logical
                        st.commanded_target_rad = logical
                        st.hold_center_rad = logical
                        st.bypass_limit_clamp = not self._within_limits(st, logical)

            # Excitation + clamp + ramp
            for st in self.motor_states.values():
                if not st.enabled:
                    continue  # disabled: don't update commands

                ex = st.excitation
                if ex.mode == "sine" and (not config.TEMP_SAFETY_ENABLED or st.temp_state != "HOLD"):
                    if ex.duration_s is not None and (now - ex.t0) >= ex.duration_s:
                        st.excitation = Excitation()
                        st.target_rad = st.hold_center_rad
                    else:
                        st.target_rad = ex.center_rad + ex.amp_rad * math.sin(
                            2.0 * math.pi * ex.freq_hz * (now - ex.t0)
                        )

                # Enforce joint limits in logical space for any motion command
                if not st.bypass_limit_clamp:
                    st.target_rad = self._clamp_to_limits(st, st.target_rad)

                # Ramp (derate motion if hot)
                motion_scale = 1.0
                if config.TEMP_SAFETY_ENABLED and st.temp_state == "DERATE":
                    motion_scale = self._motion_scale_from_temp(st.temperature)

                max_step = self.ramp_rad_s_nominal * dt * motion_scale
                delta = st.target_rad - st.commanded_target_rad
                if abs(delta) <= max_step:
                    st.commanded_target_rad = st.target_rad
                else:
                    st.commanded_target_rad += math.copysign(max_step, delta)

                if not st.bypass_limit_clamp:
                    st.commanded_target_rad = self._clamp_to_limits(st, st.commanded_target_rad)

            # Send frames (fixed gains; RL-safe) + IO timing + step-delay arming
            for st in self.motor_states.values():
                if not st.enabled:
                    continue
                try:
                    physical_target = self._physical_from_logical(st, st.commanded_target_rad)
                    arm_step = (
                        st.excitation.mode != "sine"
                        and (not st.step_pending)
                        and abs(physical_target - st.prev_sent_phys_target) >= config.STEP_CMD_EPS_RAD
                    )

                    t0 = time.perf_counter()
                    self.bus.write_operation_frame(
                        st.name, physical_target, float(st.kp), float(st.kd), 0.0, 0.0
                    )
                    t1 = time.perf_counter()

                    st.write_dt_ms = (t1 - t0) * 1000.0
                    st.last_write_end_t = time.time()

                    if arm_step:
                        st.step_pending = True
                        st.step_cmd_t = st.last_write_end_t
                        st.step_pos0 = st.position
                        st.last_step_delay_s = math.nan

                    st.prev_sent_phys_target = physical_target

                except Exception as exc:
                    if "No response" not in str(exc):
                        st.last_error = str(exc)

            # Read frames + IO timing + step-delay completion
            for st in self.motor_states.values():
                if not st.enabled:
                    continue
                try:
                    t0 = time.perf_counter()
                    pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
                    t1 = time.perf_counter()

                    st.read_dt_ms = (t1 - t0) * 1000.0
                    st.last_read_end_t = time.time()

                    self._update_telemetry_from_raw(st, pos, vel, tq, temp)
                    self._check_torque_spikes(st)
                    st.last_error = None

                    st.consecutive_read_failures = 0

                    if st.last_write_end_t > 0.0:
                        st.io_gap_ms = (st.last_read_end_t - st.last_write_end_t) * 1000.0

                    if st.step_pending:
                        if (st.last_read_end_t - st.step_cmd_t) > config.STEP_TIMEOUT_S:
                            st.step_pending = False
                        else:
                            if abs(st.position - st.step_pos0) >= config.STEP_POS_EPS_RAD:
                                st.last_step_delay_s = st.last_read_end_t - st.step_cmd_t
                                st.step_pending = False

                except Exception as exc:
                    st.consecutive_read_failures += 1

                    if (
                        config.COMMS_LOSS_ENABLED
                        and st.consecutive_read_failures >= config.COMMS_MAX_CONSECUTIVE_FAILURES
                    ):
                        self._trip_safety(
                            f"[SAFETY] motor {st.id} communication lost: "
                            f"{st.consecutive_read_failures} consecutive read failures. "
                            f"Last error: {exc}"
                        )
                    elif "No response" not in str(exc):
                        st.last_error = str(exc)

            # Immediate post-read critical check (affects next cycle)
            if config.TEMP_SAFETY_ENABLED:
                for st in self.motor_states.values():
                    if st.enabled and st.temperature >= config.TEMP_DISABLE_C:
                        st.temp_state = "DISABLED"
                        st.last_disable_t = now
                        self._disable_motor_locked(st, now)

    # ------------------------------------------------------------------
    # User commands
    # ------------------------------------------------------------------
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
            print(f"[WARN] Motor {motor_id} has last_error: {st.last_error}")

        print(f"Selected motor: {motor_id}")

    def invert(self, ids: Set[int]):
        """Toggle mount direction immediately, then re-hold to avoid motion."""
        with self.lock:
            for mid in ids:
                if mid not in self.motor_states:
                    continue
                st = self.motor_states[mid]
                st.direction *= -1
                st.startup_motor_offset_rad = offset_to_pi(st.raw_position / float(st.direction))
                st.position = self._logical_from_raw(st, st.raw_position)
                st.velocity *= -1.0
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
                    f"\n[IMU] port={config.IMU_SERIAL_PORT} "
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
