"""
------------
1) Streaming interface:
   - iter_imu_samples(...) yields dict samples at a requested rate.
   - Supports serial CSV format:
       t_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, roll_deg, pitch_deg, (temp_C optional)
   - Supports I2C mode via a local MPU6050Reader that returns (acc_ms2, gyro_rads).
   - Supports CAN request/reply mode using the 0x100/0x101/0x102 IMU protocol.

2) RK4 attitude integration (quaternion):
   - Integrates orientation using gyro angular rate with RK4.
   - Outputs:
       q_xyzw      : orientation quaternion (x, y, z, w)
       rpy_deg/rad : roll/pitch/yaw (for convenience)
       up_body     : world +Z expressed in the body frame (tilt indicator)

3) Conservative tilt drift suppression (Mahony-style, roll/pitch only):
   - The accelerometer is used as a gravity direction measurement ONLY when a
     "dual-gate" test passes:
       Gate A: accel magnitude close to 1g  ->  abs(|a|/g - 1) < acc_gate_g
       Gate B: low jerk (small accel change)->  |(a_now - a_prev)/dt| < jerk_gate_ms3
     Optional Gate C: low angular rate      ->  |gyro| < gyro_gate_dps
   - Additionally, the gate must pass for N consecutive samples (gate_min_count)
     before correction is applied. This prevents one-off spikes from triggering
     correction.
   - When gated, a Mahony-style correction is applied:
       omega_corrected = omega + kp_acc * (up_pred x up_meas)
     which pulls roll/pitch back toward gravity. Yaw is not corrected because
     gravity does not observe yaw.

4) Still detection and gyro bias learning (optional):
   - Independently detects "stationary" periods using accel magnitude + gyro
     magnitude thresholds.
   - When stationary, a slow exponential update estimates gyro bias to reduce drift.
   - IMPORTANT: This implementation intentionally does NOT reset the quaternion
     to identity when stationary (no "hard reset to (0,0,0,1)"), to avoid sudden
     discontinuities in orientation estimates.

Notes / Assumptions
-------------------
- Accelerometer is assumed to include gravity. If the platform experiences strong
  linear accelerations, the gate should prevent using accel as a gravity reference.
- For best results, calibrate sensor offsets/scales and ensure consistent axis
  conventions between firmware and this code.
- This module focuses on robust tilt (roll/pitch) and an up vector; it is not
  meant to provide accurate long-term position without external aiding.
- Translation is integrated in body coordinates after subtracting gravity in
  body frame; this intentionally ignores world-frame transport due to rotation.
- Treat lin_vel_ms / lin_pos_m as stationary-only diagnostics; during real
  motion, accel/gravity leakage can dominate them quickly.

Typical Usage
-------------
Create an RK4DeadReckoner and pass it into iter_imu_samples(..., integrator=...),
then read s["up_body"] or s["rpy_deg"] to monitor tilt. Enable include_all=True
to inspect gate diagnostics (acc_gate_err_g, jerk_ms3, gate_ok, etc.).
"""
from __future__ import annotations
import math
import time
from typing import Dict, Iterator, Optional, Sequence, Tuple, Any


# --------------------
# constants
# --------------------
G = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

_SKIP_PREFIXES = (
    "t_ms,", "serial_ok", "Using SDA=", "ping_", "MPU found", "No MPU found",
    "Write PWR", "read_fail", "#"
)


# --------------------
# small vector helpers (no numpy dependency)
# --------------------
def v_add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def v_sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def v_mul(s, a): return (s * a[0], s * a[1], s * a[2])
def v_norm(a): return math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])


def v_rotate_xy_cw_90(a):
    # Clockwise 90 deg rotation in the XY plane: [x'; y'] = [[0, 1], [-1, 0]] [x; y]
    return (a[1], -a[0], a[2])


def be_i16(buf: bytes, offset: int) -> int:
    return int.from_bytes(buf[offset:offset + 2], byteorder="big", signed=True)

def v_cross(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )

def s_add(a, b):  # 6D state add
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3], a[4]+b[4], a[5]+b[5])

def s_mul(s, a):  # 6D state scale
    return (s*a[0], s*a[1], s*a[2], s*a[3], s*a[4], s*a[5])


# --------------------
# quaternion (x,y,z,w) helpers
# --------------------
def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / n
    return (x*inv, y*inv, z*inv, w*inv)

def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def quat_rotate(q, v):
    # v' = q ⊗ [v,0] ⊗ conj(q)
    x, y, z, w = quat_normalize(q)
    vq = (v[0], v[1], v[2], 0.0)
    q_conj = (-x, -y, -z, w)
    out = quat_mul(quat_mul((x, y, z, w), vq), q_conj)
    return (out[0], out[1], out[2])

def quat_rotate_inv(q, v):
    # inverse rotation: world -> body
    x, y, z, w = quat_normalize(q)
    q_conj = (-x, -y, -z, w)
    vq = (v[0], v[1], v[2], 0.0)
    out = quat_mul(quat_mul(q_conj, vq), (x, y, z, w))
    return (out[0], out[1], out[2])

def quat_to_rpy(q):
    x, y, z, w = quat_normalize(q)

    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quat_xyzw_to_up_body(q):
    """
    up_body = R_wb^T * [0,0,1] (world +Z expressed in body frame)
    """
    x, y, z, w = quat_normalize(q)
    return (
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


# --------------------
# Integrator (RK4)
# --------------------
class RK4DeadReckoner:
    """
    RK4 dead-reckoning integrator.

    State:
      - q_xyzw: orientation quaternion (x,y,z,w)
      - v: linear velocity in body [m/s]
      - p: linear position integrated in body axes [m]
        Use v/p only near stationary periods; they are not trustworthy for
        general robot motion without external aiding.

    Still detection & bias:
      - stationary when accel/gyro stay within stillness thresholds long enough
        to pass hysteresis + consecutive-sample confirmation
      - if stationary: gyro_bias <- (1-a)*bias + a*gyro_raw
      - if stationary and enable_zupt: set v = 0

    Attitude drift suppression:
      - Dual gate (""):
          1) | |acc|-1g | < acc_gate_g
          2) jerk = |(acc_now - acc_prev)/dt| < jerk_gate_ms3
        + optional gyro gate:
          3) |gyro| < gyro_gate_dps
      - Require
      - Mahony-style correction: omega += kp * (up_pred x up_meas)  (body-frame)
        (only roll/pitch are observable; yaw is free)
    """

    def __init__(
        self,
        *,
        gravity_world: Tuple[float, float, float] = (0.0, 0.0, 9.80665),
        acc_includes_gravity: bool = True,
        integrate_translation: bool = False,

        enable_zupt: bool = True,
        zupt_acc_g: float = 0.05,
        zupt_gyro_dps: float = 2.0,
        stationary_sensitivity_scale: float = 1.5,
        stationary_release_ratio: float = 1.25,
        stationary_min_count: int = 5,
        debug_stationary: bool = False,
        alpha_gyro_bias: float = 0.01,
        max_dt: float = 0.2,
        use_board_time_if_available: bool = True,

        # ---- acc correction ----
        enable_acc_correction: bool = True,
        acc_gate_g: float = 0.05,          # tighter default
        jerk_gate_ms3: float = 0.5,        # your current preference (very strict)
        enable_gyro_gate: bool = True,
        gyro_gate_dps: float = 8.0,
        gate_min_count: int = 5,           # consecutive frames required

        kp_acc: float = 2.0,               # proportional correction strength (rad/s)
        ki_acc: float = 0.0,               # optional: integrate into bias (keep 0 first)
        debug_acc_gate: bool = False,
    ):
        self.gw = gravity_world
        self.acc_includes_gravity = acc_includes_gravity
        self.integrate_translation = integrate_translation

        self.enable_zupt = enable_zupt
        self.zupt_acc_g = zupt_acc_g
        self.zupt_gyro_dps = zupt_gyro_dps
        self.stationary_sensitivity_scale = max(0.1, stationary_sensitivity_scale)
        self.stationary_release_ratio = max(1.0, stationary_release_ratio)
        self.stationary_min_count = max(1, int(stationary_min_count))
        self.debug_stationary = debug_stationary
        self.alpha_bias = alpha_gyro_bias
        self.max_dt = max_dt
        self.use_board_time_if_available = use_board_time_if_available

        # acc correction params
        self.enable_acc_correction = enable_acc_correction
        self.acc_gate_g = max(0.001, acc_gate_g)
        self.jerk_gate_ms3 = max(0.01, jerk_gate_ms3)
        self.enable_gyro_gate = enable_gyro_gate
        self.gyro_gate_dps = max(0.1, gyro_gate_dps)
        self.gate_min_count = max(1, int(gate_min_count))
        self._gate_count = 0

        self.kp_acc = max(0.0, kp_acc)
        self.ki_acc = max(0.0, ki_acc)
        self.debug_acc_gate = debug_acc_gate

        self.q = (0.0, 0.0, 0.0, 1.0)
        self.v = (0.0, 0.0, 0.0)
        self.p = (0.0, 0.0, 0.0)
        self.gyro_bias_rads = (0.0, 0.0, 0.0)

        self._t_prev: Optional[float] = None
        self._omega_prev_raw: Optional[Tuple[float, float, float]] = None
        self._a_prev_body: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._stationary_prev: bool = False
        self._stationary_count = 0

        # for jerk gate
        self._acc_prev_ms2: Optional[Tuple[float, float, float]] = None

    def _q_dot(self, q, omega_rads):
        ox, oy, oz = omega_rads
        omega_q = (ox, oy, oz, 0.0)
        dq = quat_mul(q, omega_q)
        return (0.5*dq[0], 0.5*dq[1], 0.5*dq[2], 0.5*dq[3])

    def _integrate_quat_rk4(self, q0, omega0, omega1, dt):
        def omega_of_tau(tau):
            if dt <= 0:
                return omega1
            k = tau / dt
            return (
                omega0[0] + (omega1[0]-omega0[0])*k,
                omega0[1] + (omega1[1]-omega0[1])*k,
                omega0[2] + (omega1[2]-omega0[2])*k,
            )

        k1 = self._q_dot(q0, omega_of_tau(0.0))
        q1 = (q0[0] + 0.5*dt*k1[0], q0[1] + 0.5*dt*k1[1], q0[2] + 0.5*dt*k1[2], q0[3] + 0.5*dt*k1[3])

        k2 = self._q_dot(q1, omega_of_tau(0.5*dt))
        q2 = (q0[0] + 0.5*dt*k2[0], q0[1] + 0.5*dt*k2[1], q0[2] + 0.5*dt*k2[2], q0[3] + 0.5*dt*k2[3])

        k3 = self._q_dot(q2, omega_of_tau(0.5*dt))
        q3 = (q0[0] + dt*k3[0], q0[1] + dt*k3[1], q0[2] + dt*k3[2], q0[3] + dt*k3[3])

        k4 = self._q_dot(q3, omega_of_tau(dt))

        q_new = (
            q0[0] + (dt/6.0)*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
            q0[1] + (dt/6.0)*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1]),
            q0[2] + (dt/6.0)*(k1[2] + 2*k2[2] + 2*k3[2] + k4[2]),
            q0[3] + (dt/6.0)*(k1[3] + 2*k2[3] + 2*k3[3] + k4[3]),
        )
        return quat_normalize(q_new)

    def _integrate_pv_rk4(self, p0, v0, a0, a1, dt):
        # state s = [p(3), v(3)]
        def a_of_tau(tau):
            if dt <= 0:
                return a1
            k = tau / dt
            return (
                a0[0] + (a1[0]-a0[0])*k,
                a0[1] + (a1[1]-a0[1])*k,
                a0[2] + (a1[2]-a0[2])*k,
            )

        def f(s, tau):
            v = (s[3], s[4], s[5])
            a = a_of_tau(tau)
            return (v[0], v[1], v[2], a[0], a[1], a[2])

        s0 = (p0[0], p0[1], p0[2], v0[0], v0[1], v0[2])
        k1 = f(s0, 0.0)
        k2 = f(s_add(s0, s_mul(0.5*dt, k1)), 0.5*dt)
        k3 = f(s_add(s0, s_mul(0.5*dt, k2)), 0.5*dt)
        k4 = f(s_add(s0, s_mul(dt, k3)), dt)

        s1 = s_add(s0, s_mul(dt/6.0, s_add(s_add(k1, s_mul(2.0, k2)), s_add(s_mul(2.0, k3), k4))))
        p1 = (s1[0], s1[1], s1[2])
        v1 = (s1[3], s1[4], s1[5])
        return p1, v1

    def update(self, sample: Dict[str, Any]) -> Dict[str, Any]:
        # pick time
        if self.use_board_time_if_available and (sample.get("t_s") is not None):
            t = float(sample["t_s"])
        elif sample.get("host_time_s") is not None:
            t = float(sample["host_time_s"])
        else:
            t = time.time()

        # get acc / gyro in SI
        if sample.get("acc_ms2") is not None:
            acc_ms2 = tuple(sample["acc_ms2"])
        else:
            ax, ay, az = sample["acc_g"]
            acc_ms2 = (ax*G, ay*G, az*G)

        if sample.get("gyro_rads") is not None:
            gyro_rads_raw = tuple(sample["gyro_rads"])
        else:
            gx, gy, gz = sample["gyro_dps"]
            gyro_rads_raw = (gx*DEG2RAD, gy*DEG2RAD, gz*DEG2RAD)

        # still detection in g/dps space
        axg, ayg, azg = (acc_ms2[0]/G, acc_ms2[1]/G, acc_ms2[2]/G)
        gxd, gyd, gzd = (gyro_rads_raw[0]*RAD2DEG, gyro_rads_raw[1]*RAD2DEG, gyro_rads_raw[2]*RAD2DEG)
        a_mag_g = math.sqrt(axg*axg + ayg*ayg + azg*azg)
        gyro_mag_dps = math.sqrt(gxd*gxd + gyd*gyd + gzd*gzd)
        acc_err_g = abs(a_mag_g - 1.0)

        enter_acc = self.zupt_acc_g * self.stationary_sensitivity_scale
        enter_gyro = self.zupt_gyro_dps * self.stationary_sensitivity_scale
        exit_acc = enter_acc * self.stationary_release_ratio
        exit_gyro = enter_gyro * self.stationary_release_ratio

        was_stationary = self._stationary_prev
        if was_stationary:
            stationary_candidate = (acc_err_g < exit_acc) and (gyro_mag_dps < exit_gyro)
            stationary = stationary_candidate
        else:
            stationary_candidate = (acc_err_g < enter_acc) and (gyro_mag_dps < enter_gyro)
            stationary = False

        if stationary_candidate:
            self._stationary_count = min(self._stationary_count + 1, 1_000_000)
        else:
            self._stationary_count = 0

        if not was_stationary:
            stationary = self._stationary_count >= self.stationary_min_count

        self._stationary_prev = stationary

        if self.debug_stationary:
            print(
                f"[stationary] cand={stationary_candidate} s={stationary} prev={was_stationary} "
                f"cnt={self._stationary_count}/{self.stationary_min_count} "
                f"acc_err_g={acc_err_g:.4f} gyro_mag_dps={gyro_mag_dps:.3f} "
                f"enter(acc={enter_acc:.4f},gyro={enter_gyro:.3f}) "
                f"exit(acc={exit_acc:.4f},gyro={exit_gyro:.3f})"
            )

        # init
        if self._t_prev is None:
            self._t_prev = t
            self._omega_prev_raw = gyro_rads_raw
            self._a_prev_body = (0.0, 0.0, 0.0)
            self._acc_prev_ms2 = acc_ms2
            return {
                "dt_s": None,
                "stationary": stationary,
                "q_xyzw": self.q,
                "rpy_rad": quat_to_rpy(self.q),
                "rpy_deg": tuple(a*RAD2DEG for a in quat_to_rpy(self.q)),
                "up_body": quat_xyzw_to_up_body(self.q),
                "lin_vel_ms": self.v,
                "lin_pos_m": self.p,
                "acc_world_ms2": None,
                "acc_lin_body_ms2": None,
                "acc_lin_world_ms2": None,
                "gyro_bias_rads": self.gyro_bias_rads,
                "acc_gate_ok": False,
                "acc_gate_err_g": acc_err_g,
                "jerk_ms3": None,
                "jerk_gate_ok": False,
                "gyro_gate_ok": False,
            }

        dt = t - self._t_prev
        self._t_prev = t
        if dt <= 0.0 or dt > self.max_dt:
            self._omega_prev_raw = gyro_rads_raw
            self._acc_prev_ms2 = acc_ms2
            return {"dt_s": dt, "stationary": stationary}

        # gyro bias update (still-based) — keep this, but NO reset to q=identity
        if stationary:
            bx, by, bz = self.gyro_bias_rads
            ox, oy, oz = gyro_rads_raw
            a = self.alpha_bias
            self.gyro_bias_rads = ((1-a)*bx + a*ox, (1-a)*by + a*oy, (1-a)*bz + a*oz)

        # bias-corrected omega
        omega1 = v_sub(gyro_rads_raw, self.gyro_bias_rads)
        omega0 = v_sub(self._omega_prev_raw, self.gyro_bias_rads) if self._omega_prev_raw is not None else omega1

        # --------------------
        # Dual gate + consecutive count + tilt correction
        # --------------------
        acc_gate_ok = False
        jerk_gate_ok = False
        gyro_gate_ok = False
        jerk_ms3 = None

        a_mag = v_norm(acc_ms2)
        acc_gate_err_g = abs((a_mag / G) - 1.0) if a_mag > 1e-9 else 999.0
        if a_mag > 1e-9 and acc_gate_err_g < self.acc_gate_g:
            acc_gate_ok = True

        if self._acc_prev_ms2 is not None and dt > 1e-6:
            da = v_sub(acc_ms2, self._acc_prev_ms2)
            jerk_ms3 = v_norm(da) / dt
            if jerk_ms3 < self.jerk_gate_ms3:
                jerk_gate_ok = True

        if (not self.enable_gyro_gate) or (gyro_mag_dps < self.gyro_gate_dps):
            gyro_gate_ok = True

        gate_now = (
            self.enable_acc_correction
            and (self.kp_acc > 0.0)
            and acc_gate_ok
            and jerk_gate_ok
            and gyro_gate_ok
        )

        if gate_now:
            self._gate_count = min(self._gate_count + 1, 1_000_000)
        else:
            self._gate_count = 0

        gate_ok = (self._gate_count >= self.gate_min_count)

        if self.debug_acc_gate:
            print(
                f"[acc_gate] now={gate_now} ok={gate_ok} cnt={self._gate_count}/{self.gate_min_count} "
                f"mag_err_g={acc_gate_err_g:.3f}(<{self.acc_gate_g:.3f}) "
                f"jerk={('%.2f'%jerk_ms3) if jerk_ms3 is not None else 'None'}(<{self.jerk_gate_ms3:.2f}) "
                f"gyro={gyro_mag_dps:.2f}(<{self.gyro_gate_dps:.2f} if enabled) "
                f"kp={self.kp_acc:.2f} ki={self.ki_acc:.2f}"
            )

        if gate_ok:
            # measured up in body ~ acc direction (assumes acc includes gravity)
            up_meas = v_mul(1.0 / a_mag, acc_ms2)
            up_pred = quat_xyzw_to_up_body(self.q)
            # Measured-vs-predicted order matters here; the opposite sign drives
            # the estimate toward an upside-down equilibrium when tilted.
            e = v_cross(up_meas, up_pred)  # body-frame error axis

            omega0 = v_add(omega0, v_mul(self.kp_acc, e))
            omega1 = v_add(omega1, v_mul(self.kp_acc, e))

            if self.ki_acc > 0.0:
                bx, by, bz = self.gyro_bias_rads
                self.gyro_bias_rads = (
                    bx - self.ki_acc * e[0] * dt,
                    by - self.ki_acc * e[1] * dt,
                    bz - self.ki_acc * e[2] * dt,
                )

        # attitude RK4
        self.q = self._integrate_quat_rk4(self.q, omega0, omega1, dt)
        self._acc_prev_ms2 = acc_ms2

        acc_world = None
        a_lin_body = None
        a_lin_world = None
        if self.integrate_translation:
            acc_world = quat_rotate(self.q, acc_ms2)
            gravity_body = quat_rotate_inv(self.q, self.gw)
            if self.acc_includes_gravity:
                a_lin_body = v_sub(acc_ms2, gravity_body)
                a_lin_world = v_sub(acc_world, self.gw)
            else:
                a_lin_body = acc_ms2
                a_lin_world = acc_world

            # Translation is only trustworthy near stationary conditions; during
            # dynamic motion, small accel/gravity errors integrate into large drift.
            if stationary and self.enable_zupt:
                self.v = (0.0, 0.0, 0.0)
            else:
                self.p, self.v = self._integrate_pv_rk4(self.p, self.v, self._a_prev_body, a_lin_body, dt)
            self._a_prev_body = a_lin_body
        else:
            self.v = (0.0, 0.0, 0.0)
            self.p = (0.0, 0.0, 0.0)
            self._a_prev_body = (0.0, 0.0, 0.0)

        self._omega_prev_raw = gyro_rads_raw

        rpy = quat_to_rpy(self.q)
        up_body = quat_xyzw_to_up_body(self.q)

        return {
            "dt_s": dt,
            "stationary": stationary,
            "q_xyzw": self.q,
            "rpy_rad": rpy,
            "rpy_deg": (rpy[0]*RAD2DEG, rpy[1]*RAD2DEG, rpy[2]*RAD2DEG),
            "up_body": up_body,
            "lin_vel_ms": self.v,
            "lin_pos_m": self.p,
            "acc_world_ms2": acc_world,
            "acc_lin_body_ms2": a_lin_body,
            "acc_lin_world_ms2": a_lin_world,
            "gyro_bias_rads": self.gyro_bias_rads,

            # debug gates
            "acc_gate_ok": acc_gate_ok,
            "acc_gate_err_g": acc_gate_err_g,
            "jerk_ms3": jerk_ms3,
            "jerk_gate_ok": jerk_gate_ok,
            "gyro_gate_ok": gyro_gate_ok,
            "gate_count": self._gate_count,
            "gate_ok": gate_ok,
        }


# --------------------
# parsing
# --------------------
def _should_skip(line: str) -> bool:
    s = line.strip()
    if not s:
        return True
    return any(s.startswith(p) for p in _SKIP_PREFIXES)

def parse_arduino_imu_csv(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse one Arduino/ESP32 CSV line:
      t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,temp_C
    temp_C optional. Returns None if header/status/invalid.
    """
    if _should_skip(line):
        return None
    parts = [p.strip() for p in line.strip().split(",")]
    if len(parts) < 9:
        return None
    try:
        t_ms = float(parts[0])
        ax_g, ay_g, az_g = float(parts[1]), float(parts[2]), float(parts[3])
        gx_dps, gy_dps, gz_dps = float(parts[4]), float(parts[5]), float(parts[6])
        roll_deg, pitch_deg = float(parts[7]), float(parts[8])
        temp_C = float(parts[9]) if len(parts) >= 10 else None

        acc_g = v_rotate_xy_cw_90((ax_g, ay_g, az_g))
        gyro_dps = v_rotate_xy_cw_90((gx_dps, gy_dps, gz_dps))
        acc_ms2 = v_rotate_xy_cw_90((ax_g * G, ay_g * G, az_g * G))
        gyro_rads = v_rotate_xy_cw_90((gx_dps * DEG2RAD, gy_dps * DEG2RAD, gz_dps * DEG2RAD))

        return {
            "source": "serial_csv",
            "t_ms": t_ms,
            "t_s": t_ms * 1e-3,
            "acc_g": acc_g,
            "gyro_dps": gyro_dps,
            "acc_ms2": acc_ms2,
            "gyro_rads": gyro_rads,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "temp_C": temp_C,
            "acc_norm_g": v_norm(acc_g),
            "gyro_norm_dps": v_norm(gyro_dps),
            "raw": line.strip(),
        }
    except ValueError:
        return None


def decode_can_imu_frames(rsp1: bytes, rsp2: bytes) -> Dict[str, Any]:
    ax = be_i16(rsp1, 0)
    ay = be_i16(rsp1, 2)
    az = be_i16(rsp1, 4)
    gx = be_i16(rsp1, 6)
    gy = be_i16(rsp2, 0)
    gz = be_i16(rsp2, 2)
    seq = rsp2[4]

    return {
        "seq": seq,
        "ax_raw": ax,
        "ay_raw": ay,
        "az_raw": az,
        "gx_raw": gx,
        "gy_raw": gy,
        "gz_raw": gz,
        "acc_g": (ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G),
        "gyro_dps": (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS),
    }


def recv_can_until(bus: Any, arb_id: int, timeout_s: float) -> Optional[Any]:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None or msg.is_extended_id:
            continue
        if msg.arbitration_id == arb_id:
            return msg
    return None

def _select_keys(full: Dict[str, Any], keys: Optional[Sequence[str]], include_all: bool) -> Dict[str, Any]:
    if include_all or keys is None:
        return full
    out: Dict[str, Any] = {}
    for k in keys:
        if k in full:
            out[k] = full[k]
    return out


# --------------------
# main generator
# --------------------
def iter_imu_samples(
    *,
    source: str = "serial",   # "serial", "i2c", or "can"
    # serial params
    port: str = "COM13",
    baud: int = 115200,
    timeout: float = 1.0,
    # i2c params
    i2c_bus: int = 1,
    i2c_addr: int = 0x68,
    # can params
    can_interface: str = "socketcan",
    can_channel: str = "can0",
    can_bitrate: int = 500000,
    can_timeout: float = 0.2,
    can_req_id: int = 0x100,
    can_rsp1_id: int = 0x101,
    can_rsp2_id: int = 0x102,
    # output control
    keys: Optional[Sequence[str]] = ("acc_g", "gyro_dps"),
    include_all: bool = False,
    add_host_time: bool = True,
    # output rate
    rate_hz: Optional[float] = None,
    # integrator
    integrator: Optional[RK4DeadReckoner] = None,
) -> Iterator[Dict[str, Any]]:
    source = source.lower().strip()
    if source not in ("serial", "i2c", "can"):
        raise ValueError("source must be 'serial', 'i2c', or 'can'")

    period: Optional[float] = None
    if rate_hz is not None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0 or None")
        period = 1.0 / float(rate_hz)

    def should_emit(now_s: float, next_emit_s: Optional[float]) -> Tuple[bool, Optional[float]]:
        if period is None:
            return True, next_emit_s
        if next_emit_s is None:
            return True, now_s + period
        if now_s >= next_emit_s:
            return True, now_s + period
        return False, next_emit_s

    if source == "serial":
        import serial  # pyserial
        ser = serial.Serial(port, baud, timeout=timeout)
        next_emit_s: Optional[float] = None
        try:
            time.sleep(0.5)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                full = parse_arduino_imu_csv(line)
                if full is None:
                    continue

                now = time.time()
                emit, next_emit_s = should_emit(now, next_emit_s)
                if not emit:
                    continue

                if add_host_time:
                    full["host_time_s"] = now

                if integrator is not None:
                    full.update(integrator.update(full))

                yield _select_keys(full, keys, include_all)
        finally:
            try:
                ser.close()
            except Exception:
                pass

    elif source == "i2c":
        from imu_i2c_reader import MPU6050Reader  # needs local file
        reader = MPU6050Reader(bus_id=i2c_bus, addr=i2c_addr)
        next_emit_s: Optional[float] = None
        try:
            while True:
                if period is not None:
                    now = time.time()
                    if next_emit_s is None:
                        next_emit_s = now
                    sleep_s = next_emit_s - now
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                acc_ms2, gyro_rads = reader.read()
                acc_ms2 = v_rotate_xy_cw_90(acc_ms2)
                gyro_rads = v_rotate_xy_cw_90(gyro_rads)

                ax, ay, az = acc_ms2
                gx, gy, gz = gyro_rads

                acc_g = (ax / G, ay / G, az / G)
                gyro_dps = (gx * RAD2DEG, gy * RAD2DEG, gz * RAD2DEG)

                now = time.time()
                if period is not None:
                    next_emit_s = now + period

                full: Dict[str, Any] = {
                    "source": "i2c",
                    "t_s": now,          # host time
                    "t_ms": None,
                    "host_time_s": now if add_host_time else None,
                    "acc_ms2": (ax, ay, az),
                    "gyro_rads": (gx, gy, gz),
                    "acc_g": acc_g,
                    "gyro_dps": gyro_dps,
                    "roll_deg": None,
                    "pitch_deg": None,
                    "temp_C": None,
                    "acc_norm_g": v_norm(acc_g),
                    "gyro_norm_dps": v_norm(gyro_dps),
                    "raw": None,
                }

                if integrator is not None:
                    full.update(integrator.update(full))

                yield _select_keys(full, keys, include_all)
        finally:
            try:
                reader.close()
            except Exception:
                pass

    else:
        import can

        bus = can.interface.Bus(
            interface=can_interface,
            channel=can_channel,
            bitrate=can_bitrate,
        )
        next_emit_s: Optional[float] = None
        try:
            while True:
                if period is not None:
                    now = time.time()
                    if next_emit_s is None:
                        next_emit_s = now
                    sleep_s = next_emit_s - now
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                req = can.Message(
                    arbitration_id=can_req_id,
                    is_extended_id=False,
                    is_remote_frame=False,
                    data=b"",
                )
                bus.send(req, timeout=can_timeout)

                msg1 = recv_can_until(bus, can_rsp1_id, can_timeout)
                msg2 = recv_can_until(bus, can_rsp2_id, can_timeout)
                if msg1 is None or msg2 is None:
                    continue

                decoded = decode_can_imu_frames(bytes(msg1.data), bytes(msg2.data))
                acc_g = v_rotate_xy_cw_90(decoded["acc_g"])
                gyro_dps = v_rotate_xy_cw_90(decoded["gyro_dps"])
                acc_ms2 = (acc_g[0] * G, acc_g[1] * G, acc_g[2] * G)
                gyro_rads = (gyro_dps[0] * DEG2RAD, gyro_dps[1] * DEG2RAD, gyro_dps[2] * DEG2RAD)

                now = time.time()
                if period is not None:
                    next_emit_s = now + period

                full: Dict[str, Any] = {
                    "source": "can",
                    "t_s": now,
                    "t_ms": None,
                    "host_time_s": now if add_host_time else None,
                    "acc_ms2": acc_ms2,
                    "gyro_rads": gyro_rads,
                    "acc_g": acc_g,
                    "gyro_dps": gyro_dps,
                    "roll_deg": None,
                    "pitch_deg": None,
                    "temp_C": None,
                    "acc_norm_g": v_norm(acc_g),
                    "gyro_norm_dps": v_norm(gyro_dps),
                    "raw": None,
                    "seq": decoded["seq"],
                    "ax_raw": decoded["ax_raw"],
                    "ay_raw": decoded["ay_raw"],
                    "az_raw": decoded["az_raw"],
                    "gx_raw": decoded["gx_raw"],
                    "gy_raw": decoded["gy_raw"],
                    "gz_raw": decoded["gz_raw"],
                }

                if integrator is not None:
                    full.update(integrator.update(full))

                yield _select_keys(full, keys, include_all)
        finally:
            try:
                bus.shutdown()
            except Exception:
                pass


# optional quick demo
if __name__ == "__main__":
    CAN_INTERFACE = "socketcan"
    CAN_CHANNEL = "can0"
    CAN_BITRATE = 500000
    dr = RK4DeadReckoner(
        gravity_world=(0.0, 0.0, 9.80665),
        integrate_translation=False,

        # zupt/bias
        enable_zupt=True,
        stationary_sensitivity_scale=3.0,
        stationary_release_ratio=1.4,
        debug_stationary=False,

        # dual-gate + gyro gate + consecutive
        enable_acc_correction=True,
        acc_gate_g=0.05,
        jerk_gate_ms3=0.5,
        enable_gyro_gate=True,
        gyro_gate_dps=8.0,
        gate_min_count=5,

        kp_acc=2.0,
        ki_acc=0.0,
        debug_acc_gate=False,
    )

    gen = iter_imu_samples(
        source="can",
        can_interface=CAN_INTERFACE,
        can_channel=CAN_CHANNEL,
        can_bitrate=CAN_BITRATE,
        rate_hz=50,
        integrator=dr,
        include_all=True,
    )

    for i, s in zip(range(100000), gen):
        print(
            "up=", s.get("up_body"),
            "rpy=", s.get("rpy_deg"),
            #"gate=", (s.get("acc_gate_ok"), s.get("jerk_gate_ok"), s.get("gyro_gate_ok"), s.get("gate_ok")),
            #"cnt=", s.get("gate_count"),
            #"jerk=", s.get("jerk_ms3"),
        )
