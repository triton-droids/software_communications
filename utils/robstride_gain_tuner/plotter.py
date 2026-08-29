"""Matplotlib live plotter.

The animation callback drives ``GainTunerMIT.control_step`` on the main
thread, which avoids macOS GUI starvation with TkAgg.
"""
from __future__ import annotations

import logging
import math
import os
import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from .logging_utils import clamp, print_full_exception, wrap_to_pi

LOG = logging.getLogger("robstride_gain_tuner")


class LivePlotter:
    def __init__(self, tuner, window_s: float = 10.0, ui_hz: float = 30.0):
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
            if self.tuner.selected:
                return sorted(self.tuner.selected)[0]
            return sorted(self.tuner.motor_states.keys())[0]

    def _estimate_sine_delay(self, freq_hz: float):
        """Estimate cmd->pos effective delay at ``freq_hz`` using phase lag.

        Returns ``(delay_s, phase_lag_rad)`` or ``(None, None)``.
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

        phase_lag = wrap_to_pi(phi_cmd - phi_pos)
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
            direction = st.direction

        # Store samples for plotting + delay estimation
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
            + f" | limits=[{lim_lo:.3f},{lim_hi:.3f}] rad | dir={direction:+d}",
            y=0.985,
            fontsize=10.0,
        )

    def _update(self, _frame):
        try:
            now = time.time()
            dt_wall = clamp(now - self._last_update_t, 0.0, 0.1)
            self._last_update_t = now

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

            for ax in self.ax:
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)

            return (self.l_pos, self.l_cmd, self.l_err, self.l_vel, self.l_tq, self.l_temp)

        except Exception as exc:
            print_full_exception("[PLOT] Exception in LivePlotter._update (matplotlib callback)", exc, LOG)
            try:
                self.tuner.shutdown()
            finally:
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
