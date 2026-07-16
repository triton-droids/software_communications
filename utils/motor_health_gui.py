#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Small Tk GUI for RobStride motor health and cautious movement tests.

This intentionally starts in a passive state. Motors are only enabled after the
user clicks "Connect + Hold", which reads each motor and commands its current
position before allowing steps.
"""

from __future__ import annotations

import math
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    import can
    from robstride_dynamics import CommunicationType, Motor, ParameterType, RobstrideBus
except Exception as exc:  # pragma: no cover - user-facing startup path
    raise SystemExit(
        "Failed to import motor dependencies. Try:\n"
        "  cd ~/Documents/embedded\n"
        "  ./.venv/bin/python utils/motor_health_gui.py\n\n"
        f"Import error: {exc}"
    ) from exc


MOTOR_MODEL_BY_ID: dict[int, str] = {
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

INVERSION_ARRAY = [-1, -1, -1, 1, 1, 1, -1, 1, -1, -1]
INVERSION_BY_ID = {i + 1: INVERSION_ARRAY[i] for i in range(len(INVERSION_ARRAY))}

JOINT_LIMITS: dict[int, tuple[float, float]] = {
    1: (-1.57, 1.57),
    2: (-1.57, 0.436332),
    3: (-0.785398, 0.785398),
    4: (-2.0944, 0.0),
    5: (-0.6, 0.6),
    6: (-1.57, 1.57),
    7: (-0.436332, 1.57),
    8: (-0.785398, 0.785398),
    9: (-2.0944, 0.0),
    10: (-0.6, 0.6),
}

DEFAULT_KP_BY_ID = {
    1: 30.0,
    2: 30.0,
    3: 20.0,
    4: 30.0,
    5: 30.0,
    6: 30.0,
    7: 30.0,
    8: 20.0,
    9: 30.0,
    10: 30.0,
}
DEFAULT_KD_BY_ID = {mid: 0.5 for mid in range(1, 11)}


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def offset_to_pi(x: float) -> float:
    return x - wrap_to_pi(x)


def parse_ids(text: str) -> list[int]:
    ids: list[int] = []
    for raw in text.replace(",", " ").split():
        mid = int(raw)
        if mid < 1 or mid > 255:
            raise ValueError(f"Motor ID out of range: {mid}")
        ids.append(mid)
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate motor IDs are not allowed")
    return ids


@dataclass
class MotorGuiState:
    mid: int
    name: str
    model: str
    direction: int
    limit_lo: float
    limit_hi: float
    kp: float
    kd: float
    enabled: bool = False
    raw_pos: float = 0.0
    pos: float = 0.0
    vel: float = 0.0
    torque: float = 0.0
    temp: float = 0.0
    target: float = 0.0
    commanded: float = 0.0
    startup_offset: float = 0.0
    safe_to_move: bool = True
    last_error: str = ""
    last_read_s: float = 0.0

    def logical_from_raw(self, raw_pos: float) -> float:
        return float(raw_pos) / float(self.direction) - self.startup_offset

    def physical_from_logical(self, logical_rad: float) -> float:
        return (float(logical_rad) + self.startup_offset) * float(self.direction)

    def update_from_raw(self, pos: float, vel: float, torque: float, temp: float) -> None:
        self.raw_pos = float(pos)
        self.pos = self.logical_from_raw(float(pos))
        self.vel = float(vel) / float(self.direction)
        self.torque = float(torque)
        self.temp = float(temp)
        self.last_read_s = time.time()

    def within_limits(self) -> bool:
        return self.limit_lo <= self.pos <= self.limit_hi


class MotorWorker(threading.Thread):
    def __init__(self, ui_queue: queue.Queue[dict[str, Any]]):
        super().__init__(daemon=True, name="motor_gui_worker")
        self.ui_queue = ui_queue
        self.commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.stop_evt = threading.Event()
        self.bus: RobstrideBus | None = None
        self.states: dict[int, MotorGuiState] = {}
        self.channel = "can0"
        self.bitrate = 1_000_000
        self.ramp_rad_s = math.radians(30.0)
        self.last_loop = time.time()
        self.last_publish = 0.0

    def post(self, kind: str, payload: Any = None) -> None:
        self.commands.put((kind, payload))

    def run(self) -> None:
        while not self.stop_evt.is_set():
            self._drain_commands()
            self._control_tick()
            self._publish()
            time.sleep(0.02)
        self._disconnect(disable=True)

    def _log(self, msg: str) -> None:
        self.ui_queue.put({"type": "log", "msg": msg})

    def _publish(self) -> None:
        now = time.time()
        if (now - self.last_publish) < 0.10:
            return
        self.last_publish = now
        rows = []
        for st in sorted(self.states.values(), key=lambda s: s.mid):
            age = now - st.last_read_s if st.last_read_s else math.inf
            health = "OK"
            if st.last_error:
                health = "ERR"
            elif not st.enabled:
                health = "DIS"
            elif not st.safe_to_move:
                health = "LIMIT"
            elif age > 0.5:
                health = "STALE"
            rows.append(
                {
                    "id": st.mid,
                    "model": st.model,
                    "enabled": st.enabled,
                    "pos_deg": math.degrees(st.pos),
                    "cmd_deg": math.degrees(st.commanded),
                    "vel_deg_s": math.degrees(st.vel),
                    "torque": st.torque,
                    "temp": st.temp,
                    "health": health,
                    "error": st.last_error,
                    "safe_to_move": st.safe_to_move,
                }
            )
        self.ui_queue.put({"type": "rows", "rows": rows})

    def _drain_commands(self) -> None:
        for _ in range(20):
            try:
                kind, payload = self.commands.get_nowait()
            except queue.Empty:
                return
            try:
                if kind == "scan":
                    self._scan(payload)
                elif kind == "connect":
                    self._connect(**payload)
                elif kind == "step":
                    self._step(**payload)
                elif kind == "hold":
                    self._hold(payload)
                elif kind == "goto_zero":
                    self._goto_zero(payload)
                elif kind == "disable":
                    self._disable(payload)
                elif kind == "zero":
                    self._zero(payload)
                elif kind == "gains":
                    self._set_gains(**payload)
                elif kind == "disconnect":
                    self._disconnect(disable=True)
            except Exception as exc:
                self._log(f"ERROR: {exc}")

    def _scan(self, payload: dict[str, Any]) -> None:
        channel = str(payload.get("channel", "can0"))
        bitrate = int(payload.get("bitrate", 1_000_000))
        start_id = int(payload.get("start", 1))
        end_id = int(payload.get("end", 10))
        self._log(f"Scanning IDs {start_id}..{end_id} on {channel} @ {bitrate} bps")
        bus = can.interface.Bus(interface="socketcan", channel=channel, bitrate=bitrate)
        found: list[int] = []
        try:
            for mid in range(start_id, end_id + 1):
                arb_id = (CommunicationType.GET_DEVICE_ID << 24) | (0xFF << 8) | mid
                msg = can.Message(arbitration_id=arb_id, is_extended_id=True, data=b"\x00" * 8)
                bus.send(msg)
                deadline = time.time() + 0.12
                while time.time() < deadline:
                    rsp = bus.recv(timeout=0.03)
                    if rsp is None or not rsp.is_extended_id:
                        continue
                    comm_type = (rsp.arbitration_id >> 24) & 0x1F
                    if comm_type == CommunicationType.GET_DEVICE_ID:
                        found.append(mid)
                        break
        finally:
            bus.shutdown()
        self._log(f"Scan found IDs: {found if found else 'none'}")
        self.ui_queue.put({"type": "scan_result", "ids": found})

    def _connect(self, *, channel: str, bitrate: int, ids: list[int]) -> None:
        self._disconnect(disable=True)
        if not ids:
            raise ValueError("No motor IDs selected")
        self.channel = channel
        self.bitrate = int(bitrate)
        motors = {
            f"motor_{mid}": Motor(id=mid, model=MOTOR_MODEL_BY_ID.get(mid, "rs-03"))
            for mid in ids
        }
        calibration = {name: {"direction": 1, "homing_offset": 0.0} for name in motors}
        self.bus = RobstrideBus(self.channel, motors, calibration, bitrate=self.bitrate)
        self.bus.connect(handshake=True)
        self.states = {}
        for mid in ids:
            name = f"motor_{mid}"
            direction = 1 if INVERSION_BY_ID.get(mid, 1) >= 0 else -1
            lo, hi = JOINT_LIMITS.get(mid, (-math.inf, math.inf))
            st = MotorGuiState(
                mid=mid,
                name=name,
                model=MOTOR_MODEL_BY_ID.get(mid, "rs-03"),
                direction=direction,
                limit_lo=lo,
                limit_hi=hi,
                kp=DEFAULT_KP_BY_ID.get(mid, 30.0),
                kd=DEFAULT_KD_BY_ID.get(mid, 0.5),
            )
            self._log(f"Enabling motor {mid}; holding current position")
            self.bus.enable(name)
            st.enabled = True
            time.sleep(0.05)
            self._set_mode_mit(mid)
            pos, vel, tq, temp = self.bus.read_operation_frame(name)
            st.startup_offset = offset_to_pi(float(pos) / float(st.direction))
            st.update_from_raw(pos, vel, tq, temp)
            st.target = st.pos
            st.commanded = st.pos
            self.bus.write_operation_frame(name, st.physical_from_logical(st.commanded), st.kp, st.kd, 0.0, 0.0)
            self.states[mid] = st
            st.safe_to_move = st.within_limits()
            if not st.safe_to_move:
                self._log(
                    f"Warning: motor {mid} starts outside limits "
                    f"pos={st.pos:.3f} rad limits=[{st.limit_lo:.3f},{st.limit_hi:.3f}]. "
                    "Disabling motor; fix pose/zero before reconnecting."
                )
                self.bus.disable(name)
                st.enabled = False
        enabled_ids = [st.mid for st in self.states.values() if st.enabled]
        disabled_ids = [st.mid for st in self.states.values() if not st.enabled]
        self._log(f"Connected. Enabled/holding IDs: {enabled_ids if enabled_ids else 'none'}")
        if disabled_ids:
            self._log(f"Disabled unsafe IDs: {disabled_ids}")

    def _set_mode_mit(self, mid: int) -> None:
        if self.bus is None:
            return
        name = f"motor_{mid}"
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", 0, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, self.bus.motors[name].id, data)
        time.sleep(0.05)

    def _selected_states(self, ids: list[int] | None) -> list[MotorGuiState]:
        if ids is None:
            return list(self.states.values())
        return [self.states[mid] for mid in ids if mid in self.states]

    def _step(self, *, ids: list[int], delta_deg: float) -> None:
        delta = math.radians(float(clamp(delta_deg, -15.0, 15.0)))
        for st in self._selected_states(ids):
            if not st.enabled:
                st.last_error = "disabled; reconnect after fixing pose/zero"
                self._log(f"Blocked step for motor {st.mid}: disabled")
                continue
            if not st.safe_to_move:
                st.last_error = "outside limits; set zero or fix pose before stepping"
                self._log(f"Blocked step for motor {st.mid}: outside configured limits")
                continue
            st.target = clamp(st.target + delta, st.limit_lo, st.limit_hi)
            st.last_error = ""
        self._log(f"Step {delta_deg:+.2f} deg for IDs {ids}")

    def _hold(self, ids: list[int] | None) -> None:
        for st in self._selected_states(ids):
            st.target = st.pos
            st.commanded = st.pos
            st.last_error = ""
        self._log("Hold current position")

    def _goto_zero(self, ids: list[int]) -> None:
        for st in self._selected_states(ids):
            if not st.enabled:
                st.last_error = "disabled; reconnect after fixing pose/zero"
                self._log(f"Blocked goto 0 for motor {st.mid}: disabled")
                continue
            if not st.safe_to_move:
                st.last_error = "outside limits; set zero or fix pose before goto 0"
                self._log(f"Blocked goto 0 for motor {st.mid}: outside configured limits")
                continue
            st.target = clamp(0.0, st.limit_lo, st.limit_hi)
            st.last_error = ""
        self._log(f"Goto 0 for IDs {ids}")

    def _disable(self, ids: list[int] | None) -> None:
        if self.bus is None:
            return
        for st in self._selected_states(ids):
            try:
                self.bus.disable(st.name)
                st.enabled = False
            except Exception as exc:
                st.last_error = str(exc)
        self._log("Disabled selected motors")

    def _zero(self, ids: list[int]) -> None:
        if self.bus is None:
            raise RuntimeError("Connect before zeroing")
        for st in self._selected_states(ids):
            data = b"\x01\x00\x00\x00\x00\x00\x00\x00"
            self.bus.transmit(CommunicationType.SET_ZERO_POSITION, 0xFE, st.mid, data)
            time.sleep(0.05)
            if st.enabled:
                pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
                st.startup_offset = offset_to_pi(float(pos) / float(st.direction))
                st.update_from_raw(pos, vel, tq, temp)
                st.safe_to_move = st.within_limits()
                st.target = st.pos
                st.commanded = st.pos
                if st.safe_to_move:
                    self.bus.write_operation_frame(
                        st.name,
                        st.physical_from_logical(st.commanded),
                        st.kp,
                        st.kd,
                        0.0,
                        0.0,
                    )
                else:
                    self.bus.disable(st.name)
                    st.enabled = False
            else:
                st.last_error = "zero sent while disabled; reconnect to resync"
        self._log(f"Set mechanical zero for IDs {ids}. Reconnect before moving zeroed disabled motors.")

    def _set_gains(self, *, ids: list[int], kp: float, kd: float) -> None:
        for st in self._selected_states(ids):
            st.kp = float(clamp(kp, 0.0, 5000.0))
            st.kd = float(clamp(kd, 0.0, 100.0))
        self._log(f"Updated gains for IDs {ids}: kp={kp:.2f}, kd={kd:.2f}")

    def _control_tick(self) -> None:
        if self.bus is None or not self.states:
            return
        now = time.time()
        dt = clamp(now - self.last_loop, 0.0, 0.05)
        self.last_loop = now
        max_step = self.ramp_rad_s * dt
        for st in self.states.values():
            if not st.enabled:
                continue
            try:
                if not st.within_limits():
                    st.safe_to_move = False
                    st.target = st.pos
                    st.commanded = st.pos
                    st.last_error = "outside configured limits; motor disabled"
                    self.bus.disable(st.name)
                    st.enabled = False
                    self._log(
                        f"Motor {st.mid} outside limits "
                        f"pos={st.pos:.3f} rad limits=[{st.limit_lo:.3f},{st.limit_hi:.3f}]; disabled"
                    )
                    continue
                if not st.safe_to_move:
                    st.last_error = "motion locked; reconnect after fixing limits"
                    self.bus.disable(st.name)
                    st.enabled = False
                    self._log(f"Motor {st.mid} motion locked; disabled")
                    continue
                else:
                    st.target = clamp(st.target, st.limit_lo, st.limit_hi)
                delta = st.target - st.commanded
                if abs(delta) <= max_step:
                    st.commanded = st.target
                else:
                    st.commanded += math.copysign(max_step, delta)
                if st.safe_to_move:
                    st.commanded = clamp(st.commanded, st.limit_lo, st.limit_hi)
                self.bus.write_operation_frame(
                    st.name,
                    st.physical_from_logical(st.commanded),
                    st.kp,
                    st.kd,
                    0.0,
                    0.0,
                )
                pos, vel, tq, temp = self.bus.read_operation_frame(st.name, timeout=0.05)
                st.update_from_raw(pos, vel, tq, temp)
                if st.safe_to_move and not st.within_limits():
                    st.safe_to_move = False
                    st.target = st.pos
                    st.commanded = st.pos
                    st.last_error = "moved outside configured limits; motor disabled"
                    self.bus.disable(st.name)
                    st.enabled = False
                    self._log(f"Motor {st.mid} moved outside limits; disabled")
                else:
                    st.last_error = ""
            except Exception as exc:
                if "No response" not in str(exc):
                    st.last_error = str(exc)

    def _disconnect(self, disable: bool) -> None:
        if self.bus is not None:
            try:
                self.bus.disconnect(disable_torque=disable)
            except Exception as exc:
                self._log(f"Disconnect warning: {exc}")
        for st in self.states.values():
            st.enabled = False
        self.bus = None


class MotorHealthGui:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("RobStride Motor Health + Test")
        self.root.geometry("1120x720")
        self.ui_queue: queue.Queue[dict[str, Any]] = queue.Queue()
        self.worker = MotorWorker(self.ui_queue)
        self.worker.start()
        self.rows: dict[int, dict[str, Any]] = {}
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._poll_ui()

    def _build(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        self.channel_var = tk.StringVar(value="can0")
        self.bitrate_var = tk.StringVar(value="1000000")
        self.ids_var = tk.StringVar(value="4")
        self.step_var = tk.StringVar(value="5")
        self.kp_var = tk.StringVar(value="30")
        self.kd_var = tk.StringVar(value="0.5")

        self._field(top, "Channel", self.channel_var, 0, width=8)
        self._field(top, "Bitrate", self.bitrate_var, 1, width=10)
        self._field(top, "Motor IDs", self.ids_var, 2, width=18)
        ttk.Button(top, text="Scan 1-10", command=self._scan).grid(row=0, column=6, padx=4)
        ttk.Button(top, text="Connect + Hold", command=self._connect).grid(row=0, column=7, padx=4)
        ttk.Button(top, text="Disable Selected", command=self._disable_selected).grid(row=0, column=8, padx=4)
        ttk.Button(top, text="Disable All", command=self._disable_all).grid(row=0, column=9, padx=4)

        move = ttk.LabelFrame(self.root, text="Movement", padding=10)
        move.pack(fill=tk.X, padx=10, pady=(0, 8))
        self._field(move, "Step deg", self.step_var, 0, width=8)
        ttk.Button(move, text="- Step", command=lambda: self._step(-1.0)).grid(row=0, column=2, padx=4)
        ttk.Button(move, text="+ Step", command=lambda: self._step(1.0)).grid(row=0, column=3, padx=4)
        ttk.Button(move, text="Goto 0 Selected", command=self._goto_zero_selected).grid(row=0, column=4, padx=4)
        ttk.Button(move, text="Hold Selected", command=self._hold_selected).grid(row=0, column=5, padx=4)
        ttk.Button(move, text="Set Zero Selected", command=self._zero_selected).grid(row=0, column=6, padx=4)
        self._field(move, "Kp", self.kp_var, 7, width=8)
        self._field(move, "Kd", self.kd_var, 8, width=8)
        ttk.Button(move, text="Apply Gains", command=self._apply_gains).grid(row=0, column=18, padx=4)

        columns = ("id", "model", "enabled", "pos", "cmd", "vel", "torque", "temp", "health", "error")
        self.tree = ttk.Treeview(self.root, columns=columns, show="headings", selectmode="extended", height=14)
        headings = {
            "id": "ID",
            "model": "Model",
            "enabled": "Enabled",
            "pos": "Pos deg",
            "cmd": "Cmd deg",
            "vel": "Vel deg/s",
            "torque": "Torque",
            "temp": "Temp C",
            "health": "Health",
            "error": "Last Error",
        }
        widths = {"id": 50, "model": 70, "enabled": 80, "pos": 90, "cmd": 90, "vel": 100, "torque": 90, "temp": 80, "health": 80, "error": 320}
        for col in columns:
            self.tree.heading(col, text=headings[col])
            self.tree.column(col, width=widths[col], anchor=tk.CENTER if col != "error" else tk.W)
        self.tree.pack(fill=tk.BOTH, expand=True, padx=10)

        log_frame = ttk.LabelFrame(self.root, text="Log", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=False, padx=10, pady=10)
        self.log = tk.Text(log_frame, height=8, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

    def _field(self, parent: ttk.Frame, label: str, var: tk.StringVar, col: int, width: int) -> None:
        ttk.Label(parent, text=label).grid(row=0, column=col * 2, sticky=tk.W, padx=(0, 4))
        ttk.Entry(parent, textvariable=var, width=width).grid(row=0, column=col * 2 + 1, sticky=tk.W, padx=(0, 8))

    def _selected_ids(self) -> list[int]:
        selected = []
        for iid in self.tree.selection():
            try:
                selected.append(int(iid))
            except ValueError:
                pass
        if selected:
            return selected
        return parse_ids(self.ids_var.get())

    def _scan(self) -> None:
        self.worker.post("scan", {"channel": self.channel_var.get(), "bitrate": int(self.bitrate_var.get()), "start": 1, "end": 10})

    def _connect(self) -> None:
        try:
            ids = parse_ids(self.ids_var.get())
        except Exception as exc:
            messagebox.showerror("Invalid IDs", str(exc))
            return
        if not messagebox.askyesno(
            "Mechanical zero check",
            f"Have motor IDs {ids} been mechanically zeroed in their correct physical zero pose?\n\n"
            "Choose No if you are not sure. Do not enable/move motors until zeroing is known correct.",
        ):
            self._append_log("Connect cancelled: user did not confirm mechanical zeroing.")
            return
        if not messagebox.askokcancel(
            "Enable motors?",
            "This will enable selected motors and command them to hold current position.\n\n"
            "Keep clear of joints and be ready to cut power.",
        ):
            return
        self.worker.post("connect", {"channel": self.channel_var.get(), "bitrate": int(self.bitrate_var.get()), "ids": ids})

    def _step(self, sign: float) -> None:
        try:
            ids = self._selected_ids()
            delta = sign * abs(float(self.step_var.get()))
        except Exception as exc:
            messagebox.showerror("Invalid step", str(exc))
            return
        self.worker.post("step", {"ids": ids, "delta_deg": delta})

    def _hold_selected(self) -> None:
        self.worker.post("hold", self._selected_ids())

    def _goto_zero_selected(self) -> None:
        ids = self._selected_ids()
        if not messagebox.askokcancel(
            "Goto 0?",
            f"Command selected motor IDs {ids} to logical 0 degrees?\n\n"
            "Only continue if mechanical zeroing is known correct.",
        ):
            return
        self.worker.post("goto_zero", ids)

    def _disable_selected(self) -> None:
        self.worker.post("disable", self._selected_ids())

    def _disable_all(self) -> None:
        self.worker.post("disable", None)

    def _zero_selected(self) -> None:
        ids = self._selected_ids()
        if not messagebox.askokcancel(
            "Set mechanical zero?",
            f"Set mechanical zero for IDs {ids}?\n\n"
            "Only do this when each selected joint is physically in its correct zero pose.",
        ):
            return
        self.worker.post("zero", ids)

    def _apply_gains(self) -> None:
        self.worker.post("gains", {"ids": self._selected_ids(), "kp": float(self.kp_var.get()), "kd": float(self.kd_var.get())})

    def _poll_ui(self) -> None:
        try:
            while True:
                msg = self.ui_queue.get_nowait()
                if msg["type"] == "log":
                    self._append_log(msg["msg"])
                elif msg["type"] == "rows":
                    self._update_rows(msg["rows"])
                elif msg["type"] == "scan_result":
                    if msg["ids"]:
                        self.ids_var.set(" ".join(str(i) for i in msg["ids"]))
        except queue.Empty:
            pass
        self.root.after(150, self._poll_ui)

    def _append_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{stamp}] {text}\n")
        self.log.see(tk.END)

    def _update_rows(self, rows: list[dict[str, Any]]) -> None:
        seen = set()
        for row in rows:
            iid = str(row["id"])
            seen.add(iid)
            vals = (
                row["id"],
                row["model"],
                "yes" if row["enabled"] else "no",
                f"{row['pos_deg']:+.2f}",
                f"{row['cmd_deg']:+.2f}",
                f"{row['vel_deg_s']:+.2f}",
                f"{row['torque']:+.3f}",
                f"{row['temp']:.1f}",
                row["health"],
                row["error"],
            )
            if self.tree.exists(iid):
                self.tree.item(iid, values=vals)
            else:
                self.tree.insert("", tk.END, iid=iid, values=vals)
        for iid in list(self.tree.get_children()):
            if iid not in seen:
                self.tree.delete(iid)

    def _close(self) -> None:
        self.worker.stop_evt.set()
        self.worker.post("disconnect", None)
        self.root.after(300, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    MotorHealthGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
