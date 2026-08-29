#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import mimetypes
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from utils.robstride_gain_tuner import config
from utils.robstride_gain_tuner.tuner import GainTunerMIT

STATIC_ROOT = Path(__file__).resolve().parent


class WebGainTuner(GainTunerMIT):
    def _confirm_large_change(self, delta_deg: float) -> bool:
        return True


class RobStrideWebState:
    def __init__(
        self,
        motor_ids: list[int],
        channel: str,
        bitrate: int,
        model: str,
        hz: float,
        ramp_deg_s: float,
    ) -> None:
        self.motor_ids = list(motor_ids)
        self.channel = channel
        self.bitrate = int(bitrate)
        self.model = model
        self.hz = float(hz)
        self.ramp_deg_s = float(ramp_deg_s)
        self._lock = threading.Lock()
        self._loop_stop = threading.Event()
        self._loop_thread: threading.Thread | None = None
        self.tuner = self._new_tuner()
        self.last_message = "not connected"

    def status(self) -> dict[str, Any]:
        with self.tuner.lock:
            motors = [self._motor_payload(self.tuner.motor_states[mid]) for mid in sorted(self.tuner.motor_states)]
            selected = sorted(self.tuner.selected)
            connected = bool(self.tuner.connected)
            safety_tripped = bool(self.tuner.safety_tripped)
            safety_reason = self.tuner.safety_reason or ""

        return {
            "connected": connected,
            "running": bool(self._loop_thread and self._loop_thread.is_alive()),
            "channel": self.channel,
            "bitrate": self.bitrate,
            "selected": selected,
            "safety_tripped": safety_tripped,
            "safety_reason": safety_reason,
            "last_message": self.last_message,
            "motors": motors,
        }

    def command(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = str(payload.get("action", "")).strip()
        params = payload.get("params") or {}
        motor_ids = self._resolve_motor_ids(payload.get("motor_ids"))
        if not action:
            raise ValueError("action is required")

        if action == "connect":
            return self._connect()
        if action == "disconnect":
            self._disconnect()
            return self._accepted(action, "disconnected")
        if action == "select":
            self._select(motor_ids)
            return self._accepted(action, f"selected {motor_ids}")

        self._select(motor_ids)

        if action == "hold":
            self.tuner.hold()
        elif action == "step":
            self.tuner.step(float(params.get("delta_deg", 0.0)))
        elif action == "goto":
            self.tuner.goto(float(params.get("angle_deg", 0.0)))
        elif action == "sine":
            duration = params.get("duration_s")
            self.tuner.sine(
                float(params.get("amp_deg", 0.0)),
                float(params.get("freq_hz", 0.0)),
                None if duration in (None, "") else float(duration),
            )
        elif action == "stop_excitation":
            self.tuner.stop_excitation()
        elif action == "set_kp":
            self.tuner.set_kp(float(params.get("kp", 0.0)))
        elif action == "set_kd":
            self.tuner.set_kd(float(params.get("kd", 0.0)))
        elif action == "invert":
            self.tuner.invert(set(motor_ids))
        else:
            raise ValueError(f"unknown action: {action}")

        return self._accepted(action, f"{action} applied")

    def _connect(self) -> dict[str, Any]:
        with self._lock:
            if self.tuner.connected:
                self._start_loop_locked()
                return self._accepted("connect", "already connected")
            self.tuner = self._new_tuner()
            ok = self.tuner.connect()
            if not ok:
                self.last_message = "RobStride connect failed"
                return {"accepted": False, "action": "connect", "message": self.last_message}
            self._start_loop_locked()
            return self._accepted("connect", "connected")

    def _disconnect(self) -> None:
        with self._lock:
            self._loop_stop.set()
            thread = self._loop_thread
            self._loop_thread = None
        if thread is not None:
            thread.join(timeout=1.5)
        self.tuner.shutdown()

    def _start_loop_locked(self) -> None:
        if self._loop_thread is not None and self._loop_thread.is_alive():
            return
        self._loop_stop.clear()
        self._loop_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._loop_thread.start()

    def _control_loop(self) -> None:
        last = time.time()
        while not self._loop_stop.is_set():
            now = time.time()
            dt = now - last
            last = now
            try:
                self.tuner.control_step(dt)
            except Exception as exc:
                self.last_message = str(exc)
            time.sleep(max(0.0, self.tuner.dt - (time.time() - now)))

    def _new_tuner(self) -> WebGainTuner:
        return WebGainTuner(
            motor_ids=self.motor_ids,
            channel=self.channel,
            bitrate=self.bitrate,
            model=self.model,
            hz=self.hz,
            ramp_deg_s=self.ramp_deg_s,
        )

    def _select(self, motor_ids: list[int]) -> None:
        if set(motor_ids) == set(self.tuner.motor_states):
            self.tuner.select(None)
            return
        with self.tuner.lock:
            self.tuner.selected = set(motor_ids)

    def _resolve_motor_ids(self, value: Any) -> list[int]:
        if value in (None, "", []):
            return list(self.tuner.motor_states.keys())
        if not isinstance(value, list):
            raise ValueError("motor_ids must be a list")
        ids = [int(item) for item in value]
        unknown = [mid for mid in ids if mid not in self.tuner.motor_states]
        if unknown:
            raise ValueError(f"unknown motor id(s): {unknown}")
        return ids

    def _motor_payload(self, st) -> dict[str, Any]:
        connected = bool(self.tuner.connected)
        return {
            "id": st.id,
            "name": st.name,
            "model": st.model,
            "selected": st.id in self.tuner.selected,
            "enabled": connected and bool(st.enabled),
            "direction": int(st.direction),
            "limit_lo": self._finite(st.limit_lo),
            "limit_hi": self._finite(st.limit_hi),
            "raw_position_rad": st.raw_position,
            "position_rad": st.position,
            "position_deg": math.degrees(st.position),
            "velocity_radps": st.velocity,
            "torque_nm": st.torque,
            "temperature_c": st.temperature,
            "kp": st.kp,
            "kd": st.kd,
            "target_rad": st.target_rad,
            "commanded_target_rad": st.commanded_target_rad,
            "temp_state": st.temp_state,
            "excitation": st.excitation.mode,
            "loop_dt_ms": st.loop_dt_ms,
            "write_dt_ms": st.write_dt_ms,
            "read_dt_ms": st.read_dt_ms,
            "io_gap_ms": st.io_gap_ms,
            "last_step_delay_s": None if math.isnan(st.last_step_delay_s) else st.last_step_delay_s,
            "read_failures": st.consecutive_read_failures,
            "last_error": st.last_error or "",
        }

    def _finite(self, value: float) -> float | None:
        return float(value) if math.isfinite(value) else None

    def _accepted(self, action: str, message: str) -> dict[str, Any]:
        self.last_message = message
        return {"accepted": True, "action": action, "message": message}


class RobStrideWebHandler(BaseHTTPRequestHandler):
    server: "RobStrideWebServer"

    def do_GET(self) -> None:
        if self.path == "/api/status":
            self._send_json(self.server.state.status())
            return
        path = "/" if self.path == "/" else self.path.split("?", 1)[0]
        static_path = STATIC_ROOT / ("index.html" if path == "/" else path.lstrip("/"))
        try:
            resolved = static_path.resolve()
            root = STATIC_ROOT.resolve()
        except OSError:
            self.send_error(404)
            return
        if not resolved.is_file() or root not in resolved.parents:
            self.send_error(404)
            return
        content = resolved.read_bytes()
        content_type = mimetypes.guess_type(str(resolved))[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def do_POST(self) -> None:
        if self.path != "/api/command":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
            self._send_json(self.server.state.command(payload))
        except Exception as exc:
            self._send_json({"accepted": False, "message": str(exc)}, status=400)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[robstride-webui] {self.address_string()} {fmt % args}")

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)


class RobStrideWebServer(ThreadingHTTPServer):
    def __init__(self, address: tuple[str, int], state: RobStrideWebState) -> None:
        super().__init__(address, RobStrideWebHandler)
        self.state = state


def _parse_motor_ids(raw: str) -> list[int]:
    ids = [int(part) for part in raw.replace(",", " ").split() if part.strip()]
    if not ids:
        raise ValueError("at least one motor id is required")
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate motor ids are not allowed")
    return ids


def main() -> None:
    default_ids = " ".join(str(mid) for mid in sorted(config.MOTOR_MODEL_BY_ID))
    parser = argparse.ArgumentParser(description="Serve the direct RobStride SDK motor Web UI.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--motor-ids", default=default_ids)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--model", default="rs-03")
    parser.add_argument("--hz", type=float, default=60.0)
    parser.add_argument("--ramp-deg-s", type=float, default=30.0)
    args = parser.parse_args()

    state = RobStrideWebState(
        motor_ids=_parse_motor_ids(args.motor_ids),
        channel=args.channel,
        bitrate=args.bitrate,
        model=args.model,
        hz=args.hz,
        ramp_deg_s=args.ramp_deg_s,
    )
    server = RobStrideWebServer((args.host, args.port), state)
    print(f"RobStride Web UI: http://{args.host}:{args.port}")
    print(f"CAN channel: {args.channel}, bitrate={args.bitrate}, motor_ids={args.motor_ids}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state._disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
