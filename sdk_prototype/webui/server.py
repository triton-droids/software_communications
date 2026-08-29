#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import mimetypes
import sys
import threading
from dataclasses import asdict, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

rosenv = REPO_ROOT / "rosenv"
if rosenv.is_dir():
    for site_packages in reversed(sorted((rosenv / "lib").glob("python*/site-packages"))):
        site_packages_str = str(site_packages)
        if site_packages_str not in sys.path:
            sys.path.insert(1, site_packages_str)

import grpc
import yaml

from sdk_prototype.python.robot_sdk import GainTuner, MotorGrpcClient, load_motor_configs_from_yaml

DEFAULT_CONFIG = REPO_ROOT / "humanoid_control/motor_control_hybrid/config/motors.yaml"
STATIC_ROOT = Path(__file__).resolve().parent


def _grpc_message(exc: Exception) -> str:
    if isinstance(exc, grpc.RpcError):
        details = exc.details() if callable(getattr(exc, "details", None)) else str(exc)
        code = exc.code() if callable(getattr(exc, "code", None)) else None
        return f"{code}: {details}" if code else details
    return str(exc)


class MotorWebState:
    def __init__(self, config_path: Path, motor_addr: str, tuner_hz: float) -> None:
        self.config_path = config_path
        self.motor_addr = motor_addr
        self.tuner_hz = float(tuner_hz)
        self.client = MotorGrpcClient(addr=motor_addr)
        self.configs = self._load_configs(config_path)
        self.joint_names = list(self.configs.keys())
        self._lock = threading.Lock()
        self._tuner: GainTuner | None = None
        self._tuner_joints: tuple[str, ...] = ()

    def status(self) -> dict[str, Any]:
        motors: dict[str, dict[str, Any]] = {
            name: self._motor_payload(name, None)
            for name in self.joint_names
        }
        connected = True
        error = ""

        try:
            reply = self.client.get_motor_status(self.joint_names)
            for motor in reply.motors:
                motors[motor.joint_name] = self._motor_payload(motor.joint_name, motor)
        except Exception as exc:
            connected = False
            error = _grpc_message(exc)

        return {
            "connected": connected,
            "error": error,
            "grpc_addr": self.motor_addr,
            "config_path": self._display_path(self.config_path),
            "motors": list(motors.values()),
            "tuner_running": self._tuner is not None,
            "tuner_joints": list(self._tuner_joints),
        }

    def _display_path(self, path: Path) -> str:
        try:
            return str(path.relative_to(Path.cwd()))
        except ValueError:
            return str(path)

    def _load_configs(self, config_path: Path):
        configs = load_motor_configs_from_yaml(config_path)
        with config_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if "motor_control_node" in data:
            params = data["motor_control_node"].get("ros__parameters", {})
        else:
            params = data
        default_kp = params.get("KP")
        default_kd = params.get("KD")
        if default_kp is None and default_kd is None:
            return configs
        patched = {}
        for name, cfg in configs.items():
            patched[name] = replace(
                cfg,
                kp=float(default_kp) if cfg.kp is None and default_kp is not None else cfg.kp,
                kd=float(default_kd) if cfg.kd is None and default_kd is not None else cfg.kd,
            )
        return patched

    def command(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = str(payload.get("action", "")).strip()
        joints = self._resolve_joints(payload.get("joints"))
        params = payload.get("params") or {}
        if not action:
            raise ValueError("action is required")

        if action == "enable":
            reply = self.client.enable_motors(joints)
            return self._reply(action, reply)
        if action == "disable":
            self._stop_tuner()
            reply = self.client.disable_motors(joints)
            return self._reply(action, reply)
        if action == "set_position":
            positions = self._number_list(params, "position_rad", len(joints))
            velocity = self._optional_number_list(params, "velocity_radps", len(joints))
            kp = self._optional_number_list(params, "kp", len(joints))
            kd = self._optional_number_list(params, "kd", len(joints))
            reply = self.client.set_motor_position(joints, positions, velocity, kp, kd)
            return self._reply(action, reply)
        if action == "set_velocity":
            velocity = self._number_list(params, "velocity_radps", len(joints))
            accel = self._optional_number_list(params, "acceleration_radps2", len(joints))
            reply = self.client.set_motor_velocity(joints, velocity, accel)
            return self._reply(action, reply)
        if action == "set_mit":
            positions = self._number_list(params, "position_rad", len(joints))
            velocity = self._number_list(params, "velocity_radps", len(joints))
            torque = self._optional_number_list(params, "torque_nm", len(joints))
            kp = self._optional_number_list(params, "kp", len(joints))
            kd = self._optional_number_list(params, "kd", len(joints))
            reply = self.client.set_motor_mit(joints, positions, velocity, torque, kp, kd)
            return self._reply(action, reply)

        tuner = self._ensure_tuner(joints)
        if action == "hold":
            tuner.hold()
        elif action == "step":
            tuner.step(float(params.get("delta_deg", 0.0)))
        elif action == "goto":
            tuner.goto(float(params.get("angle_deg", 0.0)))
        elif action == "sine":
            duration = params.get("duration_s")
            tuner.sine(
                float(params.get("amp_deg", 0.0)),
                float(params.get("freq_hz", 0.0)),
                None if duration in (None, "") else float(duration),
            )
        elif action == "stop_excitation":
            tuner.stop_excitation()
        elif action == "set_kp":
            tuner.set_kp(float(params.get("kp", 0.0)))
        elif action == "set_kd":
            tuner.set_kd(float(params.get("kd", 0.0)))
        elif action == "stop_tuner":
            self._stop_tuner()
        else:
            raise ValueError(f"unknown action: {action}")

        return {"accepted": True, "action": action, "message": f"{action} applied", "joint_names": joints}

    def _motor_payload(self, name: str, motor: Any | None) -> dict[str, Any]:
        cfg = self.configs.get(name)
        payload = asdict(cfg) if cfg is not None else {"joint_name": name}
        payload.update(
            {
                "joint_name": name,
                "online": motor is not None,
                "position_rad": float(getattr(motor, "position_rad", 0.0)),
                "velocity_radps": float(getattr(motor, "velocity_radps", 0.0)),
                "effort_nm": float(getattr(motor, "effort_nm", 0.0)),
                "temperature_c": float(getattr(motor, "temperature_c", 0.0)),
                "stamp_unix_s": float(getattr(motor, "stamp_unix_s", 0.0)),
            }
        )
        return payload

    def _ensure_tuner(self, joints: list[str]) -> GainTuner:
        key = tuple(joints)
        with self._lock:
            if self._tuner is not None and self._tuner_joints == key:
                return self._tuner
            self._stop_tuner_locked()
            cfgs = {name: self.configs[name] for name in joints if name in self.configs}
            self._tuner = GainTuner.from_client(self.client, joints, hz=self.tuner_hz, motor_configs=cfgs)
            self._tuner.start()
            self._tuner_joints = key
            return self._tuner

    def _stop_tuner(self) -> None:
        with self._lock:
            self._stop_tuner_locked()

    def _stop_tuner_locked(self) -> None:
        if self._tuner is not None:
            self._tuner.stop()
        self._tuner = None
        self._tuner_joints = ()

    def _resolve_joints(self, value: Any) -> list[str]:
        if value in (None, "", []):
            return list(self.joint_names)
        if not isinstance(value, list):
            raise ValueError("joints must be a list")
        joints = [str(item) for item in value if str(item)]
        if not joints:
            raise ValueError("at least one joint is required")
        unknown = [name for name in joints if name not in self.joint_names]
        if unknown:
            raise ValueError(f"unknown joint(s): {', '.join(unknown)}")
        return joints

    def _number_list(self, params: dict[str, Any], key: str, count: int) -> list[float]:
        if key not in params:
            raise ValueError(f"{key} is required")
        return self._coerce_list(params[key], count, key)

    def _optional_number_list(self, params: dict[str, Any], key: str, count: int) -> list[float] | None:
        value = params.get(key)
        if value in (None, ""):
            return None
        return self._coerce_list(value, count, key)

    def _coerce_list(self, value: Any, count: int, key: str) -> list[float]:
        raw = value if isinstance(value, list) else [value]
        result = [float(item) for item in raw]
        if len(result) == 1 and count > 1:
            return result * count
        if len(result) != count:
            raise ValueError(f"{key} must contain 1 or {count} value(s)")
        return result

    def _reply(self, action: str, reply: Any) -> dict[str, Any]:
        return {
            "accepted": bool(reply.accepted),
            "action": action,
            "message": str(reply.message),
            "joint_names": list(reply.joint_names),
        }


class MotorWebHandler(BaseHTTPRequestHandler):
    server: "MotorWebServer"

    def do_GET(self) -> None:
        if self.path == "/api/status":
            self._send_json(self.server.state.status())
            return
        path = "/" if self.path == "/" else self.path.split("?", 1)[0]
        static_path = STATIC_ROOT / ("index.html" if path == "/" else path.lstrip("/"))
        if not static_path.is_file() or STATIC_ROOT not in static_path.resolve().parents:
            self.send_error(404)
            return
        content = static_path.read_bytes()
        content_type = mimetypes.guess_type(str(static_path))[0] or "application/octet-stream"
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
            result = self.server.state.command(payload)
            self._send_json(result)
        except Exception as exc:
            self._send_json({"accepted": False, "message": _grpc_message(exc)}, status=400)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[webui] {self.address_string()} {fmt % args}")

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)


class MotorWebServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], state: MotorWebState) -> None:
        super().__init__(server_address, MotorWebHandler)
        self.state = state


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the motor control Web UI.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--motor-grpc-addr", default="127.0.0.1:50052")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--tuner-hz", type=float, default=60.0)
    args = parser.parse_args()

    state = MotorWebState(args.config, args.motor_grpc_addr, args.tuner_hz)
    server = MotorWebServer((args.host, args.port), state)
    print(f"Motor Web UI: http://{args.host}:{args.port}")
    print(f"Motor gRPC gateway: {args.motor_grpc_addr}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state._stop_tuner()
        server.server_close()


if __name__ == "__main__":
    main()
