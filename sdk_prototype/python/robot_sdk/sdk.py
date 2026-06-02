"""Programmatic SDK wrapper for the prototype.

This package is the client-facing SDK only. The demo under
`sdk_prototype.demo.robot_sdk_demo.model` shows this SDK driving the ROS2
double-pendulum setup through the motor gateway.
"""
from __future__ import annotations

from .grpc_client import GrpcRobotClient


class RobotSDK:
    """High-level SDK wrapper exposing robot commands."""

    def __init__(
        self,
        motor_configs: dict | None = None,
    ) -> None:
        self._client = GrpcRobotClient()
        from .grpc_client import MotorGrpcClient

        self.motor_client = MotorGrpcClient()
        self.motor_configs = motor_configs or {}

    def enable_robot(self):
        return self._client.enable_robot()

    def disable_robot(self):
        return self._client.disable_robot()

    def set_mode(self, mode: str):
        return self._client.set_mode(mode)

    def load_policy(self, policy_id: str, uri: str):
        return self._client.load_policy(policy_id, uri)

    def start_policy(self, policy_id: str):
        return self._client.start_policy(policy_id)

    def stop_policy(self):
        return self._client.stop_policy()

    def set_velocity_command(self, vx_mps: float, vy_mps: float, wz_radps: float, timeout_s: float = 0.25):
        return self._client.set_velocity_command(vx_mps, vy_mps, wz_radps, timeout_s)

    def get_robot_status(self):
        return self._client.get_robot_status()

    def list_motors(self) -> list[str]:
        """Return configured joint names in insertion order."""
        return list(self.motor_configs.keys())

    def motor(self, joint_name: str):
        """Create a single-motor proxy for exactly one joint."""
        from .motor import Motor, MotorConfig

        cfg = self.motor_configs.get(joint_name)
        motor_config = None
        if isinstance(cfg, MotorConfig):
            motor_config = cfg
        elif isinstance(cfg, dict):
            motor_config = MotorConfig.from_ros2_yaml(joint_name, cfg)

        return Motor.from_sdk(self, joint_name=joint_name, motor_config=motor_config)

    def gain_tuner(self, joint_names: list[str]):
        """Create a multi-motor control helper."""
        from .gain_tuner import GainTuner
        from .motor import MotorConfig

        motor_configs: dict[str, MotorConfig] = {}
        for name in joint_names:
            cfg = self.motor_configs.get(name)
            if isinstance(cfg, MotorConfig):
                motor_configs[name] = cfg
            elif isinstance(cfg, dict):
                motor_configs[name] = MotorConfig.from_ros2_yaml(name, cfg)

        return GainTuner.from_client(self.motor_client, joint_names, motor_configs=motor_configs or None)
