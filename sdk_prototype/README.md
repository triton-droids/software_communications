# SDK Prototype

This package exposes a small Python SDK for robot control. The SDK is designed
to hide ROS2 details behind a client-facing API.

The main entry points are:

- `RobotSDK` for robot-level commands and motor access
- `Motor` for single-joint control
- `MotorGrpcClient` and `GrpcRobotClient` for direct gRPC access
- `GainTuner` for multi-joint tuning and motion helpers

## Install

```bash
python3 -m pip install -r sdk_prototype/requirements.txt
```

Regenerate generated gRPC files after changing the proto:

```bash
bash sdk_prototype/python/generate_grpc_python.sh
```

## Public API

Import from:

```python
from sdk_prototype.python.robot_sdk import (
    RobotSDK,
    Motor,
    MotorConfig,
    load_motor_configs_from_yaml,
    GainTuner,
    GrpcRobotClient,
    MotorGrpcClient,
)
```

### `load_motor_configs_from_yaml(config_path)`

Load a ROS2-style motor registry YAML file and return:

- key: joint name
- value: `MotorConfig`

Expected file shape:

```yaml
motor_control_node:
  ros__parameters:
    motors:
      base_to_shoulder_joint:
        motor_id: 21
        min_position: -1.57
        max_position: 1.57
        kp: 40.0
        kd: 1.5
```

Use this when you want to read motor metadata before creating SDK objects.

### `RobotSDK(motor_configs=None)`

High-level SDK wrapper.

Methods:

- `enable_robot()`
- `disable_robot()`
- `set_mode(mode)`
- `load_policy(policy_id, uri)`
- `start_policy(policy_id)`
- `stop_policy()`
- `set_velocity_command(vx_mps, vy_mps, wz_radps, timeout_s=0.25)`
- `get_robot_status()`
- `list_motors()`
- `motor(joint_name)`
- `gain_tuner(joint_names)`

`motor(joint_name)` returns a `Motor` object bound to one joint. If the joint
exists in `motor_configs`, the config is attached to that `Motor`.

### `Motor`

Single-joint helper returned by `sdk.motor(joint_name)`.

Methods:

- `get_motor_config()`
- `enable()`
- `disable()`
- `set_velocity(velocity, acceleration=None)`
- `set_position(position, velocity=None, kp=None, kd=None)`
- `set_mit(position, velocity, torque_nm=None, kp=None, kd=None)`
- `get_status()`

Behavior:

- `set_position()` clamps position using `min_position` and `max_position` from
  `MotorConfig` when they exist.
- `set_position()` and `set_mit()` use `kp` and `kd` from `MotorConfig` if the
  caller does not pass gains.
- If `kp` or `kd` are missing, the call raises `ValueError`.

### `MotorConfig`

Dataclass describing one joint entry from YAML.

Fields:

- `joint_name`
- `can_interface`
- `master_id`
- `motor_id`
- `actuator_type`
- `model`
- `direction`
- `min_position`
- `max_position`
- `kp`
- `kd`

### `MotorGrpcClient`

Low-level motor RPC client.

Default address: `127.0.0.1:50052`

Methods:

- `enable_motors(joint_names)`
- `disable_motors(joint_names)`
- `set_motor_velocity(joint_names, velocity_radps, acceleration_radps2=None)`
- `set_motor_position(joint_names, position_rad, velocity_radps=None, kp=None, kd=None)`
- `set_motor_mit(joint_names, position_rad, velocity_radps, torque_nm=None, kp=None, kd=None)`
- `get_motor_status(joint_names=None)`

Use this when you want to send commands to multiple motors directly without
creating per-joint `Motor` objects.

### `GrpcRobotClient`

Robot-level gRPC client.

Default address: `127.0.0.1:50051`

Methods:

- `enable_robot()`
- `disable_robot()`
- `set_mode(mode)`
- `load_policy(policy_id, uri)`
- `start_policy(policy_id)`
- `stop_policy()`
- `set_velocity_command(vx_mps, vy_mps, wz_radps, timeout_s=0.25)`
- `get_robot_status()`

### `GainTuner`

Multi-motor helper for control loops, hold/goto/step commands, and basic
excitation.

Create it with:

```python
tuner = GainTuner.from_client(client, joint_names, hz=60.0, motor_configs=None)
```

Methods:

- `start()`
- `stop()`
- `hold()`
- `step(delta_deg)`
- `goto(angle_deg)`
- `sine(amp_deg, freq_hz, duration_s=None)`
- `stop_excitation()`
- `set_kp(kp)`
- `set_kd(kd)`
- `status()`

Utility functions in `gain_tuner.py`:

- `pd_from_natural_freq(wn_rad_s, damping_ratio=1.0, inertia=1.0)`
- `ziegler_nichols_pid(ku, tu)`
- `suggest_initial_pd(omega_hz=1.0, damping_ratio=0.7, inertia=1.0)`
- `motion_scale_from_temp(temp_c)`
- `temp_state_from_temp(temp_c)`

## Motor Web UI

The Web UI in `sdk_prototype/webui` shows live status for every motor in
`humanoid_control/motor_control_hybrid/config/motors.yaml` and exposes buttons
for the existing motor and gain-tuner commands.

Start the motor gRPC gateway first:

```bash
ros2 run motor_control_hybrid motor_sdk_gateway_node
```

Then serve the UI:

```bash
python3 sdk_prototype/webui/server.py
```

Open:

```text
http://127.0.0.1:8088
```

Useful options:

```bash
python3 sdk_prototype/webui/server.py \
  --motor-grpc-addr 127.0.0.1:50052 \
  --config humanoid_control/motor_control_hybrid/config/motors.yaml \
  --port 8088
```

The page polls `/api/status` at 4 Hz. Commands are sent through
`/api/command`, which calls the existing `MotorGrpcClient` and `GainTuner`
APIs.

## Direct RobStride Web UI

The Web UI in `sdk_prototype/robstride_webui` does not use ROS2 or gRPC. It
uses `utils.robstride_gain_tuner.GainTunerMIT` directly, connects to the CAN
bus through the RobStride SDK, and runs its own control loop.

Serve the UI:

```bash
python3 sdk_prototype/robstride_webui/server.py --motor-ids "1 2 3 4 5 6 7 8 9 10"
```

Open:

```text
http://127.0.0.1:8090
```

Useful options:

```bash
python3 sdk_prototype/robstride_webui/server.py \
  --channel can0 \
  --bitrate 1000000 \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --hz 60 \
  --ramp-deg-s 30
```

The page polls `/api/status` at 4 Hz. The `Connect` button opens the RobStride
bus and enables the selected motors. `Disconnect` stops the control loop,
disables enabled motors, and disconnects the bus.

## Typical Usage

### Load YAML and inspect config

```python
from sdk_prototype.python.robot_sdk import RobotSDK, load_motor_configs_from_yaml

configs = load_motor_configs_from_yaml(
    "humanoid_control/motor_control_hybrid/config/control_config.yaml"
)
sdk = RobotSDK(motor_configs=configs)

cfg = sdk.motor_configs["base_to_shoulder_joint"]
print(cfg.motor_id)
print(cfg.min_position, cfg.max_position)
```

### Control one motor

```python
motor = sdk.motor("base_to_shoulder_joint")
motor.enable()
motor.set_position(0.45)
motor.disable()
```

### Control multiple motors

```python
joint_names = [
    "base_to_shoulder_joint",
    "shoulder_to_upper_arm_joint",
    "upper_arm_to_lower_arm_joint",
    "lower_arm_to_wrist_joint",
]

motors = {name: sdk.motor(name) for name in joint_names}

for motor in motors.values():
    motor.enable()

motors["base_to_shoulder_joint"].set_position(0.2)
motors["shoulder_to_upper_arm_joint"].set_position(-0.2)

for motor in motors.values():
    motor.disable()
```

### Use the lower-level motor client

```python
client = MotorGrpcClient("127.0.0.1:50052")
reply = client.enable_motors(["test_joint", "test_joint2"])
print(reply)
```

## YAML Notes

The humanoid arm config used by the demo is:

```text
humanoid_control/motor_control_hybrid/config/control_config.yaml
```

The loader reads:

- `motor_control_node.ros__parameters.motors`
- or a flat `motors` mapping if the YAML is already flattened

If a joint config is missing `kp` or `kd`, `Motor.set_position()` and
`Motor.set_mit()` will raise `ValueError` unless you pass the gains explicitly.

## ROS2 Gateway

The motor SDK talks to the ROS2-side gateway through gRPC.

Default endpoints:

- robot RPC: `127.0.0.1:50051`
- motor RPC: `127.0.0.1:50052`

The gateway is expected to bridge SDK calls to the ROS2 motor stack.
