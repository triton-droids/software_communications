import argparse
import sys
import time
from pathlib import Path

## Read motor config
def motor_config_path() -> Path:
    repo_root = Path(__file__).resolve().parent
    return repo_root / "humanoid_control" / "motor_control_hybrid" / "config" / "control_config.yaml"

## Must have to use SDK
def build_sdk(config_path: Path):
    from sdk_prototype.python.robot_sdk import RobotSDK, load_motor_configs_from_yaml

    if not config_path.exists():
        raise FileNotFoundError(f"Motor config not found: {config_path}")

    motor_configs = load_motor_configs_from_yaml(config_path)
    return RobotSDK(motor_configs=motor_configs)

## If using arguments
def parse_args() -> argparse.Namespace:
    pass

def main() -> None:
    #args = parse_args()
    ## Read motor config and build SDK
    config_path = motor_config_path()
    sdk = build_sdk(config_path)
    ## Create motor objects defined in SDK 
    ## using motor names defined in the config file
    motors = {name: sdk.motor(name) 
            for name in sdk.motor_configs.keys()}
    ## Enable motors
    for motor in motors.values():
        motor.enable()
    ## Move a motor
    motors["base_to_shoulder_joint"].set_position(position_rad=[0.5], velocity_radps=[1.0]) ## radian
    motor["upper_arm_to_forearm_joint"].set_velocity(velocity = 5) ## degree
    motors["forearm_to_wrist_joint"].set_mit(position_rad=[0.2], velocity_radps=[0.5], torque_nm=[0.1], kp=[10.0], kd=[0.5])
    time.sleep(1.0)
    ## Get status
    status = motors["base_to_shoulder_joint"].get_status()
    ## Disable all motors 
    for motor in motors.values():
        motor.disable()
    ## That is bascailly what we need rigt now. 
    ## Next step will be implement functions that controls whole robot such as enable_robot()
    ## To change setup we need to change the config file, in humanoids_control/motor_control_hybrid/config/control_config.yaml or motors.yaml
    
if __name__ == "__main__":
    main()
