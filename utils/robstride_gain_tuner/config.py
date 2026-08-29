"""Configuration constants for the RobStride gain tuner.

All magic numbers from the original monolithic script live here so the
behaviour can be tuned without reading control/plot code.
"""
from __future__ import annotations

# -------------------- Inversion array (CAN IDs 1..10) --------------------
# Interpreted sequentially for CAN IDs 1..10.  1 = normal, -1 = inverted.
INVERSION_ARRAY = [-1, -1, -1, 1, 1, 1, -1, 1, -1, -1]
INVERSION_BY_ID = {i + 1: direction for i, direction in enumerate(INVERSION_ARRAY)}

# -------------------- Joint limits (radians, logical space) --------------------
JOINT_LIMITS = {
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

# -------------------- Per-motor model map --------------------
MOTOR_MODEL_BY_ID = {
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

# -------------------- Safety feature switches --------------------
ACTUATION_SAFETY_ENABLED = False  # joint-limit/jump trips disabled for tuner testing
TORQUE_SAFETY_ENABLED = True
GROUND_CHECK_ENABLED = True
COMMS_LOSS_ENABLED = True
IMU_ENABLED = True
TEMP_SAFETY_ENABLED = False  # thermal safety disabled; temps still displayed

# -------------------- Torque spike detection --------------------
# Conservative per-model torque limits (Nm), below rated peak torque.
TORQUE_LIMITS_NM = {
    "rs-04": 80.0,   # rated 40 Nm, peak 120 Nm
    "rs-03": 40.0,   # rated 21 Nm, peak 60 Nm
    "rs-02": 12.0,   # rated 7 Nm,  peak 17 Nm
}
TORQUE_DEFAULT_LIMIT_NM = 30.0  # fallback when model is unknown
TORQUE_CONSECUTIVE_LIMIT = 3     # consecutive over-limit samples before tripping

# -------------------- Ground contact detection --------------------
LEFT_ANKLE_ID = 5
RIGHT_ANKLE_ID = 10
ANKLE_CONTACT_MIN_TORQUE_NM = 0.5

LEFT_HIP_ID = 1
RIGHT_HIP_ID = 6
HIP_CONTACT_MIN_TORQUE_NM = 0.3

# -------------------- Communication loss detection --------------------
COMMS_MAX_CONSECUTIVE_FAILURES = 5  # at 60 Hz, 5 failures ~= 83 ms of silence

# -------------------- IMU fall detection --------------------
IMU_SERIAL_PORT = "/dev/ttyACM0"
IMU_BAUD = 115200
IMU_RATE_HZ = 100.0
IMU_FALL_ROLL_DEG = 40.0
IMU_FALL_PITCH_DEG = 40.0
IMU_CONFIRM_COUNT = 3
IMU_USE_INTEGRATOR = True  # False => use firmware roll/pitch directly

# -------------------- Temperature supervision --------------------
TEMP_DERATE_START_C = 65.0   # start slowing motion
TEMP_HOLD_C = 75.0           # freeze at current pose
TEMP_DISABLE_C = 85.0        # disable motor
TEMP_REENABLE_C = 70.0       # cool below this before re-enable (hysteresis)
TEMP_VALID_MIN_C = -40.0     # reject implausible low telemetry
TEMP_VALID_MAX_C = 200.0     # reject implausible high telemetry
TEMP_MAX_STEP_C = 40.0       # reject one-sample temperature spikes

DERATE_MIN_SCALE = 0.20      # minimum motion scale at/above disable threshold
DISABLE_COOLDOWN_S = 2.0     # stay disabled for this long before re-enable

# -------------------- Delay measurement knobs --------------------
STEP_CMD_EPS_RAD = 0.005     # detect "command started moving" (~0.29 deg)
STEP_POS_EPS_RAD = 0.010     # detect "position started moving" (~0.57 deg)
STEP_TIMEOUT_S = 2.0         # give up if no motion is seen in this time
