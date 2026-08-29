"""Dataclasses shared by the tuner and live plotter."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Excitation:
    """Excitation profile for one motor."""

    mode: str = "none"  # "none" | "sine"
    amp_rad: float = 0.0
    freq_hz: float = 0.0
    t0: float = 0.0
    duration_s: Optional[float] = None
    center_rad: float = 0.0


@dataclass
class MotorState:
    """Runtime state for one motor.

    Position/velocity are kept in logical joint space; ``direction`` maps
    between logical and physical (raw) motor space.
    """

    id: int
    name: str
    model: str

    # Telemetry
    raw_position: float = 0.0   # raw physical motor angle (rad)
    position: float = 0.0       # adjusted logical angle (rad)
    velocity: float = 0.0       # adjusted logical velocity (rad/s)
    torque: float = 0.0         # Nm
    temperature: float = 0.0    # C

    # Gains
    kp: float = 10.0
    kd: float = 0.2

    # Mount direction (1 normal, -1 inverted)
    direction: int = 1

    # Joint limits (logical space)
    limit_lo: float = -math.inf
    limit_hi: float = math.inf

    # Targets (logical, pre-direction)
    target_rad: float = 0.0
    commanded_target_rad: float = 0.0
    hold_center_rad: float = 0.0
    startup_motor_offset_rad: float = 0.0
    bypass_limit_clamp: bool = False  # hold out-of-limits startup pose until user commands motion

    # Excitation state
    excitation: Excitation = field(default_factory=Excitation)

    # Temp safety state
    temp_state: str = "OK"  # "OK" | "DERATE" | "HOLD" | "DISABLED"
    enabled: bool = True
    last_disable_t: float = 0.0

    last_error: Optional[str] = None

    # -------------------- Timing / delay metrics --------------------
    loop_dt_ms: float = 0.0        # wall-time between control_step calls
    write_dt_ms: float = 0.0       # duration of write_operation_frame()
    read_dt_ms: float = 0.0        # duration of read_operation_frame()
    io_gap_ms: float = 0.0         # (read_end - write_end) in ms
    last_write_end_t: float = 0.0  # epoch seconds when last write finished
    last_read_end_t: float = 0.0   # epoch seconds when last read finished
    prev_sent_phys_target: float = 0.0
    step_pending: bool = False
    step_cmd_t: float = 0.0        # epoch seconds when first changed command was sent
    step_pos0: float = 0.0         # position at that time
    last_step_delay_s: float = math.nan
    consecutive_torque_spikes: int = 0
    consecutive_read_failures: int = 0
