# RobStride Gain Tuner (modular)

Maintainable refactor of `robstride_gain_tuner_liveplot_mac_safe.py`.

## Run

```bash
# from repo root
python3 -m utils.robstride_gain_tuner
# or
python3 utils/robstride_gain_tuner_liveplot_mac_safe.py
```

## Modules

| File | Responsibility |
| --- | --- |
| `config.py` | All tunable constants (joint limits, torque limits, IMU/thermal thresholds, timing) |
| `state.py` | `Excitation` and `MotorState` dataclasses |
| `logging_utils.py` | Math helpers + debug logging + exception hooks |
| `imu.py` | IMU fall detector and compatibility import of `imu_stream` / `imu_read` |
| `tuner.py` | `GainTunerMIT` core (connect, control loop, safety, user commands) |
| `plotter.py` | Matplotlib live plotter (main-thread control) |
| `cli.py` | Motor ID parsing and interactive command loop |
| `main.py` | Entry point wiring |
| `__main__.py` | Enables `python -m utils.robstride_gain_tuner` |

## Notes

- RobStride SDK is imported lazily from `robstride_dynamics` (installed in
  `rosenv`) or the repo-root fallback.
- If the optional `utils/actuation_safety.py` monitor is missing, it is skipped
  with a warning (it is disabled by default via `ACTUATION_SAFETY_ENABLED`).
- If no usable `imu_stream` / `imu_read` module is found, IMU fall detection is
  disabled with a warning instead of crashing.
