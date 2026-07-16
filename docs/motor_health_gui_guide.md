# RobStride Motor Health GUI Guide

This guide explains how to use:

```bash
utils/motor_health_gui.py
```

The GUI is for cautious single-motor or small-group motor testing. It can scan motors, enable and hold selected motors, show live health, send small step commands, set mechanical zero, and disable motors.

## Start the GUI

Make sure `can0` is up first:

```bash
ip link show can0
```

Then run:

```bash
cd ~/Documents/embedded
./.venv/bin/python utils/motor_health_gui.py
```

If `can0` is not up:

```bash
cd ~/Documents/embedded
./setup.sh
```

## Safe First Use

1. Put one motor ID in `Motor IDs`, for example:

```text
4
```

2. Click `Scan 1-10`.

This checks which motors reply over CAN. It does not enable torque and does not move motors.

3. Click `Connect + Hold`.

The GUI will first ask whether the selected motors have been mechanically zeroed in their correct physical zero pose. Choose `No` if you are not sure.

If you choose `Yes`, the GUI enables the selected motor, reads its current position, and commands it to hold that same position.

4. Select the motor row in the table.

5. Use a small step.

For knee motors `4` or `9`, start with:

```text
Step deg: 5
```

Then click `- Step`.

To return, click `+ Step`.

For ankle motors `5` or `10`, start with `+ Step`, then return with `- Step`.

6. Click `Hold Selected`.

7. Click `Disable Selected` or close the GUI.

## What the Table Shows

- `ID`: motor CAN ID
- `Model`: RobStride model used for command scaling
- `Enabled`: whether the GUI believes the motor is enabled
- `Pos deg`: measured logical position
- `Cmd deg`: commanded logical position
- `Vel deg/s`: measured velocity
- `Torque`: reported motor torque
- `Temp C`: reported motor temperature
- `Health`: `OK`, `DIS`, `LIMIT`, `STALE`, or `ERR`
- `Last Error`: most recent communication/control error

## Buttons

`Scan 1-10`: passive no-movement scan.

`Connect + Hold`: enables listed motors and holds current position.

Before enabling, the GUI always asks whether the selected motors have been mechanically zeroed. This is intentional: wrong zeroing can make later commands move in surprising directions.

`- Step` / `+ Step`: moves selected motors by `Step deg`.

`Goto 0 Selected`: commands selected motors to logical `0` degrees. Only use this when mechanical zeroing is known correct.

`Hold Selected`: sets the command to the current measured position.

`Set Zero Selected`: sends the mechanical-zero command to selected motors. Only use this when the selected joint is physically in the correct zero pose.

`Apply Gains`: updates `Kp` and `Kd` for selected motors.

`Disable Selected`: disables selected motors.

`Disable All`: disables every connected motor.

## Safety Notes

Start with one motor.

Do not use large steps. The GUI clamps step commands to 15 degrees, but first tests should be 1 to 5 degrees.

Use `Goto 0 Selected` carefully. If zeroing is wrong, logical `0` may not be the physical pose you expect.

Only click `Set Zero Selected` when the joint is physically in the pose software should call zero.

When clicking `Connect + Hold`, always answer the mechanical-zero question honestly. If the motor was not zeroed correctly, stop and zero it before enabling movement.

If `Cmd deg` changes but `Pos deg` does not, the command is being sent but the motor is not following. Check gains, power, wiring, motor ID, mechanical blockage, and zeroing.

If `Health` shows `LIMIT`, the motor's measured position is outside the configured software limits. The GUI disables that motor and blocks step commands. Fix the physical pose or set mechanical zero correctly, then reconnect before trying to move it.
