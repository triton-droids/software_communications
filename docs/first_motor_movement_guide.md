# First RobStride Motor Movement Guide

This guide explains how to move one robot leg motor using `utils/gain_tuner.py`.

Use this only after the no-movement CAN test works. The no-movement test is documented in:

```text
docs/no_movement_motor_test_guide.md
```

## Goal

The goal is to move **one motor by a very small amount** and verify that:

- the motor can be enabled
- the motor can hold its current position
- the motor can receive a small position command
- the reported position changes

Start with one motor. Do not start with all motors.

## Safety

This process can move the robot.

Before running the tuner:

- support the robot so it cannot fall
- keep hands, cables, and tools away from joints
- make sure someone can cut motor power quickly
- test one motor at a time
- start with small commands like `step -1` or `step 1`

Do not run the full leg demo until single-motor movement works.

## Required Setup

CAN must already be up.

Check:

```bash
ip link show can0
```

Good output includes:

```text
UP
LOWER_UP
state UP
```

If `can0` is not up, bring it up first:

```bash
cd ~/Documents/embedded
./setup.sh
```

or manually:

```bash
sudo modprobe slcan
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Use the correct `/dev/ttyACM*` device for your adapter.

## Step 1: Optional No-Movement Scan

Before moving anything, confirm that the motors reply:

```bash
cd ~/Documents/embedded
python3 utils/validation_code/id_check.py --channel can0 --bitrate 1000000 --start 1 --end 10
```

This does not move motors.

## Step 2: Start the Tuner

Use the repo virtual environment:

```bash
cd ~/Documents/embedded
./.venv/bin/python utils/gain_tuner.py
```

If system `python3` gives an import error, use `./.venv/bin/python` as shown above.

The tuner asks:

```text
Motor IDs:
```

For the first test, enter one motor only.

Example for left knee motor `4`:

```text
4
```

The tuner will connect, enable that motor, set MIT mode, read the current position, and command the motor to hold its current position.

Expected message:

```text
Connected. Motors are holding their current position (no motion).
```

## Step 3: Check Status

At the tuner prompt, run:

```text
status
```

Example prompt:

```text
[ALL] >>
```

Even if you selected only motor `4`, the prompt may say `[ALL]` because all loaded motors are selected. If you loaded only motor `4`, `[ALL]` means motor `4`.

Important columns:

- `Pos(deg)`: measured motor/joint position
- `Cmd(deg)`: commanded target
- `Vel`: measured velocity
- `Tq`: measured torque
- `Kp` and `Kd`: control gains
- `Lim(rad)`: software joint limits

If `State` says `OFF`, that means temperature safety is off. It does **not** mean the motor is disabled.

## Step 4: Move Motor 4 or 9

Motor `4` and motor `9` are knee motors. Their configured range is approximately:

```text
[-2.094, 0.000] rad
```

That means their safe command direction from zero is usually negative.

For motor `4` or `9`, start with:

```text
step -1
```

Wait one or two seconds, then check:

```text
status
```

Then move back:

```text
step 1
```

Wait again, then:

```text
status
```

If `1` degree is too small to see, use:

```text
step -5
```

Wait, then:

```text
status
step 5
```

Do not immediately type the return command before looking at the joint. If you type `step -1` and then `step 1` right away, the motor may return before you notice movement.

## Step 5: Move Motor 5 or 10

Motor `5` and motor `10` are ankle motors. For a tiny first test:

```text
step 1
```

Wait and check:

```text
status
```

Then move back:

```text
step -1
```

## Step 6: Holding and Exiting

To stop any active motion command and hold the current measured position:

```text
hold
```

To exit and disable torque:

```text
q
```

The tuner should print that it disabled torque and disconnected.

## If the Motor Does Not Move

Run:

```text
status
```

If `Cmd(deg)` changes but `Pos(deg)` does not change:

- the command was accepted by the tuner
- the motor may not be applying enough torque
- the gains may be too low
- the motor may be physically blocked
- the wrong motor ID may be selected
- zeroing or direction may be wrong

Try slightly higher gains:

```text
kp 30
kd 0.5
step -5
```

For motor `4` or `9`, use `step -5`. Then return with:

```text
step 5
```

If it still does not move, stop:

```text
hold
q
```

Then inspect wiring, power, motor ID, mechanical blockage, and CAN traffic.

## If the Tuner Trips on Joint Limits

You may see:

```text
[SAFETY] motor 4 out of joint limits: pos=0.1070 rad, limits=[-2.0944,0.0000]
```

That means the motor's current position is outside the configured software limits.

For motor `4`, `0.1070 rad` is about `6.1 deg` past the expected zero. The tuner refuses to move because it sees an unsafe starting state.

Fix this by setting mechanical zero while the joint is physically in the correct zero pose.

### Zero One Motor

For motor `4` only:

```bash
cansend can0 0600FE04#0100000000000000
```

### Zero All Motors

Only do this when all joints are physically in their correct neutral zero pose:

```bash
cd ~/Documents/embedded
./zero_out.sh
```

This runs:

```bash
for i in {1..10}; do cansend can0 $(printf "0600FE%02X#0100000000000000" $i); sleep 0.05; done; echo "Set mechanical zeros"
```

Only zero a motor when the joint is physically in the pose that software should call `0`.

## Useful Tuner Commands

```text
status
```

Print current position, command, velocity, torque, temperature, gains, direction, and limits.

```text
step <deg>
```

Move relative to the current command by a number of degrees.

Examples:

```text
step -1
step 1
step -5
step 5
```

```text
goto <deg>
```

Move to an absolute angle in degrees. Avoid this for the first test unless you are sure zeroing is correct.

```text
sine <amp_deg> <freq_hz> [duration_s]
```

Run a sinusoidal command. Avoid this until simple steps work.

A small slow example:

```text
sine 2 0.2 5
```

```text
stop
```

Stop sine excitation.

```text
hold
```

Hold the current measured position.

```text
q
```

Quit and disable torque.

## Recommended First Test Sequence

For motor `4`:

```bash
cd ~/Documents/embedded
./.venv/bin/python utils/gain_tuner.py
```

Then:

```text
Motor IDs: 4
[ALL] >> status
[ALL] >> step -5
```

Wait one or two seconds.

```text
[ALL] >> status
[ALL] >> step 5
```

Wait one or two seconds.

```text
[ALL] >> status
[ALL] >> hold
[ALL] >> q
```

If that works, repeat one motor at a time.

## When to Use the Leg Demo

Only use the leg demo after single-motor tests work.

The leg demo moves motors `4`, `5`, `9`, and `10` in a repeated pattern:

```bash
python3 ctrl_scripts/leg_swing_test.py
```

Treat it as much riskier than a small `step` command in `gain_tuner.py`.

