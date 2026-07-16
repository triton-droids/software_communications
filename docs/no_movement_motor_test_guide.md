# No-Movement RobStride Motor Test Guide

This guide explains how to check that the robot leg motors are powered, connected, and reachable over CAN **without enabling torque and without commanding movement**.

It is written for someone who has not used CAN, RobStride motors, or this repo before.

## Goal

By the end of this process, you should know whether the laptop can communicate with motor IDs `1` through `10`.

This test proves:

- the USB-C CAN adapter is detected by Linux
- the `can0` interface is up
- the motors have power
- the motors are reachable over CAN
- the motor IDs are correct

This test does **not** prove:

- the motors can apply torque
- the joints are mechanically safe
- the legs can walk
- the gains are tuned correctly

Those require later tests that enable or move the motors.

## Safety First

For this guide, do **not** run these scripts:

```bash
python3 utils/gain_tuner.py
python3 ctrl_scripts/leg_swing_test.py
python3 ctrl_scripts/run_policy.py
```

Those scripts can enable motors, hold position, or move legs.

The safe no-movement command in this guide is:

```bash
python3 utils/validation_code/id_check.py --channel can0 --bitrate 1000000 --start 1 --end 10
```

It sends `GET_DEVICE_ID` messages only. It does not enable torque and does not command motion.

## Hardware Needed

- robot leg motor power connected
- Blue/Green USB-C to CAN bus adapter
- USB-C cable from adapter to laptop
- CAN wiring from adapter to robot motor bus
- Linux laptop

## Step 1: Plug In the CAN Adapter

Plug the Blue/Green USB-C CAN adapter into the laptop.

Then check which serial device it became:

```bash
ls /dev/ttyACM*
```

Example output:

```text
/dev/ttyACM0
```

In that case, your adapter is `/dev/ttyACM0`.

If you see multiple devices, unplug other USB serial devices and run the command again.

## Step 2: Go to the Repo

Open a terminal and go to this repo:

```bash
cd ~/Documents/embedded
```

This matters because commands like:

```bash
python3 utils/validation_code/id_check.py
```

only work when your terminal is inside the `embedded` repo.

If you run from the wrong directory, you may see an error like:

```text
python3: can't open file '/home/droids/utils/validation_code/id_check.py': [Errno 2] No such file or directory
```

Fix it by running:

```bash
cd ~/Documents/embedded
```

## Step 3: Bring Up CAN

If your adapter is `/dev/ttyACM0`, run:

```bash
sudo modprobe slcan
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

What these commands do:

- `modprobe slcan`: loads Linux support for serial CAN adapters
- `slcand ... /dev/ttyACM0 can0`: creates a CAN interface named `can0`
- `ip link set can0 down`: temporarily stops `can0` so it can be configured
- `ip link set can0 type can bitrate 1000000`: sets CAN speed to 1 Mbps
- `ip link set can0 up`: starts the CAN interface

You can also use the repo helper:

```bash
./setup.sh
```

That script tries to auto-detect the `/dev/ttyACM*` device.

## Step 4: Confirm `can0` Is Up

Run:

```bash
ip link show can0
```

Good output looks like:

```text
can0: <NOARP,UP,LOWER_UP> ... state UP
```

The important parts are:

- `UP`
- `LOWER_UP`
- `state UP`

If `can0` does not exist, repeat Step 3 and check that the adapter path is correct.

## Step 5: Optional CAN Monitor

Open a second terminal and run:

```bash
candump -x -t a can0
```

This listens to CAN traffic. It does not transmit anything and does not move motors.

It may show nothing at first. That is normal if nothing is sending CAN messages.

Leave this terminal open while running the scan in the next step. When the scan runs, `candump` should show CAN frames.

## Step 6: Scan Motor IDs Without Moving

In the first terminal, make sure you are in the repo:

```bash
cd ~/Documents/embedded
```

Then run:

```bash
python3 utils/validation_code/id_check.py --channel can0 --bitrate 1000000 --start 1 --end 10
```

This scans IDs `1` through `10`.

Expected result:

```text
[+] Scanning IDs 1..10 on can0 @ 1000000 bps
[+] Sending GET_DEVICE_ID only (no enable, no movement).

[>] Ping id=1 ...
[+] Reply received!

[>] Ping id=2 ...
[+] Reply received!
...
```

At the end, it should list the responding IDs.

## Step 7: Interpret Results

Best result:

```text
Responding IDs:
  - ID 1
  - ID 2
  - ID 3
  - ID 4
  - ID 5
  - ID 6
  - ID 7
  - ID 8
  - ID 9
  - ID 10
```

That means all 10 motors are reachable over CAN.

If only some IDs reply:

- those motors are reachable
- missing IDs may have no power, wrong CAN ID, wiring issue, or bus issue

If no IDs reply:

- check motor power
- check CAN adapter wiring
- check CAN termination
- check that `can0` is up
- check that bitrate is `1000000`
- check that you are using the right `/dev/ttyACM*`

## Common Problems

### `python3: can't open file ...`

You are probably in the wrong folder.

Fix:

```bash
cd ~/Documents/embedded
python3 utils/validation_code/id_check.py --channel can0 --bitrate 1000000 --start 1 --end 10
```

### `candump` Shows Nothing

That can be normal. `candump` only shows traffic when something is transmitting.

Run the motor scan in another terminal:

```bash
python3 utils/validation_code/id_check.py --channel can0 --bitrate 1000000 --start 1 --end 10
```

Then check whether `candump` prints frames.

### `dmesg: read kernel buffer failed: Operation not permitted`

Use sudo:

```bash
sudo dmesg | tail -30
```

This is only for checking USB device messages. It is not required if you already found `/dev/ttyACM0`.

### `can0` Already Exists

If `can0` already exists and is up, you can usually continue.

Check:

```bash
ip link show can0
```

If it is broken or stale after replugging the adapter, bring it down and recreate the setup:

```bash
sudo ip link set can0 down
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## What Not to Run Yet

Do not run:

```bash
python3 ctrl_scripts/leg_swing_test.py
```

That script moves the legs.

Do not run:

```bash
python3 utils/gain_tuner.py
```

That script is designed for tuning and holding motors. On startup it enables motors and writes hold commands at the current position.

Do not run:

```bash
python3 ctrl_scripts/run_policy.py
```

That script runs the robot controller and can command all joints.

## Next Step After No-Movement Test

Once all IDs respond, the next lowest-risk test is usually a **hold-current-position** test. That is not a no-movement test because the motors are enabled and can become stiff.

Only do that when:

- the robot is supported
- the legs are clear of people
- you have a fast way to cut power
- someone understands that motors may hold or twitch

The no-movement test is complete once `id_check.py` sees the expected motor IDs.

