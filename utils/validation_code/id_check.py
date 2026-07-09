#!/usr/bin/env python3
"""
RobStride GET_DEVICE_ID scan (no enable, no movement).
Scans CAN_IDs 1..10 and reports any replies.

Run:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set up can0
  python3 robstride_scan_1_10.py --channel can0 --timeout 0.3

Notes:
- Sends only GET_DEVICE_ID (comm_type=0). No torque enable, no movement.
"""

import argparse
import time
import can

# RobStride protocol constants (from your repo)
GET_DEVICE_ID = 0
DEFAULT_HOST_ID = 0xFF


def build_ext_id(comm_type: int, extra_data: int, device_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((extra_data & 0xFFFF) << 8) | (device_id & 0xFF)


def send_get_device_id(bus: can.Bus, device_id: int, host_id: int):
    msg = can.Message(
        arbitration_id=build_ext_id(GET_DEVICE_ID, host_id, device_id),
        is_extended_id=True,
        data=b"\x00" * 8,
    )
    bus.send(msg)


def parse_comm_type(arbitration_id: int) -> int:
    return (arbitration_id >> 24) & 0x1F


def parse_extra(arbitration_id: int) -> int:
    return (arbitration_id >> 8) & 0xFFFF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--end", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=0.3, help="seconds to wait per ID")
    ap.add_argument("--gap", type=float, default=0.02, help="seconds between pings")
    ap.add_argument("--host-id", type=lambda x: int(x, 0), default=DEFAULT_HOST_ID)
    args = ap.parse_args()

    bus = can.interface.Bus(interface="socketcan", channel=args.channel, bitrate=args.bitrate)

    found = {}  # id -> reply info
    try:
        print(f"[+] Scanning IDs {args.start}..{args.end} on {args.channel} @ {args.bitrate} bps")
        print(f"[+] Host/main CAN ID: 0x{args.host_id:02X}")
        print("[+] Sending GET_DEVICE_ID only (no enable, no movement).")

        for device_id in range(int(args.start), int(args.end) + 1):
            print(f"\n[>] Ping id={device_id} ...")
            send_get_device_id(bus, device_id, args.host_id)
            time.sleep(args.gap)

            t_end = time.time() + float(args.timeout)
            got_reply = False

            while time.time() < t_end:
                msg = bus.recv(timeout=0.05)
                if msg is None or not msg.is_extended_id:
                    continue

                comm_type = parse_comm_type(msg.arbitration_id)
                if comm_type != GET_DEVICE_ID:
                    continue

                extra = parse_extra(msg.arbitration_id)
                host_id = msg.arbitration_id & 0xFF

                print("[+] Reply received!")
                print(f"    arb_id=0x{msg.arbitration_id:08X}  extra=0x{extra:04X}  host_id=0x{host_id:02X}")
                print(f"    data={msg.data.hex()}  (often includes UUID bytes)")
                found[device_id] = {
                    "arb_id": msg.arbitration_id,
                    "extra": extra,
                    "host_id": host_id,
                    "data_hex": msg.data.hex(),
                }
                got_reply = True
                break

            if not got_reply:
                print("[ ] No reply.")

        print("\n" + "=" * 60)
        if found:
            print("[+] Scan complete. Responding IDs:")
            for k in sorted(found.keys()):
                print(f"  - ID {k}: data={found[k]['data_hex']}")
        else:
            print("[!] Scan complete. No replies from IDs in range.")
            print("    Check: bitrate=1Mbps, wiring/termination, power, correct bus (can0), and that motors are on the bus.")
        print("=" * 60)

    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
