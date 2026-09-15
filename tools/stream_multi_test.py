#!/usr/bin/env python3
"""Multi-joint STREAM_TARGET test at a fixed frame rate (binary NOET protocol).

Drives every joint of one arm with an independent sine trajectory (per-joint
amplitude / frequency / phase chosen to stay well inside soft limits and the
configured per-frame speed cap), sends frames at --fps, and records tracking
error from STATUS snapshots. Counts ACK / REJECT / ERROR replies. Ends with
STOP + RESET (also on Ctrl+C).

Per-joint sine parameters (deg, Hz, rad phase) are sized so the peak slew
A*2*pi*f stays under ~8 deg/s (cap is 20 deg/s for all joints now).

Usage:
  python3 tools/stream_multi_test.py --host 192.168.127.40 --fps 30 --seconds 30
"""

import argparse
import csv
import math
import socket
import struct
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from pid_step_test import Client, JOINT_NAMES, KINDS, frame, parse_frame  # noqa: E402
from range_motion_test import load_soft_limits  # noqa: E402

# name -> (amplitude_deg, freq_hz, phase_rad)
SINE = {
    "LJ1": (8.0, 0.10, 0.0),
    "LJ2": (6.0, 0.13, 0.9),
    "LJ3": (10.0, 0.08, 1.8),
    "LJ4": (8.0, 0.15, 2.7),
    "LJ5": (10.0, 0.11, 3.6),
    "LJ6": (8.0, 0.18, 4.5),
    "LJ7": (10.0, 0.09, 5.4),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.127.40")
    ap.add_argument("--port", type=int, default=8890)
    ap.add_argument("--arm", choices=["left", "right"], default="left")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--margin", type=float, default=4.0)
    ap.add_argument("--ttl", type=int, default=3000)
    ap.add_argument("--control-ttl", type=int, default=120000)
    ap.add_argument("--config", default="dual_arm_joints.cfg")
    ap.add_argument("--output", help="CSV output path")
    args = ap.parse_args()

    joints = [f"{args.arm[0].upper()}J{i}" for i in range(1, 8)]
    limits = load_soft_limits(args.config)
    mask = 0x007F if args.arm == "left" else 0x3F80

    client = Client(args.host, args.port)
    if not client.ping():
        print(f"PING failed for {args.host}:{args.port}", file=sys.stderr)
        return 1

    out = None
    writer = None
    if args.output:
        out = open(args.output, "w", newline="")
        writer = csv.writer(out)
        writer.writerow(["t_s", "joint", "pos_deg", "target_deg", "err_steps"])

    sent = ack = replies = 0
    stats = {j: 0 for j in joints}  # max |err| steps per joint
    t0 = time.monotonic()

    try:
        st = client.status()
        center = {j: st["pos"][JOINT_NAMES.index(j)] for j in joints}
        # clamp sine center so the whole swing stays inside soft limits
        for j in joints:
            amp = SINE[j][0]
            lo, hi = limits[j]
            center[j] = max(lo + args.margin + amp,
                            min(hi - args.margin - amp, center[j]))
        print("centers:", {j: round(v, 1) for j, v in center.items()})
        seq, msg = client.control(mask, args.control_ttl)
        print(f"CONTROL mask=0x{mask:04x}: {msg}")

        st = client.status()
        hold = list(st["target"])
        period = 1.0 / args.fps
        duration = max(25, min(1000, int(period * 1000)))
        # per-frame slew limit: stay well under the 20 deg/s receiver cap
        step_max = 15.0 * period
        current = {j: center[j] for j in joints}
        next_frame = time.monotonic()
        last_sample = 0.0
        client.sock.settimeout(0)  # non-blocking reply drain

        while time.monotonic() - t0 < args.seconds:
            now = time.monotonic()
            t = now - t0
            targets = list(hold)
            for j in joints:
                amp, freq, phase = SINE[j]
                idx = JOINT_NAMES.index(j)
                ref = center[j] + amp * math.sin(2 * math.pi * freq * t + phase)
                delta = ref - current[j]
                delta = max(-step_max, min(step_max, delta))
                current[j] += delta
                targets[idx] = current[j]
            payload = struct.pack(">II", args.ttl, duration)
            payload += struct.pack(">14d", *targets)
            client.sock.sendto(frame(KINDS["STREAM_TARGET"],
                                     client._next_seq(), payload),
                               client.addr)
            sent += 1
            # drain replies without blocking
            while True:
                try:
                    data, _ = client.sock.recvfrom(4096)
                except (BlockingIOError, socket.error):
                    break
                parsed = parse_frame(data)
                if parsed is None:
                    continue
                if parsed[0] == 10:
                    ack += 1
                elif parsed[0] in (11, 12):
                    replies += 1
                    if parsed[0] == 12 and replies <= 5:
                        print(f"  frame ERROR: {parsed[2].hex()}")
            # sample tracking error at ~10 Hz
            if now - last_sample >= 0.1:
                client.sock.settimeout(0.2)
                st = client.status()
                client.sock.settimeout(0)
                if st["fault"]:
                    raise RuntimeError("receiver FAULT: " + st["message"])
                for j in joints:
                    idx = JOINT_NAMES.index(j)
                    stats[j] = max(stats[j], abs(st["err"][idx]))
                    if writer:
                        writer.writerow([f"{t:.2f}", j,
                                         f"{st['pos'][idx]:.3f}",
                                         f"{st['target'][idx]:.3f}",
                                         st["err"][idx]])
                last_sample = now
            next_frame += period
            time.sleep(max(0.0, next_frame - time.monotonic()))
        # final drain to account for replies still in flight
        client.sock.settimeout(0.5)
        while True:
            try:
                data, _ = client.sock.recvfrom(4096)
            except (socket.timeout, BlockingIOError, socket.error):
                break
            parsed = parse_frame(data)
            if parsed is None:
                continue
            if parsed[0] == 10:
                ack += 1
            elif parsed[0] in (11, 12):
                replies += 1
    except KeyboardInterrupt:
        print("\ninterrupted, stopping...", file=sys.stderr)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
    finally:
        client.sock.settimeout(0.2)
        client.stop()
        client.reset(args.control_ttl)
        if out:
            out.close()

    elapsed = time.monotonic() - t0
    print(f"\n==== summary ====")
    print(f"frames sent={sent} in {elapsed:.1f}s "
          f"({sent / max(elapsed, 0.1):.1f} fps)  ACK={ack}  RESULT/ERROR={replies}")
    for j in joints:
        print(f"{j}: max|err| = {stats[j]} steps "
              f"({stats[j] / 11.3778:.2f} deg)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
