#!/usr/bin/env python3
"""Full-range motion test for selected joints over the binary NOETIX protocol.

Sweeps each given joint across its configured soft-limit range (with a
safety margin) in segments, logging position/target/tracking-error at
--sample-hz, then returns to the start pose. Reports per joint: max
|tracking error| during motion, arrival error at each waypoint, and any
receiver fault. Always finishes with STOP + RESET (also on Ctrl+C).

Soft limits are read from the joint config file (same format as
dual_arm_joints.cfg on the robot).

Usage:
  python3 tools/range_motion_test.py --host 192.168.127.40 \
      --joints LJ1,LJ2 --config dual_arm_joints.cfg --output out.csv
"""

import argparse
import csv
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from pid_step_test import Client, JOINT_NAMES, STEPS_PER_DEG  # noqa: E402


def load_soft_limits(path):
    """Parse rows like `LJ1=dir zero calib soft_min soft_max ... P D I`."""
    limits = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, rest = line.partition("=")
            key = key.strip()
            if key not in JOINT_NAMES:
                continue
            fields = rest.split()
            if len(fields) < 14:
                continue
            limits[key] = (float(fields[3]), float(fields[4]))
    return limits


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.127.40")
    ap.add_argument("--port", type=int, default=8890)
    ap.add_argument("--joints", required=True, help="e.g. LJ1,LJ2")
    ap.add_argument("--control-scope", choices=["tested", "left", "right", "both"],
                    default="tested",
                    help="which joints enter CONTROL (torque on, hold pose); "
                         "'tested' = only the --joints, 'left' = whole left arm")
    ap.add_argument("--config", default="dual_arm_joints.cfg",
                    help="joint config with soft limits")
    ap.add_argument("--margin", type=float, default=2.0,
                    help="stay this many degrees inside soft limits")
    ap.add_argument("--speed", type=float, default=5.0,
                    help="nominal deg/s used to size segment durations")
    ap.add_argument("--sample-hz", type=float, default=20.0)
    ap.add_argument("--ttl", type=int, default=120000)
    ap.add_argument("--output", help="CSV output path")
    args = ap.parse_args()

    joints = [j.strip().upper() for j in args.joints.split(",") if j.strip()]
    for j in joints:
        if j not in JOINT_NAMES:
            ap.error(f"unknown joint {j}")
    limits = load_soft_limits(args.config)
    missing = [j for j in joints if j not in limits]
    if missing:
        ap.error(f"no soft limits in {args.config} for {missing}")

    client = Client(args.host, args.port)
    if not client.ping():
        print(f"PING failed for {args.host}:{args.port}", file=sys.stderr)
        return 1

    out = None
    writer = None
    if args.output:
        out = open(args.output, "w", newline="")
        writer = csv.writer(out)
        writer.writerow(["t_ms", "joint", "waypoint_deg", "t_rel_s",
                         "pos_deg", "target_deg", "err_steps", "fault"])

    if args.control_scope == "left":
        mask = 0x007F
    elif args.control_scope == "right":
        mask = 0x3F80
    elif args.control_scope == "both":
        mask = 0x3FFF
    else:
        mask = 0
        for j in joints:
            mask |= 1 << JOINT_NAMES.index(j)

    t0 = time.monotonic()
    last_hb = time.monotonic()
    summary = {}

    def maybe_heartbeat():
        nonlocal last_hb
        if time.monotonic() - last_hb > 20:
            client.heartbeat(args.ttl)
            last_hb = time.monotonic()

    def move_and_track(jidx, jname, dest, stats):
        """Issue one direct TARGET to dest (device-profiled move) and
        sample until completion plus a small settle tail."""
        st = client.status()
        if st["fault"]:
            raise RuntimeError("receiver FAULT: " + st["message"])
        cur = st["target"][jidx]
        delta = dest - cur
        if abs(delta) < 0.05:
            return
        targets = list(st["target"])
        targets[jidx] = dest
        # receiver rejects trajectories faster than max_speed_deg_s (10),
        # the servo speed register caps at ~8.8 deg/s — size generously
        duration = int(abs(delta) / args.speed * 1000) + 500
        seq, _ = client.target(targets, args.ttl, duration)
        start = time.monotonic()
        done_at = None
        while True:
            st = client.status()
            if st["fault"]:
                raise RuntimeError("receiver FAULT during motion: "
                                   + st["message"])
            err = st["err"][jidx]
            stats["max_abs_err"] = max(stats["max_abs_err"], abs(err))
            if writer:
                writer.writerow([int((time.monotonic() - t0) * 1000),
                                 jname, f"{dest:.2f}",
                                 f"{time.monotonic() - start:.3f}",
                                 f"{st['pos'][jidx]:.4f}",
                                 f"{st['target'][jidx]:.4f}", err, 0])
            if st["result"] >= seq:
                done_at = done_at or time.monotonic()
                if time.monotonic() - done_at > 0.5:
                    stats["final_err"] = err
                    stats["result_code"] = st["result_code"]
                    break
            if time.monotonic() - start > duration / 1000.0 + 30:
                stats["final_err"] = err
                stats["result_code"] = -1
                print(f"  {jname}: completion timeout near {dest:.1f}")
                break
            time.sleep(1.0 / args.sample_hz)
        maybe_heartbeat()

    try:
        base = client.status()
        start_pos = {j: base["target"][JOINT_NAMES.index(j)] for j in joints}
        print("start pos:", {j: round(v, 2) for j, v in start_pos.items()})
        seq, msg = client.control(mask, args.ttl)
        print(f"CONTROL mask=0x{mask:04x}: {msg}")

        for jname in joints:
            jidx = JOINT_NAMES.index(jname)
            lo, hi = limits[jname]
            lo += args.margin
            hi -= args.margin
            home = start_pos[jname]
            home = max(lo, min(hi, home))
            stats = {"max_abs_err": 0, "final_err": 0, "result_code": 0}
            print(f"{jname}: sweep {home:.1f} -> {lo:.1f} -> {hi:.1f} -> {home:.1f}")
            for dest in (lo, hi, home):
                move_and_track(jidx, jname, dest, stats)
                print(f"  reached {dest:+.1f}  final_err={stats['final_err']}st "
                      f"rcode={stats['result_code']} "
                      f"max|err|={stats['max_abs_err']}st")
            summary[jname] = stats
    except KeyboardInterrupt:
        print("\ninterrupted, stopping...", file=sys.stderr)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
    finally:
        client.stop()
        client.reset(args.ttl)
        if out:
            out.close()

    print("\n==== summary ====")
    for jname, s in summary.items():
        print(f"{jname}: max|err| during motion = {s['max_abs_err']} steps "
              f"({s['max_abs_err'] / STEPS_PER_DEG:.2f} deg), "
              f"final arrival err = {s['final_err']} steps, "
              f"result_code = {s['result_code']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
