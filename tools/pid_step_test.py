#!/usr/bin/env python3
"""Per-joint step-response PID tester for the binary NOETIX UDP protocol.

The deployed receiver on 192.168.127.40:8890 speaks the binary "NOET" frame
protocol (see tools/send_dual_arm_udp.py); the older ASCII auto_tune_pid.py
no longer works against it. This tool reproduces the auto_tune_pid.py flow
over the binary protocol:

  per joint, per offset in --offsets (default +2 -2 +5 -5 +10 -10 deg):
    1. TARGET the joint from its baseline to baseline+offset (others hold)
    2. sample STATUS at --sample-hz for --settle seconds
    3. TARGET back to baseline, sample again
    4. compute overshoot, oscillation peak-peak, steady-state error,
       settling time, and a heuristic PID recommendation

Safety: uses small absolute targets relative to the *measured* baseline,
respects the receiver-side soft limits / speed caps, keeps a control lease
with HEARTBEAT, and always finishes with STOP + RESET (also on Ctrl+C).

Usage:
  python3 tools/pid_step_test.py --host 192.168.127.40 --joints LJ1,LJ2
  python3 tools/pid_step_test.py --host 192.168.127.40 --arm left
  python3 tools/pid_step_test.py --host 192.168.127.40 --arm both --offsets 2,5
"""

import argparse
import csv
import socket
import struct
import sys
import time
from datetime import datetime

MAGIC = 0x4E4F4554
KINDS = {"PING": 0, "STATUS": 1, "MODE": 2, "CONTROL": 3, "TARGET": 4,
         "STREAM_TARGET": 5, "HOLD": 6, "HEARTBEAT": 7, "STOP": 8,
         "RESET": 9, "ACK": 10, "RESULT": 11, "ERROR": 12, "PONG": 13,
         "STATE": 14}

JOINT_NAMES = [f"LJ{i}" for i in range(1, 8)] + [f"RJ{i}" for i in range(1, 8)]
STEPS_PER_DEG = 4096.0 / 360.0  # 11.378 steps/deg

# Mirrors dual_arm_joints.cfg on the robot (fields 12-14). Used only to
# anchor recommendations; the servo-side values are what count.
CURRENT_PID = {name: (28 if name.endswith("J1") else 32, 32, 0)
               for name in JOINT_NAMES}


def frame(kind, seq, payload=b""):
    buf = bytearray()
    buf += struct.pack(">I", MAGIC)
    buf.append(kind)
    buf += b"\0\0\0"
    buf += struct.pack(">Q", seq)
    buf += struct.pack(">I", len(payload))
    buf += payload
    buf += struct.pack(">I", sum(buf) & 0xFFFFFFFF)
    return bytes(buf)


def parse_frame(data):
    if len(data) < 24 or struct.unpack_from(">I", data, 0)[0] != MAGIC:
        return None
    kind = data[4]
    seq, plen = struct.unpack_from(">QI", data, 8)
    if 20 + plen + 4 != len(data):
        return None
    return kind, seq, data[20:20 + plen]


def decode_state(payload):
    req, mode, fault = struct.unpack_from(">QBB", payload, 0)
    active, qdepth = struct.unpack_from(">HH", payload, 12)
    mono, lease, submitted, active_seq, result_seq, result_code, span = \
        struct.unpack_from(">qqqqqII", payload, 16)
    offset = 16 + 5 * 8 + 2 * 4
    reason_len = struct.unpack_from(">H", payload, offset)[0]
    offset += 2 + reason_len
    msg_len = struct.unpack_from(">H", payload, offset)[0]
    offset += 2
    message = payload[offset:offset + msg_len].decode(errors="replace")
    offset += msg_len
    pos, target, err = [], [], []
    for idx in range(14):
        p, t, e = struct.unpack_from(">ddi", payload, offset + idx * 20)
        pos.append(p)
        target.append(t)
        err.append(e)
    return {"request": req, "mode": mode, "fault": fault,
            "active_mask": active, "queue_depth": qdepth, "lease_ms": lease,
            "submitted": submitted, "active": active_seq,
            "result": result_seq, "result_code": result_code,
            "message": message, "pos": pos, "target": target, "err": err}


class Client:
    def __init__(self, host, port, timeout=2.0):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.timeout = timeout
        # The receiver requires strictly increasing sequences across the whole
        # uptime of its peer state, not per process — seed from the clock.
        self.seq = int(time.time() * 1000) * 1000

    def _next_seq(self):
        self.seq += 1
        return self.seq

    def _exchange(self, kind, payload=b"", wait_kinds=(), seq=None):
        """Send one frame; return list of (kind, seq, payload) replies."""
        seq = seq or self._next_seq()
        self.sock.sendto(frame(kind, seq, payload), self.addr)
        replies = []
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                if replies and not wait_kinds:
                    break
                continue
            parsed = parse_frame(data)
            if parsed is None:
                continue
            replies.append(parsed)
            if parsed[0] in (12,):  # ERROR
                break
            if wait_kinds and parsed[0] in wait_kinds:
                break
            if not wait_kinds and parsed[0] in (11, 13, 14):
                break
        return seq, replies

    def ping(self):
        seq, replies = self._exchange(KINDS["PING"], wait_kinds=(13,))
        return any(k == 13 for k, _, _ in replies)

    def status(self):
        seq, replies = self._exchange(KINDS["STATUS"], wait_kinds=(14, 12))
        for k, _, p in replies:
            if k == 14:
                return decode_state(p)
        raise RuntimeError(f"STATUS failed: {replies!r}")

    def _command(self, verb, payload=b"", wait_s=None):
        """Send a command; wait for ACK and (optionally) RESULT."""
        kind = KINDS[verb]
        seq = self._next_seq()
        self.sock.sendto(frame(kind, seq, payload), self.addr)
        deadline = time.monotonic() + (wait_s if wait_s else self.timeout)
        acked = False
        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            parsed = parse_frame(data)
            if parsed is None:
                continue
            k, _, p = parsed
            if k == 12:
                code, mlen = struct.unpack_from(">HH", p, 8)
                raise RuntimeError(f"{verb} ERROR code={code} "
                                   f"{p[10:10 + mlen].decode(errors='replace')}")
            if k == 10:
                acked = True
                if wait_s is None:
                    return seq, "ACK"
            elif k == 11:
                code, mlen = struct.unpack_from(">IH", p, 8)
                msg = p[22:22 + mlen].decode(errors="replace") \
                    if len(p) >= 22 else ""
                return seq, f"RESULT code={code} {msg}"
        raise RuntimeError(f"{verb}: timeout (acked={acked})")

    def control(self, mask, ttl_ms):
        return self._command("CONTROL", struct.pack(">HI", mask, ttl_ms),
                             wait_s=30.0)

    def target(self, targets, ttl_ms, duration_ms, wait_s=None):
        payload = struct.pack(">II", ttl_ms, duration_ms)
        payload += struct.pack(">14d", *targets)
        return self._command("TARGET", payload, wait_s=wait_s)

    def wait_completion(self, seq, timeout_s=30.0):
        """Poll STATUS until the receiver marks command `seq` completed
        (its `result` field reaches our sequence). More robust than waiting
        for the RESULT frame, which STATUS polling may swallow."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            st = self.status()
            if st["fault"]:
                raise RuntimeError("receiver FAULT: " + st["message"])
            if st["result"] >= seq:
                return st["result_code"], st["message"]
            time.sleep(0.1)
        return None, "completion timeout"

    def heartbeat(self, ttl_ms):
        """HEARTBEAT is itself a sequenced command on the receiver — while
        its RESULT is outstanding, any other command is rejected as Busy.
        Send it sequenced and wait for completion."""
        seq, _ = self._command("HEARTBEAT", struct.pack(">I", ttl_ms))
        return self.wait_completion(seq, 5.0)

    def stop(self):
        try:
            seq, msg = self._command("STOP", wait_s=15.0)
            self.wait_completion(seq, 5.0)
            return msg
        except RuntimeError as exc:
            print(f"stop: {exc}", file=sys.stderr)

    def reset(self, ttl_ms):
        try:
            seq, msg = self._command("RESET", struct.pack(">I", ttl_ms),
                                     wait_s=15.0)
            self.wait_completion(seq, 5.0)
            return msg
        except RuntimeError as exc:
            print(f"reset: {exc}", file=sys.stderr)


def analyze(samples, step_sign):
    """samples: list of (t_rel_s, pos_deg, target_deg, err_steps) after the
    step command. Returns metrics dict."""
    if len(samples) < 5:
        return None
    final_target = samples[-1][2]
    dev = [(t, p - final_target) for t, p, _, _ in samples]
    n = len(dev)
    tail = dev[int(n * 0.7):]
    steady = sum(d for _, d in tail) / len(tail)
    pp = max(d for _, d in tail) - min(d for _, d in tail)
    if step_sign > 0:
        overshoot = max(d for _, d in dev)
    else:
        overshoot = -min(d for _, d in dev)
    band = 2.0 / STEPS_PER_DEG  # ±2 steps
    settling = dev[-1][0]
    for i in range(n - 1, -1, -1):
        if abs(dev[i][1]) > band:
            settling = dev[i][0]
            break
    else:
        settling = 0.0
    # count sign reversals of deviation in tail (oscillation indicator)
    reversals = 0
    last = 0
    for _, d in tail:
        s = 1 if d > band / 2 else (-1 if d < -band / 2 else 0)
        if s and last and s != last:
            reversals += 1
        if s:
            last = s
    return {"overshoot_deg": overshoot, "osc_pp_deg": pp,
            "steady_err_deg": steady, "settling_s": settling,
            "tail_reversals": reversals}


def recommend(metrics_by_offset, joint):
    p, d, i = CURRENT_PID.get(joint, (32, 32, 0))
    notes = []
    worst_osc = max(m["osc_pp_deg"] for m in metrics_by_offset)
    worst_over = max(m["overshoot_deg"] for m in metrics_by_offset)
    worst_steady = max(abs(m["steady_err_deg"]) for m in metrics_by_offset)
    worst_settle = max(m["settling_s"] for m in metrics_by_offset)
    if worst_osc > 5.0 / STEPS_PER_DEG:
        d = min(254, d + 8)
        notes.append(f"oscillation {worst_osc * STEPS_PER_DEG:.1f} steps pp -> D+8")
    if worst_over > 10.0 / STEPS_PER_DEG:
        p = max(0, p - 4)
        notes.append(f"overshoot {worst_over * STEPS_PER_DEG:.1f} steps -> P-4")
    if worst_steady > 3.0 / STEPS_PER_DEG:
        i = min(254, i + 1)
        notes.append(f"steady error {worst_steady * STEPS_PER_DEG:.1f} steps -> I+1")
    if worst_settle > 0.8 and worst_osc <= 5.0 / STEPS_PER_DEG:
        p = min(254, p + 4)
        notes.append(f"slow settling {worst_settle:.2f}s -> P+4")
    if not notes:
        notes.append("response looks OK")
    return (p, d, i), "; ".join(notes)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.127.40")
    ap.add_argument("--port", type=int, default=8890)
    ap.add_argument("--arm", choices=["left", "right", "both"], default="both")
    ap.add_argument("--joints", help="comma list, e.g. LJ1,RJ1 (overrides --arm)")
    ap.add_argument("--offsets", default="2,-2,5,-5,10,-10",
                    help="comma list of step sizes in degrees")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="sample window after each step (s)")
    ap.add_argument("--sample-hz", type=float, default=20.0)
    ap.add_argument("--duration-ms", type=int, default=1000,
                    help="TARGET duration hint per step")
    ap.add_argument("--ttl", type=int, default=60000)
    ap.add_argument("--output", help="CSV output path")
    args = ap.parse_args()

    if args.joints:
        joints = [j.strip().upper() for j in args.joints.split(",") if j.strip()]
    else:
        joints = ([f"LJ{i}" for i in range(1, 8)] if args.arm in ("left", "both") else []) + \
                 ([f"RJ{i}" for i in range(1, 8)] if args.arm in ("right", "both") else [])
    for j in joints:
        if j not in JOINT_NAMES:
            ap.error(f"unknown joint {j}")
    offsets = [float(x) for x in args.offsets.split(",") if x.strip()]

    client = Client(args.host, args.port)
    if not client.ping():
        print("PING failed: is noetix-dual-arm-udp running on "
              f"{args.host}:{args.port}?", file=sys.stderr)
        return 1

    out = None
    writer = None
    if args.output:
        out = open(args.output, "w", newline="")
        writer = csv.writer(out)
        writer.writerow(["t_ms", "joint", "offset_deg", "phase", "t_rel_s",
                         "pos_deg", "target_deg", "err_steps", "fault"])

    mask = 0
    for j in joints:
        mask |= 1 << JOINT_NAMES.index(j)

    t0 = time.monotonic()
    last_hb = 0.0
    results = {}

    def sample_window(joint_idx, duration_s):
        samples = []
        start = time.monotonic()
        period = 1.0 / args.sample_hz
        while time.monotonic() - start < duration_s:
            st = client.status()
            if st["fault"]:
                raise RuntimeError("receiver FAULT during sampling: "
                                   + st["message"])
            samples.append((time.monotonic() - start,
                            st["pos"][joint_idx],
                            st["target"][joint_idx],
                            st["err"][joint_idx]))
            time.sleep(period)
        return samples

    try:
        base = client.status()
        print(f"baseline pos_deg=" +
              ",".join(f"{v:.2f}" for v in base["pos"]))
        seq, msg = client.control(mask, args.ttl)
        print(f"CONTROL mask=0x{mask:04x}: {msg}")

        for jname in joints:
            jidx = JOINT_NAMES.index(jname)
            metrics_all = []
            for off in offsets:
                st = client.status()
                base_pos = st["target"][jidx]
                targets = list(st["target"])
                targets[jidx] = base_pos + off
                tseq, _ = client.target(targets, args.ttl, args.duration_ms)
                samples = sample_window(jidx, args.settle)
                for t_rel, p, tg, e in samples:
                    if writer:
                        writer.writerow([int((time.monotonic() - t0) * 1000),
                                         jname, off, "step", f"{t_rel:.3f}",
                                         f"{p:.4f}", f"{tg:.4f}", e, 0])
                m = analyze(samples, 1 if off > 0 else -1)
                if m:
                    metrics_all.append(m)
                    print(f"{jname} step {off:+.0f}deg: "
                          f"overshoot={m['overshoot_deg'] * STEPS_PER_DEG:.1f}st "
                          f"osc_pp={m['osc_pp_deg'] * STEPS_PER_DEG:.1f}st "
                          f"steady={m['steady_err_deg'] * STEPS_PER_DEG:+.1f}st "
                          f"settle={m['settling_s']:.2f}s "
                          f"reversals={m['tail_reversals']}")
                code, rmsg = client.wait_completion(tseq)
                if code not in (0, None):
                    print(f"  trajectory RESULT code={code} {rmsg}")
                elif code is None:
                    print(f"  warning: {rmsg}")
                # return to baseline
                st = client.status()
                targets = list(st["target"])
                targets[jidx] = base_pos
                rseq, _ = client.target(targets, args.ttl, args.duration_ms)
                back = sample_window(jidx, args.settle / 2)
                for t_rel, p, tg, e in back:
                    if writer:
                        writer.writerow([int((time.monotonic() - t0) * 1000),
                                         jname, off, "return", f"{t_rel:.3f}",
                                         f"{p:.4f}", f"{tg:.4f}", e, 0])
                code, rmsg = client.wait_completion(rseq)
                if code not in (0, None):
                    print(f"  return RESULT code={code} {rmsg}")
                elif code is None:
                    print(f"  warning: return {rmsg}")
                if time.monotonic() - last_hb > 3:
                    client.heartbeat(args.ttl)
                    last_hb = time.monotonic()
            if metrics_all:
                (p, d, i), note = recommend(metrics_all, jname)
                results[jname] = (p, d, i, note)
                print(f"== {jname}: current P{CURRENT_PID[jname][0]} "
                      f"D{CURRENT_PID[jname][1]} "
                      f"I{CURRENT_PID[jname][2]} -> recommend "
                      f"P{p} D{d} I{i} ({note})")
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
    for jname, (p, d, i, note) in results.items():
        cur = CURRENT_PID[jname]
        print(f"{jname}: P{cur[0]}/D{cur[1]}/I{cur[2]} -> "
              f"P{p}/D{d}/I{i}  [{note}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
