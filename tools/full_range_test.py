#!/usr/bin/env python3
"""LJ1/LJ2/LJ4 full-range test using one absolute target per endpoint.

The six non-tested left joints retain the fixed targets captured immediately
after CONTROL.  Feedback is observed only; it is never copied back into their
command targets.
"""
import math
import argparse
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, "/home/jiayuanwang/noetix-m1-yaocao/tools")
from probe_common import Link, wait_observe, wait_stable, NOET_HDR, NOET_PAY, MAGIC

LIMITS = {0: (-43.6, 91.4), 1: (-84.7, 23.2), 3: (-87.0, 90.6)}
JOINTS = [0, 1, 3]
ENDPOINT_MARGIN_DEG = 1.0
POSITION_TOLERANCE_DEG = 1.5


class StreamRec:
    def __init__(self, joint):
        self.joint = joint
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("192.168.127.50", 8888))
        self.sock.settimeout(0.05)
        self.vals = []
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                pkt, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(pkt) != NOET_HDR.size + NOET_PAY.size:
                continue
            if NOET_HDR.unpack_from(pkt, 0)[0] != MAGIC:
                continue
            p = NOET_PAY.unpack(pkt[NOET_HDR.size:])
            self.vals.append((time.monotonic(), math.degrees(p[2 + self.joint])))

    def stop(self):
        self._stop.set()
        time.sleep(0.2)
        try:
            self.sock.close()
        except OSError:
            pass


def go(link, fixed_targets, joint, target):
    """Send the endpoint once; device speed/acceleration shape the motion."""
    command = list(fixed_targets)
    current = link.status()
    if current is None:
        return float("nan"), False
    distance = abs(target - current[3][joint])
    # DEVICE_PROFILED uses duration as its arrival budget, not as host-side
    # interpolation.  Five degrees/second plus allowance is deliberately
    # conservative for a full-range bench run.
    duration_ms = max(5000, min(60000, int(distance / 5.0 * 1000.0) + 5000))
    command[joint] = target
    code = link.target(command, duration_ms=duration_ms,
                       ttl_ms=120000, timeout_s=125.0)
    state = link.status()
    if state is None:
        return float("nan"), False
    position = state[3][joint]
    return position, code == 0 and abs(position - target) <= POSITION_TOLERANCE_DEG


def seg_stats(rec, t_a, t_b):
    import numpy as np
    arr = np.array([[v[0] - t_a, v[1]] for v in rec.vals if t_a <= v[0] <= t_b])
    if len(arr) < 20:
        return None
    p = arr[:, 1]
    d = np.abs(np.diff(p))
    big = int((d > 0.3).sum())
    pf = p.copy()
    for i in range(2, len(pf) - 2):
        pf[i] = np.median(p[i - 2:i + 3])
    rev = 0
    for i in range(2, len(pf)):
        if (pf[i - 1] - pf[i - 2]) * (pf[i] - pf[i - 1]) < 0 and abs(pf[i] - pf[i - 1]) > 0.3:
            rev += 1
    return dict(big=big, rev=rev)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--joints", default="LJ1,LJ2,LJ4",
        help="comma-separated subset/order from LJ1,LJ2,LJ4")
    args = parser.parse_args()
    name_to_index = {"LJ1": 0, "LJ2": 1, "LJ4": 3}
    try:
        joints = [name_to_index[name.strip().upper()]
                  for name in args.joints.split(",") if name.strip()]
    except KeyError as exc:
        parser.error(f"unsupported joint {exc.args[0]}; use LJ1,LJ2,LJ4")

    link = Link()
    continue_testing = True
    for j in joints:
        if not continue_testing:
            break
        rec = StreamRec(j)
        rec.start()
        lim = LIMITS[j]
        upper = lim[1] - ENDPOINT_MARGIN_DEG
        lower = lim[0] + ENDPOINT_MARGIN_DEG
        print(f"=== LJ{j+1}: 软上限内1°({upper:+.1f}) -> "
              f"软下限内1°({lower:+.1f}) -> 回零 ===", flush=True)
        if not wait_observe(link):
            print("  非Observe")
            rec.stop()
            continue
        time.sleep(1.0)
        wait_stable(link)
        code = link.control()
        if code != 0:
            link.reset()
            time.sleep(1.5)
            wait_observe(link)
            wait_stable(link)
            code = link.control()
            if code != 0:
                print("  CONTROL 失败")
                rec.stop()
                continue
        time.sleep(0.3)
        st = link.status()
        fixed_targets = list(st[3])
        time.sleep(2.0)
        rec.vals.clear()
        returned_to_zero = False
        for label, target in (("软上限内1°", upper),
                              ("软下限内1°", lower),
                              ("回零", 0.0)):
            t_a = time.monotonic()
            pos, ok = go(link, fixed_targets, j, target)
            dt = time.monotonic() - t_a
            sts = seg_stats(rec, t_a, time.monotonic())
            s = f"跳变{sts['big']}" if sts else "-"
            v = f"往复{sts['rev']}" if sts else "-"
            print(f"  {label}({target:+.1f}): 到达 {pos:+8.3f} "
                  f"{'OK' if ok else '!!超时'} 用时{dt:5.1f}s [{s} {v}]", flush=True)
            if not ok:
                continue_testing = False
                break
            fixed_targets[j] = target
            if label == "回零":
                returned_to_zero = True
            time.sleep(1.0)
        link.stop()
        time.sleep(0.3)
        rec.stop()
        time.sleep(2.0)
        if not returned_to_zero:
            print(f"  LJ{j+1} 未确认回零；终止后续关节测试。", flush=True)
            continue_testing = False
    link.stop()
    st = link.status()
    if st:
        print(f"\n收尾: mode={st[0]} fault={st[1]} LJ1-7={[f'{x:+.1f}' for x in st[3][:7]]}")


if __name__ == "__main__":
    main()
