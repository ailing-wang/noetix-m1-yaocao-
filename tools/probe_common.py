#!/usr/bin/env python3
"""主臂 UDP 基础模块: Link/StreamLogger/安全流程。"""
import math
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, "/home/jiayuanwang/noetix-m1-yaocao/tools")
from send_dual_arm_udp import KINDS, frame, parse_frame
from sample_udp_joint_jitter import decode_state

ROBOT = ("192.168.127.40", 8890)
NOET_HDR = struct.Struct("!IIII")
NOET_PAY = struct.Struct("!QQ16d20i")
MAGIC = 0x4E4F4554
LIMITS_L = [(-43.6, 91.4), (-84.7, 23.2), (-100.0, 99.3), (-87.0, 90.6),
            (-111.6, 81.9), (-55.4, 52.1), (-98.3, 55.5)]


class StreamLogger:
    def __init__(self, joint=0):
        self.joint = joint
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("192.168.127.50", 8888))
        self.sock.settimeout(0.2)
        self.samples = []
        self._stop = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

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
            self.samples.append((time.monotonic(), math.degrees(p[2 + self.joint])))

    def stop(self):
        self._stop.set()
        time.sleep(0.2)
        try:
            self.sock.close()
        except OSError:
            pass


class Link:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.seq = int(time.time() * 1000) * 1000
        self.errors = []

    def send(self, kind, payload=b""):
        self.seq += 1
        self.sock.sendto(frame(kind, self.seq, payload), ROBOT)
        return self.seq

    def rd(self, kind, timeout=5.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                d, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            p = parse_frame(d)
            if p and p[0] == kind:
                return p
            if p and p[0] == 12:
                msg = p[2][12:12 + struct.unpack_from('>H', p[2], 10)[0]].decode(errors='replace')
                self.errors.append(msg)
        return None

    def status(self):
        self.send(KINDS["STATUS"])
        p = self.rd(14)
        if p:
            req, mode, fault, active, pos, tgt, err = decode_state(p[2])
            return mode, fault, active, pos, tgt
        return None

    def reset(self):
        self.send(KINDS["RESET"], struct.pack(">I", 120000))
        self.rd(11, 10)

    def stop(self):
        self.send(KINDS["STOP"])
        time.sleep(0.3)

    def control(self, mask=0x007F):
        self.send(KINDS["CONTROL"], struct.pack(">HI", mask, 120000))
        r = self.rd(11)
        return struct.unpack_from(">I", r[2], 8)[0] if r else -1

    def stream(self, targets, duration_ms=500, ttl_ms=3000):
        self.send(KINDS["STREAM_TARGET"],
                  struct.pack(">II", ttl_ms, duration_ms) + struct.pack(">14d", *targets))

    def stream_long(self, targets, ttl_ms=10000):
        self.send(KINDS["STREAM_TARGET"],
                  struct.pack(">II", ttl_ms, 500) + struct.pack(">14d", *targets))

    def target(self, targets, duration_ms=30000, ttl_ms=120000,
               timeout_s=125.0):
        """Submit one absolute target and wait for its terminal RESULT."""
        self.send(KINDS["TARGET"],
                  struct.pack(">II", ttl_ms, duration_ms) +
                  struct.pack(">14d", *targets))
        result = self.rd(KINDS["RESULT"], timeout_s)
        if result is None:
            return -1
        return struct.unpack_from(">I", result[2], 8)[0]


def wait_observe(link):
    for _ in range(20):
        st = link.status()
        if st is None:
            time.sleep(0.2)
            continue
        mode, fault, _, _, _ = st
        if mode == 1 and fault == 0:
            return True
        if fault:
            link.reset()
        else:
            link.stop()
        time.sleep(0.5)
    return False


def wait_stable(link, tol=0.3, samples=4, interval=0.2):
    prev = None
    stable = 0
    for _ in range(40):
        st = link.status()
        if st is None:
            time.sleep(interval)
            continue
        pos = st[3][0]
        if prev is None:
            prev, stable = pos, 1
        elif abs(pos - prev) <= tol:
            stable += 1
        else:
            stable = 1
        prev = pos
        if stable >= samples:
            return True
        time.sleep(interval)
    return False


if __name__ == "__main__":
    link = Link()
    st = link.status()
    if st:
        print(f"mode={st[0]} fault={st[1]} LJ1={st[3][0]:+.2f}")
    link.sock.close()
