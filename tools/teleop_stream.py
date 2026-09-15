#!/usr/bin/env python3
"""Fixed-rate teleop stream client for the NOETIX_ARM_V1 dual-arm service.

Sends STREAM_TARGET frames at a constant frequency without waiting for
replies (ACKs are consumed only for diagnostics).  Remote state is read
back from the legacy NOET binary stream (port 8888 by default) on the
same machine.  Every frame is rate-limited to 9.5 deg/s against the last
accepted command, so encoder quantization cannot make idle targets drift and
the per-frame 10 deg/s protocol limit is never violated.  If no frame is
accepted for longer than the TTL, the service control lease expires into
FAULT; the client can resume by CONTROL.

Modes:
  hold   capture one pose at startup and keep that fixed target
  sin    J=joint  A=amplitude F=frequency(Hz): J sweeps around center
  ramp   J=joint  A=amplitude S=sec: J ramps 0 -> +A -> -A -> 0 once
  out-back J=joint A=amplitude S=sec: J ramps 0 -> +A -> 0 once

Usage:
  python3 teleop_stream.py --fps 20 --mode sin --joint LJ1 --amp 5 --freq 0.2
  python3 teleop_stream.py --fps 20 --mode hold

Exit: Ctrl+C (sends STOP, torque off).
"""

import argparse
import math
import select
import signal
import socket
import struct
import threading
import time

from send_dual_arm_udp import KINDS, parse_frame
from sample_udp_joint_jitter import decode_state

CMD_PORT = 8890
CMD_HOST = "192.168.127.40"
STREAM_PORT = 8888
KEEPALIVE_TTL_MS = 3000
FRAME_DURATION_MS = 50
MAX_SPEED_DEG_PER_SEC = 9.5
JOINT_NAMES = [f"LJ{i}" for i in range(1, 8)] + [f"RJ{i}" for i in range(1, 8)]
NAME_TO_INDEX = {name: index for index, name in enumerate(JOINT_NAMES)}

NOETIX_HEADER_STRUCT = struct.Struct("!IIII")
NOETIX_PAYLOAD_STRUCT = struct.Struct("!QQ16d20i")
NOETIX_TOTAL_SIZE = NOETIX_HEADER_STRUCT.size + NOETIX_PAYLOAD_STRUCT.size


class StreamReader:
    def __init__(self, port: int) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.setblocking(False)
        self._latest: list[float] | None = None
        self._monotonic: float | None = None
        self._ready = threading.Event()

    def poll(self, timeout_sec: float) -> None:
        readable, _, _ = select.select([self._sock], [], [], timeout_sec)
        if not readable:
            return
        try:
            while True:
                packet, _ = self._sock.recvfrom(4096)
                positions = self._parse(packet)
                if positions is not None:
                    self._latest = positions
                    self._monotonic = time.monotonic()
                    self._ready.set()
        except BlockingIOError:
            pass

    @staticmethod
    def _parse(packet: bytes) -> list[float] | None:
        if len(packet) != NOETIX_TOTAL_SIZE:
            return None
        header = NOETIX_HEADER_STRUCT.unpack_from(packet, 0)
        if header[0] != 0x4E4F4554 or header[2] != NOETIX_PAYLOAD_STRUCT.size:
            return None
        payload = NOETIX_PAYLOAD_STRUCT.unpack(packet[NOETIX_HEADER_STRUCT.size:])
        if (sum(packet[NOETIX_HEADER_STRUCT.size:]) & 0xFFFFFFFF) != header[3]:
            return None
        # Since 2026-08-27 the service's 8888 legacy stream carries RADIANS
        # (original master-program compatible); this tool works in degrees.
        return [math.degrees(v)
                for v in list(payload[2:10])[:7] + list(payload[10:18])[:7]]

    @property
    def latest(self) -> list[float] | None:
        return self._latest

    @property
    def timestamp(self) -> float | None:
        return self._monotonic


def send(sock: socket.socket, datagram: bytes) -> None:
    sock.sendto(datagram, (CMD_HOST, CMD_PORT))


def put_u32(buf: bytearray, value: int) -> None:
    for shift in (24, 16, 8, 0):
        buf.append((value >> shift) & 0xFF)


def put_u64(buf: bytearray, value: int) -> None:
    for shift in (56, 48, 40, 32, 24, 16, 8, 0):
        buf.append((value >> shift) & 0xFF)


def frame(kind: int, sequence: int, payload: bytes = b"") -> bytes:
    buf = bytearray()
    put_u32(buf, 0x4E4F4554)
    buf.append(kind)
    buf += b"\0\0\0"
    put_u64(buf, sequence)
    put_u32(buf, len(payload))
    buf += payload
    put_u32(buf, sum(buf) & 0xFFFFFFFF)
    return bytes(buf)


def stream_target_frame(sequence: int, ttl_ms: int, duration_ms: int,
                        targets: list[float]) -> bytes:
    payload = bytearray()
    put_u32(payload, ttl_ms)
    put_u32(payload, duration_ms)
    for value in targets:
        payload.extend(struct.pack(">d", value))
    return frame(5, sequence, bytes(payload))


def stop_frame(sequence: int) -> bytes:
    return frame(8, sequence)


def status_frame(request_id: int) -> bytes:
    return frame(1, request_id)


def drain_replies(sock: socket.socket, counts: dict) -> None:
    readable, _, _ = select.select([sock], [], [], 0)
    if not readable:
        return
    try:
        while True:
            packet, _ = sock.recvfrom(4096)
            if len(packet) < 24 or struct.unpack_from(">I", packet, 0)[0] != 0x4E4F4554:
                continue
            kind = packet[4]
            sequence = struct.unpack_from(">Q", packet, 8)[0]
            error_message = ""
            if kind == 12:
                code, mlen = struct.unpack_from(">HH", packet, 28)
                error_message = packet[32:32 + mlen].decode(errors="replace")
            if kind == 10:
                counts["ack"] += 1
            elif kind == 12:
                counts["rejected"] += 1
                print(f"!! ERROR seq={sequence} code={code} {error_message}")
            elif kind == 11:
                counts["result"] += 1
    except BlockingIOError:
        pass


def main() -> None:
    global CMD_HOST
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", dest="host", default="192.168.127.40")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--mode", default="hold",
                        choices=["hold", "sin", "ramp", "out-back"])
    parser.add_argument("--joint", default="LJ1")
    parser.add_argument("--amp", type=float, default=3.0)
    parser.add_argument("--freq", type=float, default=0.2)
    parser.add_argument("--ramp-sec", type=float, default=6.0)
    parser.add_argument("--ttl-ms", type=int, default=KEEPALIVE_TTL_MS)
    parser.add_argument("--start-seq", type=int, default=0,
                        help="first sequence; 0 means probe the service "
                             "submitted counter and continue after it")
    parser.add_argument("--verbose-fps", type=float, default=1.0)
    args = parser.parse_args()
    CMD_HOST = args.host

    if args.joint not in NAME_TO_INDEX:
        parser.error(f"--joint must be one of {', '.join(JOINT_NAMES)}")
    joint_index = NAME_TO_INDEX[args.joint]
    interval = 1.0 / args.fps
    if interval * 1000.0 < FRAME_DURATION_MS:
        print(f"warning: {args.fps} fps exceeds per-frame 50 ms duration; "
              f"rate-limit check still uses 50 ms per frame")

    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd.setblocking(False)
    reader = StreamReader(STREAM_PORT)

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.settimeout(3.0)
    request_id = int(time.time() * 1000) % 1000000 + 1
    probe.sendto(status_frame(request_id), (CMD_HOST, CMD_PORT))
    last_submitted = 0
    status_targets: list[float] | None = None
    status_active_mask = 0
    try:
        reply = probe.recv(4096)
        if len(reply) >= 72 and struct.unpack_from(">I", reply, 0)[0] == 0x4E4F4554:
            last_submitted = struct.unpack_from(">Q", reply, 52)[0]
            parsed = parse_frame(reply)
            if parsed is not None and parsed[0] == KINDS["STATE"]:
                (_, _, _, status_active_mask, _, status_targets,
                 _) = decode_state(parsed[2])
    except (OSError, ValueError):
        pass
    probe.close()
    if args.start_seq and args.start_seq > last_submitted:
        sequence = args.start_seq
    else:
        sequence = last_submitted + 1
    start_sequence = sequence

    stop_requested = threading.Event()

    def request_stop(signum, frame):
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        reader.poll(2.0)
        if reader.latest is None:
            print("no state frames received; is the dual-arm service streaming?")
            return 1
    except OSError as exc:
        print(f"cannot bind stream port {STREAM_PORT}: {exc}")
        return 1

    counts = {"ack": 0, "rejected": 0, "result": 0}
    last_status = time.monotonic()
    last_applied: list[float] | None = None
    start_mono = time.monotonic()
    held_target: list[float] | None = None
    status_string = ""

    print(f"teleop stream: {args.host}:{CMD_PORT} fps={args.fps} "
          f"mode={args.mode} joint={args.joint} amp={args.amp} "
          f"freq={args.freq} ttl={args.ttl_ms}ms")
    print(f"initial pose: {JOINT_NAMES}")
    print("  " + " ".join(f"{value:8.3f}" for value in reader.latest))
    print("Ctrl+C stops (torque off). Frames are rate-limited to "
          f"{MAX_SPEED_DEG_PER_SEC} deg/s against the last accepted command.")

    try:
        while not stop_requested.is_set():
            frame_start = time.monotonic()
            reader.poll(max(0.0, interval * 0.9))

            measured = reader.latest
            if measured is None:
                continue

            if held_target is None:
                # CONTROL has already written target=current.  Anchor STREAM
                # to those target registers, not to a later encoder sample;
                # otherwise normal compliance between CONTROL and frame 1
                # looks like a motion command on every selected joint.
                if status_targets is not None:
                    held_target = list(status_targets)
                else:
                    held_target = list(measured)
                    print("warning: STATUS target unavailable; using one "
                          "measured-pose fallback")
                last_applied = list(held_target)

            requested = list(held_target)
            elapsed = time.monotonic() - start_mono
            if args.mode == "sin":
                requested[joint_index] = (
                    held_target[joint_index]
                    + args.amp * math.sin(2.0 * math.pi * args.freq * elapsed)
                )
            elif args.mode == "ramp":
                period = 4.0 * args.ramp_sec
                # One complete, continuous excursion followed by a fixed
                # target.  Do not wrap with modulo: the stationary tail is
                # required to exercise the server's MOVE->HOLD transition.
                phase = min(elapsed, period)
                if phase < args.ramp_sec:
                    offset = args.amp * phase / args.ramp_sec
                elif phase < 2.0 * args.ramp_sec:
                    offset = args.amp * (2.0 - phase / args.ramp_sec)
                elif phase < 3.0 * args.ramp_sec:
                    offset = -args.amp * (phase / args.ramp_sec - 2.0)
                else:
                    offset = -args.amp * (4.0 - phase / args.ramp_sec)
                requested[joint_index] = held_target[joint_index] + offset
            elif args.mode == "out-back":
                period = 2.0 * args.ramp_sec
                phase = min(elapsed, period)
                if phase < args.ramp_sec:
                    offset = args.amp * phase / args.ramp_sec
                else:
                    offset = args.amp * (2.0 - phase / args.ramp_sec)
                requested[joint_index] = held_target[joint_index] + offset

            max_delta = MAX_SPEED_DEG_PER_SEC * max(interval, FRAME_DURATION_MS / 1000.0)
            # Build the next command from the previous command, never from
            # encoder feedback.  Feedback remains available for monitoring,
            # but its quantization/noise cannot move an idle joint's target.
            assert last_applied is not None
            frame = list(last_applied)
            slew = 0.0
            for index in range(14):
                delta = requested[index] - last_applied[index]
                delta = max(-max_delta, min(max_delta, delta))
                frame[index] = last_applied[index] + delta
                slew = max(slew, abs(delta))

            last_applied = frame
            send(cmd, stream_target_frame(sequence, args.ttl_ms,
                                          FRAME_DURATION_MS, frame))
            sequence += 1

            drain_replies(cmd, counts)

            now = time.monotonic()
            if now - last_status >= 1.0 / args.verbose_fps:
                last_status = now
                if reader.timestamp is not None:
                    age_ms = (now - reader.timestamp) * 1000.0
                else:
                    age_ms = -1.0
                monitored = [i for i in range(14)
                             if status_active_mask & (1 << i)]
                if not monitored:
                    monitored = list(range(14))
                error_max = max(abs(frame[i] - measured[i])
                                for i in monitored)
                frames_sent = sequence - start_sequence
                status_string = (
                    f"seq={sequence-1} fps={frames_sent/(now-start_mono):.1f} "
                    f"ack={counts['ack']} rej={counts['rejected']} "
                    f"slew_max={slew:.3f}deg err_max={error_max:.3f}deg "
                    f"stream_age_ms={age_ms:.0f}"
                )
                print(status_string)

            work = time.monotonic() - frame_start
            if work < interval:
                time.sleep(interval - work)
    finally:
        try:
            send(cmd, stop_frame(sequence))
        except OSError:
            pass
        print(f"STOP sent; last status: {status_string}")


if __name__ == "__main__":
    main()
