#!/usr/bin/env python3
"""Safely test one joint while its configured control scope remains held.

The program captures the other 13 targets from the initial STATUS snapshot,
enables LEFT (or optionally BOTH) with targets=current, moves the selected joint to logical
zero before every test point, samples hold jitter, returns it to zero, and
always sends STOP before exiting.
"""

import argparse
import socket
import struct
import time

from sample_udp_joint_jitter import decode_state
from send_dual_arm_udp import KINDS, frame, parse_frame, put_f64, put_u16, put_u32


def fresh_sequence():
    """Seed commands above sequences issued by earlier client processes."""
    return time.time_ns() // 1000


def command_frame(kind, sequence, ttl_ms=0, duration_ms=0, targets=None,
                  control_mask=0x007F):
    payload = bytearray()
    if kind == KINDS["CONTROL"]:
        put_u16(payload, control_mask)
        put_u32(payload, ttl_ms)
    elif kind == KINDS["TARGET"]:
        put_u32(payload, ttl_ms)
        put_u32(payload, duration_ms)
        for target in targets:
            put_f64(payload, target)
    return frame(kind, sequence, bytes(payload))


def receive_state(sock, robot, request):
    sock.sendto(frame(KINDS["STATUS"], request), robot)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        parsed = parse_frame(data)
        if parsed and parsed[0] == KINDS["STATE"]:
            return decode_state(parsed[2])
    raise RuntimeError("STATUS timed out")


def execute(sock, robot, datagram, sequence, timeout):
    sock.sendto(datagram, robot)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        parsed = parse_frame(data)
        if not parsed:
            continue
        kind, _, payload = parsed
        if kind == KINDS["ERROR"]:
            code = struct.unpack_from(">H", payload, 8)[0]
            raise RuntimeError(f"sequence {sequence} ERROR code={code}")
        if kind != KINDS["RESULT"]:
            continue
        result_sequence, code = struct.unpack_from(">QI", payload, 0)
        if result_sequence != sequence:
            continue
        message_len = struct.unpack_from(">H", payload, 20)[0]
        message = payload[22:22 + message_len].decode(errors="replace")
        print(f"RESULT sequence={sequence} code={code} message={message}")
        if code != 0:
            raise RuntimeError(f"sequence {sequence} failed: {message}")
        return
    raise RuntimeError(f"sequence {sequence} timed out")


def sample(sock, robot, joint_index, seconds, hz, request,
           expected_active_mask):
    positions, errors = [], []
    deadline = time.monotonic() + seconds
    next_sample = time.monotonic()
    mode = fault = active_mask = 0
    while time.monotonic() < deadline:
        request += 1
        _, mode, fault, active_mask, pos, _, err = receive_state(
            sock, robot, request)
        positions.append(pos[joint_index])
        errors.append(err[joint_index])
        next_sample += 1.0 / hz
        time.sleep(max(0.0, next_sample - time.monotonic()))
    print(f"samples={len(positions)} mode={mode} fault={fault} "
          f"active_mask=0x{active_mask:04x} "
          f"min_deg={min(positions):.6f} max_deg={max(positions):.6f} "
          f"jitter_span_deg={max(positions) - min(positions):.6f} "
          f"error_steps_min={min(errors)} error_steps_max={max(errors)}")
    if mode != 2 or fault or active_mask != expected_active_mask:
        raise RuntimeError("controller left the requested healthy control scope "
                           "during sampling")
    return request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--joint", required=True,
                        choices=[f"LJ{i}" for i in range(1, 8)] +
                                [f"RJ{i}" for i in range(1, 8)])
    parser.add_argument("--positions", type=float, nargs="+", required=True)
    parser.add_argument("--scope", choices=("LEFT", "BOTH"), default="LEFT",
                        help="joints held during the test (default: LEFT)")
    parser.add_argument("--duration-ms", type=int, default=10000)
    parser.add_argument("--ttl-ms", type=int, default=120000)
    parser.add_argument("--sample-seconds", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    robot = (args.host, args.port)
    joint_index = int(args.joint[2:]) - 1 + (0 if args.joint[0] == "L" else 7)
    control_mask = 0x007F if args.scope == "LEFT" else 0x3FFF
    if args.scope == "LEFT" and joint_index >= 7:
        parser.error("--scope LEFT cannot test a right-arm joint")
    sequence = fresh_sequence()
    request = sequence + 100000
    stop_needed = False

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.5)
        state = receive_state(sock, robot, request)
        _, mode, fault, active_mask, positions, _, _ = state
        if mode != 1 or fault or active_mask != 0:
            raise SystemExit("start rejected: controller must be healthy OBSERVE")
        baseline = list(positions)
        print("baseline=" + ",".join(f"{value:.3f}" for value in baseline))
        try:
            execute(sock, robot,
                    command_frame(KINDS["CONTROL"], sequence, args.ttl_ms,
                                  control_mask=control_mask),
                    sequence, 20.0)
            stop_needed = True
            for test_position in args.positions:
                zero_targets = list(baseline)
                zero_targets[joint_index] = 0.0
                sequence += 1
                execute(sock, robot,
                        command_frame(KINDS["TARGET"], sequence, args.ttl_ms,
                                      args.duration_ms, zero_targets),
                        sequence, args.duration_ms / 1000.0 + 15.0)

                test_targets = list(baseline)
                test_targets[joint_index] = test_position
                sequence += 1
                print(f"TEST {args.joint} target={test_position:.3f} deg")
                execute(sock, robot,
                        command_frame(KINDS["TARGET"], sequence, args.ttl_ms,
                                      args.duration_ms, test_targets),
                        sequence, args.duration_ms / 1000.0 + 15.0)
                request = sample(sock, robot, joint_index,
                                 args.sample_seconds, args.hz, request,
                                 control_mask)

                sequence += 1
                execute(sock, robot,
                        command_frame(KINDS["TARGET"], sequence, args.ttl_ms,
                                      args.duration_ms, zero_targets),
                        sequence, args.duration_ms / 1000.0 + 15.0)
                print(f"RETURNED {args.joint} to logical zero")
        finally:
            if stop_needed:
                sequence += 1
                try:
                    execute(sock, robot, frame(KINDS["STOP"], sequence),
                            sequence, 10.0)
                    print("STOP confirmed: all torque OFF")
                except Exception as error:
                    print(f"CRITICAL: STOP was not confirmed: {error}")


if __name__ == "__main__":
    main()
