#!/usr/bin/env python3
"""Sample cached binary STATUS frames and report per-joint hold jitter."""

import argparse
import socket
import struct
import time

from send_dual_arm_udp import KINDS, frame, parse_frame


def decode_state(payload):
    request, mode, fault = struct.unpack_from(">QBB", payload, 0)
    active_mask = struct.unpack_from(">H", payload, 12)[0]
    offset = 16 + 5 * 8 + 2 * 4
    reason_len = struct.unpack_from(">H", payload, offset)[0]
    offset += 2 + reason_len
    message_len = struct.unpack_from(">H", payload, offset)[0]
    offset += 2 + message_len
    positions = []
    targets = []
    errors = []
    for index in range(14):
        position, target, error = struct.unpack_from(">ddi", payload,
                                                     offset + index * 20)
        positions.append(position)
        targets.append(target)
        errors.append(error)
    return request, mode, fault, active_mask, positions, targets, errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--joint", required=True,
                        choices=[f"LJ{i}" for i in range(1, 8)] +
                                [f"RJ{i}" for i in range(1, 8)])
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    index = int(args.joint[2:]) - 1 + (0 if args.joint[0] == "L" else 7)
    positions = []
    errors = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.5)
        deadline = time.monotonic() + args.seconds
        request = int(time.monotonic() * 1000) & 0x7FFFFFFF
        next_sample = time.monotonic()
        while time.monotonic() < deadline:
            request += 1
            sock.sendto(frame(KINDS["STATUS"], request), (args.host, args.port))
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            parsed = parse_frame(data)
            if parsed is None or parsed[0] != KINDS["STATE"]:
                continue
            _, mode, fault, active_mask, pos, _, err = decode_state(parsed[2])
            positions.append(pos[index])
            errors.append(err[index])
            next_sample += 1.0 / args.hz
            time.sleep(max(0.0, next_sample - time.monotonic()))

    if not positions:
        raise SystemExit("no STATE samples received")
    span_deg = max(positions) - min(positions)
    print(f"joint={args.joint} samples={len(positions)} mode={mode} "
          f"fault={fault} active_mask=0x{active_mask:04x} "
          f"min_deg={min(positions):.6f} max_deg={max(positions):.6f} "
          f"jitter_span_deg={span_deg:.6f} "
          f"error_steps_min={min(errors)} error_steps_max={max(errors)}")


if __name__ == "__main__":
    main()
