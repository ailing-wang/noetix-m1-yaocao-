#!/usr/bin/env python3
"""Continuously sweep one of LJ1/LJ2/LJ4 across its calibrated range."""

import argparse
import socket
import time

from send_dual_arm_udp import KINDS, frame
from test_udp_joint_positions import (command_frame, execute, fresh_sequence,
                                      receive_state, sample)


RANGES = {
    "LJ1": (0, 90.4, -42.6),
    "LJ2": (1, 22.2, -83.6),
    "LJ4": (3, 89.6, -86.0),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--joint", choices=RANGES, required=True)
    parser.add_argument("--duration-ms", type=int, default=45000)
    parser.add_argument("--ttl-ms", type=int, default=120000)
    parser.add_argument("--sample-seconds", type=float, default=3.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    index, positive, negative = RANGES[args.joint]
    robot = (args.host, args.port)
    sequence = fresh_sequence()
    request = sequence + 200000
    stop_needed = False

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.5)
        state = receive_state(sock, robot, request)
        _, mode, fault, active_mask, positions, _, _ = state
        if mode != 1 or fault or active_mask != 0:
            raise SystemExit("start rejected: controller must be healthy OBSERVE")

        baseline = list(positions)
        initial = list(baseline)
        initial[index] = 0.0
        print(f"{args.joint} CONTINUOUS PATH 0 -> {positive:.3f} -> "
              f"{negative:.3f} -> 0", flush=True)
        print("LEFT baseline=" + ",".join(f"{v:.3f}" for v in baseline[:7]),
              flush=True)
        try:
            execute(sock, robot,
                    command_frame(KINDS["CONTROL"], sequence, args.ttl_ms,
                                  control_mask=0x007F),
                    sequence, 20.0)
            stop_needed = True

            for label, value in (("INITIAL", 0.0),
                                 ("POSITIVE", positive),
                                 ("NEGATIVE", negative),
                                 ("RETURN_INITIAL", 0.0)):
                targets = list(initial)
                targets[index] = value
                sequence += 1
                print(f"{label} target={value:.3f}", flush=True)
                execute(sock, robot,
                        command_frame(KINDS["TARGET"], sequence, args.ttl_ms,
                                      args.duration_ms, targets),
                        sequence, args.duration_ms / 1000.0 + 15.0)
                request = sample(sock, robot, index, args.sample_seconds,
                                 args.hz, request, 0x007F)
            print(f"{args.joint} FULL_RANGE_COMPLETED", flush=True)
        finally:
            if stop_needed:
                sequence += 1
                try:
                    execute(sock, robot, frame(KINDS["STOP"], sequence),
                            sequence, 10.0)
                    print("STOP confirmed: all torque OFF", flush=True)
                except Exception as error:
                    print(f"CRITICAL: STOP was not confirmed: {error}", flush=True)


if __name__ == "__main__":
    main()
