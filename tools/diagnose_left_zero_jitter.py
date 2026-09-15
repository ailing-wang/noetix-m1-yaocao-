#!/usr/bin/env python3
"""Move the complete left arm to logical zero and measure hold jitter."""

import argparse
import socket
import time

from send_dual_arm_udp import KINDS, frame
from test_udp_joint_positions import (command_frame, execute, fresh_sequence,
                                      receive_state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--duration-ms", type=int, default=30000)
    parser.add_argument("--ttl-ms", type=int, default=120000)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    robot = (args.host, args.port)
    sequence = fresh_sequence()
    request = sequence + 300000
    stop_needed = False

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.5)
        state = receive_state(sock, robot, request)
        _, mode, fault, active_mask, positions, _, _ = state
        if mode != 1 or fault or active_mask != 0:
            raise SystemExit("start rejected: controller must be healthy OBSERVE")

        targets = list(positions)
        targets[:7] = [0.0] * 7
        try:
            execute(sock, robot,
                    command_frame(KINDS["CONTROL"], sequence, args.ttl_ms,
                                  control_mask=0x007F),
                    sequence, 20.0)
            stop_needed = True
            sequence += 1
            execute(sock, robot,
                    command_frame(KINDS["TARGET"], sequence, args.ttl_ms,
                                  args.duration_ms, targets),
                    sequence, args.duration_ms / 1000.0 + 15.0)

            samples = [[] for _ in range(7)]
            errors = [[] for _ in range(7)]
            deadline = time.monotonic() + args.seconds
            next_sample = time.monotonic()
            last_mode = last_fault = last_mask = 0
            while time.monotonic() < deadline:
                request += 1
                state = receive_state(sock, robot, request)
                _, last_mode, last_fault, last_mask, pos, _, err = state
                for index in range(7):
                    samples[index].append(pos[index])
                    errors[index].append(err[index])
                next_sample += 1.0 / args.hz
                time.sleep(max(0.0, next_sample - time.monotonic()))

            if last_mode != 2 or last_fault or last_mask != 0x007F:
                raise RuntimeError("controller left healthy LEFT control mode")
            print("joint samples min_deg max_deg span_deg err_min err_max")
            for index in range(7):
                values = samples[index]
                joint_errors = errors[index]
                print(f"LJ{index + 1} {len(values)} {min(values):.6f} "
                      f"{max(values):.6f} "
                      f"{max(values) - min(values):.6f} "
                      f"{min(joint_errors)} {max(joint_errors)}")
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
