#!/usr/bin/env python3
"""Move LJ1 to +45 deg, return to zero, then measure zero hold jitter."""

import statistics
import time

from probe_common import Link, wait_observe, wait_stable


def main() -> int:
    link = Link()
    try:
        if not wait_observe(link):
            print("FAIL: controller did not reach healthy OBSERVE")
            return 1
        if not wait_stable(link):
            print("FAIL: arm position was not stable before CONTROL")
            return 1
        if link.control(mask=0x007F) != 0:
            print("FAIL: CONTROL LEFT")
            return 1

        state = link.status()
        if state is None:
            print("FAIL: no state after CONTROL")
            return 1
        fixed = list(state[4])  # device target registers, never feedback
        print(f"START LJ1 position={state[3][0]:+.3f} target={fixed[0]:+.3f}")

        outward = list(fixed)
        outward[0] = 45.0
        if link.target(outward, duration_ms=20000, ttl_ms=60000,
                       timeout_s=65.0) != 0:
            print("FAIL: LJ1 +45 target did not complete")
            return 1
        state = link.status()
        if state is None:
            print("FAIL: no state at +45")
            return 1
        print(f"AT_45 LJ1 position={state[3][0]:+.3f} target={state[4][0]:+.3f}")

        returned = list(fixed)
        returned[0] = 0.0
        if link.target(returned, duration_ms=20000, ttl_ms=60000,
                       timeout_s=65.0) != 0:
            print("FAIL: LJ1 zero target did not complete")
            return 1

        positions = []
        errors = []
        deadline = time.monotonic() + 5.0
        next_sample = time.monotonic()
        while time.monotonic() < deadline:
            state = link.status()
            if state is not None:
                positions.append(state[3][0])
                errors.append(state[4][0] - state[3][0])
            next_sample += 0.05
            time.sleep(max(0.0, next_sample - time.monotonic()))

        if not positions:
            print("FAIL: no zero-hold samples")
            return 1
        print(f"ZERO_HOLD samples={len(positions)} "
              f"mean={statistics.fmean(positions):+.3f} "
              f"min={min(positions):+.3f} max={max(positions):+.3f} "
              f"span={max(positions)-min(positions):.3f}deg "
              f"max_abs_error={max(abs(value) for value in errors):.3f}deg")
        return 0
    finally:
        link.stop()
        link.sock.close()
        print("STOP sent; torque-off requested")


if __name__ == "__main__":
    raise SystemExit(main())
