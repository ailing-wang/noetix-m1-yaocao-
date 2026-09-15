#!/usr/bin/env bash
set -euo pipefail

cd /home/noetix/workspace/noetix-m1-yaocao

exec stdbuf -oL -eL ./build-servo-test/noetix-dual-arm-udp \
  --bench-confirmed \
  --joint-config dual_arm_joints.cfg \
  --bind-ip 0.0.0.0 \
  --port 8890 \
  --peer-ip 192.168.127.50 \
  --stream-port 8888 \
  --stream-ms 0 \
  --monitor-ms 100 \
  --observe-ms 250 \
  --status-ms 4
