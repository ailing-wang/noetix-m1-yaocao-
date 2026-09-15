#!/usr/bin/env bash
# Restart and verify the single dual-arm UDP controller on the robot.

set -euo pipefail

ROBOT_HOST="${1:-192.168.127.40}"
ROBOT_USER="${ROBOT_USER:-noetix}"
ROBOT_SERVICE="${ROBOT_SERVICE:-startup.service}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "${ROBOT_USER}@${ROBOT_HOST}")

echo "[1/3] Restarting ${ROBOT_SERVICE} on ${ROBOT_USER}@${ROBOT_HOST} ..."
"${SSH[@]}" "sudo -n systemctl restart '${ROBOT_SERVICE}'"

echo "[2/3] Waiting for systemd and UDP port 8890 ..."
ready=0
for _ in $(seq 1 20); do
    if "${SSH[@]}" \
        "systemctl is-active --quiet '${ROBOT_SERVICE}' && ss -lun | grep -qE ':8890[[:space:]]'"
    then
        ready=1
        break
    fi
    sleep 0.5
done

if (( ready == 0 )); then
    echo "ERROR: service or UDP 8890 did not become ready." >&2
    "${SSH[@]}" \
        "systemctl status '${ROBOT_SERVICE}' --no-pager -l; sudo -n journalctl -u '${ROBOT_SERVICE}' -n 80 --no-pager" \
        || true
    exit 1
fi

"${SSH[@]}" \
    "systemctl show '${ROBOT_SERVICE}' -p ActiveState -p SubState -p MainPID -p NRestarts"

echo "[3/3] Checking protocol state ..."
python3 "${SCRIPT_DIR}/send_dual_arm_udp.py" \
    --host "${ROBOT_HOST}" --timeout 5 PING 9901

status="$({
    python3 "${SCRIPT_DIR}/send_dual_arm_udp.py" \
        --host "${ROBOT_HOST}" --timeout 5 STATUS 9902
} 2>&1)"
printf '%s\n' "${status}"

if [[ "${status}" != *"fault=0"* ]]; then
    echo "ERROR: controller is running but reports a latched fault." >&2
    "${SSH[@]}" \
        "sudo -n journalctl -u '${ROBOT_SERVICE}' -n 80 --no-pager" || true
    exit 1
fi

if [[ "${status}" != *"active_mask=0x0000"* ]]; then
    echo "ERROR: controller did not restart in safe OBSERVE state." >&2
    exit 1
fi

echo "OK: ${ROBOT_SERVICE} is healthy; controller is in torque-off OBSERVE."
