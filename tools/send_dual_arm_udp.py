#!/usr/bin/env python3
"""Send one NOETIX binary frame and print its decoded UDP replies.

Command syntax on the command line mirrors the old text protocol:
  PING 1 | STATUS 2 | MODE 3 STATUS 500 | MODE 4 CONTROL |
  CONTROL 5 30000 LEFT | TARGET 6 6000 1000 <14 target degrees> |
  STREAM_TARGET 7 3000 50 <14 targets> | HOLD 8 1000 |
  HEARTBEAT 9 1000 | STOP 10 | RESET 11 30000
Binary frame: "NOET" magic, kind u8, 3 reserved, seq u64 BE, payload_len
u32 BE, payload, checksum u32 BE (sum of all previous bytes).
"""

import argparse
import socket
import struct
import sys
import time

KINDS = {
    "PING": 0, "STATUS": 1, "MODE": 2, "CONTROL": 3, "TARGET": 4,
    "STREAM_TARGET": 5, "HOLD": 6, "HEARTBEAT": 7, "STOP": 8, "RESET": 9,
    "ACK": 10, "RESULT": 11, "ERROR": 12, "PONG": 13, "STATE": 14,
}
KIND_NAMES = {value: key for key, value in KINDS.items()}


def put_u8(buf, value):
    buf.append(value & 0xFF)


def put_u16(buf, value):
    buf.append((value >> 8) & 0xFF)
    buf.append(value & 0xFF)


def put_u32(buf, value):
    for shift in (24, 16, 8, 0):
        buf.append((value >> shift) & 0xFF)


def put_u64(buf, value):
    for shift in (56, 48, 40, 32, 24, 16, 8, 0):
        buf.append((value >> shift) & 0xFF)


def put_f64(buf, value):
    buf.extend(struct.pack(">d", value))


def frame(kind, sequence, payload=b""):
    buf = bytearray()
    put_u32(buf, 0x4E4F4554)
    put_u8(buf, kind)
    buf += b"\0\0\0"
    put_u64(buf, sequence)
    put_u32(buf, len(payload))
    buf += payload
    put_u32(buf, sum(buf) & 0xFFFFFFFF)
    return bytes(buf)


def parse_frame(data):
    if len(data) < 24 or struct.unpack_from(">I", data, 0)[0] != 0x4E4F4554:
        return None
    kind = data[4]
    sequence, payload_len = struct.unpack_from(">QI", data, 8)
    if 20 + payload_len + 4 != len(data):
        return None
    payload = data[20:20 + payload_len]
    return kind, sequence, payload


def decode_payload(kind, payload):
    text = KIND_NAMES.get(kind, f"kind={kind}")
    if kind == 10:
        seq, accepted, deadline, mlen = struct.unpack_from(">QQQH", payload, 0)
        return (f"ACK seq={seq} accepted_ms={accepted} deadline_ms={deadline} "
                f"message={payload[26:26 + mlen].decode(errors='replace')}")
    if kind == 11:
        seq, code, finished, mlen = struct.unpack_from(">QIQH", payload, 0)
        return (f"RESULT seq={seq} code={code} finished_ms={finished} "
                f"message={payload[22:22 + mlen].decode(errors='replace')}")
    if kind == 12:
        seq, code, mlen = struct.unpack_from(">QHH", payload, 0)
        return (f"ERROR seq={seq} code={code} "
                f"message={payload[12:12 + mlen].decode(errors='replace')}")
    if kind == 13:
        req, now = struct.unpack_from(">QQ", payload, 0)
        return f"PONG request={req} monotonic_ms={now}"
    if kind == 14:
        req, mode, fault, ppol, tpol = struct.unpack_from(">QBBBB", payload, 0)
        active, qdepth = struct.unpack_from(">HH", payload, 12)
        mono, lease, submitted, active_seq, result_seq, result_code, span = \
            struct.unpack_from(">qqqqqII", payload, 16)
        offset = 16 + 5 * 8 + 2 * 4
        reason_len = struct.unpack_from(">H", payload, offset)[0]
        offset += 2
        reason = payload[offset:offset + reason_len].decode(errors="replace")
        offset += reason_len
        msg_len = struct.unpack_from(">H", payload, offset)[0]
        offset += 2
        message = payload[offset:offset + msg_len].decode(errors="replace")
        offset += msg_len
        pos, target, err = [], [], []
        for idx in range(14):
            p, t, e = struct.unpack_from(">ddi", payload, offset + idx * 20)
            pos.append(p)
            target.append(t)
            err.append(e)
        return (f"STATE request={req} mode={mode} fault={fault} "
                f"active_mask=0x{active:04x} queue_depth={qdepth} "
                f"lease_ms={lease} snapshot_ms={mono} "
                f"submitted={submitted} active={active_seq} result={result_seq} "
                f"result_code={result_code} span_ms={span} "
                f"fault_reason={reason or 'none'} result_message={message or 'none'} "
                f"pos_deg=" + ",".join(f"{v:.3f}" for v in pos) +
                " target_deg=" + ",".join(f"{v:.3f}" for v in target) +
                " err_steps=" + ",".join(str(v) for v in err))
    return f"{text} payload={payload.hex()}"


def build(pieces):
    verb = pieces[0].upper()
    kind = KINDS.get(verb)
    if kind is None or kind > 9:
        raise ValueError(f"unknown request verb {pieces[0]}")
    if verb in ("PING", "STATUS", "STOP"):
        if len(pieces) != 2:
            raise ValueError(f"{verb} takes only a request id")
        seq = int(pieces[1], 0)
        return frame(kind, seq) if seq else None
    if verb in ("HOLD", "HEARTBEAT", "RESET"):
        if len(pieces) != 3:
            raise ValueError(f"{verb} takes request-id and ttl_ms")
        seq = int(pieces[1], 0)
        ttl = int(pieces[2])
        buf = bytearray()
        put_u32(buf, ttl)
        return frame(kind, seq, bytes(buf))
    if verb == "MODE":
        if len(pieces) not in (3, 4):
            raise ValueError("MODE takes request-id STATUS [hz] | CONTROL")
        req = int(pieces[1], 0)
        if pieces[2].upper() == "STATUS":
            if len(pieces) != 4:
                raise ValueError("MODE STATUS requires a stream hz")
            layer = 1
            hz = int(pieces[3], 0)
        else:
            layer = 0
            hz = 0
        if not 0 <= hz <= 1000:
            raise ValueError("hz must be 0..1000")
        buf = bytearray()
        put_u8(buf, layer)
        put_u32(buf, hz)
        return frame(kind, req, bytes(buf))
    if verb == "CONTROL":
        if len(pieces) != 4:
            raise ValueError("CONTROL takes request-id ttl_ms LEFT|RIGHT|BOTH|LJ1..RJ7")
        req = int(pieces[1], 0)
        ttl = int(pieces[2])
        scope = pieces[3].upper()
        mask = 0x7F if scope == "LEFT" else (0x3F80 if scope == "RIGHT"
                                             else (0x3FFF if scope == "BOTH" else 0))
        if mask == 0:
            index = int(scope[2:]) - 1
            mask = 1 << (index if scope[0] == "L" else index + 7)
        buf = bytearray()
        put_u16(buf, mask)
        put_u32(buf, ttl)
        return frame(kind, req, bytes(buf))
    if verb in ("TARGET", "STREAM_TARGET"):
        if len(pieces) != 18:
            raise ValueError(f"{verb} takes request-id ttl_ms duration_ms + 14 targets")
        req = int(pieces[1], 0)
        ttl = int(pieces[2])
        duration = int(pieces[3])
        targets = [float(piece) for piece in pieces[4:18]]
        if len(targets) != 14:
            raise ValueError("must provide exactly 14 target degrees")
        buf = bytearray()
        put_u32(buf, ttl)
        put_u32(buf, duration)
        for value in targets:
            put_f64(buf, value)
        return frame(kind, req, bytes(buf))
    raise ValueError(f"unsupported verb {verb}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="robot IPv4 address")
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not args.command:
        parser.error("a protocol command is required, e.g. PING 1")
    pieces = " ".join(args.command).split()
    try:
        datagram = build(pieces)
    except ValueError as error:
        parser.error(str(error))
    if datagram is None:
        parser.error("request id must be non-zero")

    query = args.command[0].upper() in {"PING", "STATUS", "MODE"}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(min(0.5, args.timeout))
        sock.sendto(datagram, (args.host, args.port))
        deadline = time.monotonic() + args.timeout
        got_ack = False
        while time.monotonic() < deadline:
            try:
                reply, peer = sock.recvfrom(4096)
            except socket.timeout:
                continue
            parsed = parse_frame(reply)
            if parsed is None:
                print(f"{peer[0]}:{peer[1]} <unparseable {len(reply)} bytes>")
                continue
            kind, sequence, payload = parsed
            print(f"{peer[0]}:{peer[1]} {decode_payload(kind, payload)}")
            if query:
                return 0
            got_ack = got_ack or kind == 10
            if kind == 12:
                return 2
            if kind == 11:
                result_code = struct.unpack_from(">I", payload, 8)[0]
                return 0 if result_code == 0 else 2
        print("timeout waiting for " + ("RESULT" if got_ack else "reply"),
              file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
