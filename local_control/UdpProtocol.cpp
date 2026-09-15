#include "dual_arm_udp/Protocol.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace noetix::dual_arm_udp
{
namespace
{
void putU8(std::string &out, std::uint8_t value)
{
    out.push_back(static_cast<char>(value));
}

void putU16(std::string &out, std::uint16_t value)
{
    out.push_back(static_cast<char>((value >> 8) & 0xFF));
    out.push_back(static_cast<char>(value & 0xFF));
}

void putU32(std::string &out, std::uint32_t value)
{
    for (int shift = 24; shift >= 0; shift -= 8)
        out.push_back(static_cast<char>((value >> shift) & 0xFF));
}

void putU64(std::string &out, std::uint64_t value)
{
    for (int shift = 56; shift >= 0; shift -= 8)
        out.push_back(static_cast<char>((value >> shift) & 0xFF));
}

void putF64(std::string &out, double value)
{
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    putU64(out, bits);
}

void putI32(std::string &out, std::int32_t value)
{
    putU32(out, static_cast<std::uint32_t>(value));
}

void putString(std::string &out, std::string_view text)
{
    putU16(out, static_cast<std::uint16_t>(std::min<std::size_t>(text.size(), 65535)));
    out.append(text.substr(0, 65535));
}

bool getU8(std::string_view data, std::size_t &offset, std::uint8_t &value)
{
    if (offset + 1 > data.size())
        return false;
    value = static_cast<std::uint8_t>(data[offset]);
    offset += 1;
    return true;
}

bool getU16(std::string_view data, std::size_t &offset, std::uint16_t &value)
{
    if (offset + 2 > data.size())
        return false;
    value = static_cast<std::uint16_t>((static_cast<std::uint8_t>(data[offset]) << 8) |
                                       static_cast<std::uint8_t>(data[offset + 1]));
    offset += 2;
    return true;
}

bool getU32(std::string_view data, std::size_t &offset, std::uint32_t &value)
{
    if (offset + 4 > data.size())
        return false;
    value = 0;
    for (int index = 0; index < 4; ++index)
        value = (value << 8) | static_cast<std::uint8_t>(data[offset + index]);
    offset += 4;
    return true;
}

bool getU64(std::string_view data, std::size_t &offset, std::uint64_t &value)
{
    if (offset + 8 > data.size())
        return false;
    value = 0;
    for (int index = 0; index < 8; ++index)
        value = (value << 8) | static_cast<std::uint8_t>(data[offset + index]);
    offset += 8;
    return true;
}

bool getF64(std::string_view data, std::size_t &offset, double &value)
{
    std::uint64_t bits = 0;
    if (!getU64(data, offset, bits))
        return false;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&value, &bits, sizeof(value));
    return true;
}

bool getI32(std::string_view data, std::size_t &offset, std::int32_t &value)
{
    std::uint32_t bits = 0;
    if (!getU32(data, offset, bits))
        return false;
    value = static_cast<std::int32_t>(bits);
    return true;
}

bool getString(std::string_view data, std::size_t &offset, std::string &value)
{
    std::uint16_t length = 0;
    if (!getU16(data, offset, length))
        return false;
    if (offset + length > data.size())
        return false;
    value.assign(data.substr(offset, length));
    offset += length;
    return true;
}
} // namespace

std::uint32_t checksum(std::string_view bytes) noexcept
{
    std::uint32_t sum = 0;
    for (const unsigned char byte : bytes)
        sum += byte;
    return sum;
}

std::string formatErrorText(ErrorCode code) noexcept
{
    switch (code)
    {
    case ErrorCode::BadPacket:
        return "BAD_PACKET";
    case ErrorCode::Rejected:
        return "REJECTED";
    case ErrorCode::Busy:
        return "BUSY";
    case ErrorCode::TooLarge:
        return "TOO_LARGE";
    case ErrorCode::OutOfBandStop:
        return "OUT_OF_BAND_STOP";
    case ErrorCode::SequenceReuse:
        return "SEQUENCE_REUSE";
    case ErrorCode::None:
        break;
    }
    return "NONE";
}

const char *toString(FrameKind kind) noexcept
{
    switch (kind)
    {
    case FrameKind::Ping:
        return "PING";
    case FrameKind::Status:
        return "STATUS";
    case FrameKind::Mode:
        return "MODE";
    case FrameKind::Control:
        return "CONTROL";
    case FrameKind::Target:
        return "TARGET";
    case FrameKind::StreamTarget:
        return "STREAM_TARGET";
    case FrameKind::Hold:
        return "HOLD";
    case FrameKind::Heartbeat:
        return "HEARTBEAT";
    case FrameKind::Stop:
        return "STOP";
    case FrameKind::FadeStop:
        return "FADE_STOP";
    case FrameKind::Reset:
        return "RESET";
    case FrameKind::Acknowledge:
        return "ACK";
    case FrameKind::Result:
        return "RESULT";
    case FrameKind::Error:
        return "ERROR";
    case FrameKind::Pong:
        return "PONG";
    case FrameKind::State:
        return "STATE";
    }
    return "UNKNOWN";
}

namespace
{
std::string frame(std::uint8_t kind, std::uint64_t sequence,
                  const std::string &payload)
{
    std::string output;
    output.reserve(kFrameOverhead + payload.size());
    putU32(output, kMagic);
    putU8(output, kind);
    putU8(output, 0);
    putU8(output, 0);
    putU8(output, 0);
    putU64(output, sequence);
    putU32(output, static_cast<std::uint32_t>(payload.size()));
    output.append(payload);
    putU32(output, checksum(std::string_view(output)));
    return output;
}

bool decodePayload(Command &command, std::string_view datagram,
                   std::size_t offset, std::string &error)
{
    const std::size_t payload_end = datagram.size() - kFrameCheckSize;
    switch (command.kind)
    {
    case FrameKind::Ping:
    case FrameKind::Status:
    case FrameKind::Stop:
        if (offset != payload_end)
        {
            error = "kind " + std::string(toString(command.kind)) +
                    " takes no payload";
            return false;
        }
        return true;
    case FrameKind::Mode:
    {
        std::uint8_t layer = 0;
        std::uint32_t hz = 0;
        if (!getU8(datagram, offset, layer) ||
            !getU32(datagram, offset, hz) || offset != payload_end)
        {
            error = "MODE requires layer u8 and hz u32";
            return false;
        }
        if (layer > 1)
        {
            error = "MODE layer must be 0 (control) or 1 (status)";
            return false;
        }
        command.mode_layer =
            layer == 1 ? ModeLayer::Status : ModeLayer::Control;
        if (command.mode_layer == ModeLayer::Status)
        {
            if (hz == 0 || hz > 1000)
            {
                error = "MODE status hz must be 1..1000";
                return false;
            }
            command.stream_hz = hz;
        }
        return true;
    }
    case FrameKind::Control:
    {
        std::uint16_t selection = 0;
        if (!getU16(datagram, offset, selection) ||
            !getU32(datagram, offset, command.ttl_ms) ||
            offset != payload_end)
        {
            error = "CONTROL requires selection u16 and ttl_ms u32";
            return false;
        }
        command.selection = selection;
        return true;
    }
    case FrameKind::Target:
    case FrameKind::StreamTarget:
    {
        if (!getU32(datagram, offset, command.ttl_ms) ||
            !getU32(datagram, offset, command.duration_ms))
        {
            error = std::string(toString(command.kind)) +
                    " requires ttl_ms, duration_ms and 14 f64 targets";
            return false;
        }
        for (double &target : command.targets)
        {
            if (!getF64(datagram, offset, target))
            {
                error = std::string(toString(command.kind)) +
                        " requires ttl_ms, duration_ms and 14 f64 targets";
                return false;
            }
            if (!std::isfinite(target))
            {
                error = "all absolute targets must be finite";
                return false;
            }
        }
        if (offset != payload_end)
        {
            error = std::string(toString(command.kind)) +
                    " has extra bytes after the 14 positions";
            return false;
        }
        return true;
    }
    case FrameKind::Hold:
    case FrameKind::Heartbeat:
    case FrameKind::Reset:
        if (!getU32(datagram, offset, command.ttl_ms) ||
            offset != payload_end)
        {
            error = std::string(toString(command.kind)) +
                    " requires ttl_ms u32";
            return false;
        }
        return true;
    case FrameKind::FadeStop:
        if (!getU32(datagram, offset, command.duration_ms) ||
            offset != payload_end)
        {
            error = "FADE_STOP requires fade_ms u32";
            return false;
        }
        if (command.duration_ms < 100 || command.duration_ms > 10000)
        {
            error = "FADE_STOP fade_ms must be 100..10000";
            return false;
        }
        return true;
    default:
        error = "kind is not a request frame";
        return false;
    }
}
} // namespace

bool decodeFrame(std::string_view datagram, Command &command, std::string &error)
{
    if (datagram.size() < kFrameOverhead)
    {
        error = "datagram too short for NOET frame header";
        return false;
    }
    std::size_t offset = 0;
    std::uint32_t magic = 0;
    std::uint8_t kind = 0;
    std::uint8_t reserved = 0;
    std::uint32_t payload_size = 0;
    if (!getU32(datagram, offset, magic) || magic != kMagic)
    {
        error = "bad magic; expected NOET binary frame";
        return false;
    }
    if (!getU8(datagram, offset, kind))
        return false;
    if (!getU8(datagram, offset, reserved))
        return false;
    if (!getU8(datagram, offset, reserved))
        return false;
    if (!getU8(datagram, offset, reserved))
        return false;
    if (!getU64(datagram, offset, command.sequence))
        return false;
    if (!getU32(datagram, offset, payload_size))
        return false;

    const std::size_t expected = kFrameHeaderSize + payload_size + kFrameCheckSize;
    if (expected != datagram.size())
    {
        error = "payload size mismatch";
        return false;
    }
    const std::uint32_t expected_checksum =
        checksum(std::string_view(datagram).substr(0, kFrameHeaderSize + payload_size));
    std::uint32_t received_checksum = 0;
    offset = kFrameHeaderSize + payload_size;
    if (!getU32(datagram, offset, received_checksum))
        return false;
    if (received_checksum != expected_checksum)
    {
        error = "checksum mismatch";
        return false;
    }

    command.kind = static_cast<FrameKind>(kind);
    command.ttl_ms = 0;
    command.duration_ms = 0;
    command.selection = 0;
    command.targets = {};
    command.mode_layer = ModeLayer::Control;
    command.stream_hz = 0;
    offset = kFrameHeaderSize;
    if (!decodePayload(command, datagram, offset, error))
        return false;
    return true;
}

std::optional<std::string> encodeFrame(const Command &command)
{
    if (command.sequence == 0 || command.sequence > std::numeric_limits<std::int64_t>::max())
        return std::nullopt;
    std::string payload;
    switch (command.kind)
    {
    case FrameKind::Ping:
    case FrameKind::Status:
    case FrameKind::Stop:
        break;
    case FrameKind::Mode:
        putU8(payload, command.mode_layer == ModeLayer::Status ? 1 : 0);
        putU32(payload, command.stream_hz);
        break;
    case FrameKind::Control:
        putU16(payload, static_cast<std::uint16_t>(command.selection));
        putU32(payload, command.ttl_ms);
        break;
    case FrameKind::Target:
    case FrameKind::StreamTarget:
        putU32(payload, command.ttl_ms);
        putU32(payload, command.duration_ms);
        for (const double target : command.targets)
            putF64(payload, target);
        break;
    case FrameKind::Hold:
    case FrameKind::Heartbeat:
    case FrameKind::Reset:
        putU32(payload, command.ttl_ms);
        break;
    case FrameKind::FadeStop:
        putU32(payload, command.duration_ms);
        break;
    default:
        return std::nullopt;
    }
    return frame(static_cast<std::uint8_t>(command.kind), command.sequence, payload);
}

std::optional<std::string> encodeAck(std::uint64_t sequence, std::uint64_t accepted_ms,
                                     std::uint64_t deadline_ms,
                                     std::string_view message)
{
    std::string payload;
    putU64(payload, sequence);
    putU64(payload, accepted_ms);
    putU64(payload, deadline_ms);
    putString(payload, message);
    return frame(static_cast<std::uint8_t>(FrameKind::Acknowledge), 0, payload);
}

std::optional<std::string> encodeResult(std::uint64_t sequence, std::uint32_t code,
                                        std::uint64_t finished_ms,
                                        std::string_view message)
{
    std::string payload;
    putU64(payload, sequence);
    putU32(payload, code);
    putU64(payload, finished_ms);
    putString(payload, message);
    return frame(static_cast<std::uint8_t>(FrameKind::Result), 0, payload);
}

std::optional<std::string> encodeError(std::uint64_t sequence, ErrorCode code,
                                       std::string_view message)
{
    std::string payload;
    putU64(payload, sequence);
    putU16(payload, static_cast<std::uint16_t>(code));
    putString(payload, message);
    return frame(static_cast<std::uint8_t>(FrameKind::Error), 0, payload);
}

std::optional<std::string> encodePong(std::uint64_t request_id,
                                      std::uint64_t monotonic_ms)
{
    std::string payload;
    putU64(payload, request_id);
    putU64(payload, monotonic_ms);
    return frame(static_cast<std::uint8_t>(FrameKind::Pong), 0, payload);
}

std::optional<std::string> encodeState(std::uint64_t request_id,
                                       const dual_arm_local::Snapshot &snapshot)
{
    std::string payload;
    putU64(payload, request_id);
    putU8(payload, static_cast<std::uint8_t>(snapshot.mode));
    putU8(payload, snapshot.fault_latched ? 1 : 0);
    putU8(payload, static_cast<std::uint8_t>(snapshot.position_limit_policy));
    putU8(payload, static_cast<std::uint8_t>(snapshot.trajectory_execution_policy));
    putU16(payload, static_cast<std::uint16_t>(snapshot.active_mask));
    putU16(payload, static_cast<std::uint16_t>(snapshot.queue_depth));
    putU64(payload, static_cast<std::uint64_t>(snapshot.monotonic_timestamp_ms));
    putU64(payload,
           static_cast<std::uint64_t>(std::max<std::int64_t>(0, snapshot.control_lease_remaining_ms)));
    putU64(payload, snapshot.last_submitted_sequence);
    putU64(payload, snapshot.active_sequence);
    putU64(payload, snapshot.last_result_sequence);
    putU32(payload, static_cast<std::uint32_t>(snapshot.last_result_code));
    putU32(payload, static_cast<std::uint32_t>(std::max<long>(0, snapshot.last_command_span_ms)));
    putString(payload, snapshot.fault_reason);
    putString(payload, snapshot.last_result_message);
    for (const dual_arm_local::JointSnapshot &joint : snapshot.joints)
    {
        putF64(payload, joint.absolute_degrees);
        putF64(payload, joint.target_degrees);
        putI32(payload, joint.tracking_error_steps);
    }
    return frame(static_cast<std::uint8_t>(FrameKind::State), 0, payload);
}
} // namespace noetix::dual_arm_udp