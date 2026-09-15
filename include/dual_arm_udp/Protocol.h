#pragma once

#include "dual_arm_local/Controller.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace noetix::dual_arm_udp
{
constexpr std::uint32_t kMagic = 0x4E4F4554u; // "NOET"
constexpr std::size_t kFrameHeaderSize = 20;
constexpr std::size_t kFrameCheckSize = 4;
constexpr std::size_t kFrameOverhead = kFrameHeaderSize + kFrameCheckSize;
constexpr std::size_t kMaximumDatagramSize = 1024;

enum class FrameKind : std::uint8_t
{
    Ping = 0,
    Status = 1,
    Mode = 2,
    Control = 3,
    Target = 4,
    StreamTarget = 5,
    Hold = 6,
    Heartbeat = 7,
    Stop = 8,
    Reset = 9,
    Acknowledge = 10,
    Result = 11,
    Error = 12,
    Pong = 13,
    State = 14,
    FadeStop = 15
};

enum class ErrorCode : std::uint16_t
{
    None = 0,
    BadPacket = 1,
    Rejected = 2,
    Busy = 3,
    TooLarge = 4,
    OutOfBandStop = 5,
    SequenceReuse = 6
};

enum class ModeLayer : std::uint8_t
{
    Control = 0,
    Status = 1
};

constexpr std::size_t kJointCount = dual_arm_local::kJointCount;

struct Command
{
    FrameKind kind = FrameKind::Ping;
    std::uint64_t sequence = 0;
    std::uint32_t ttl_ms = 0;
    std::uint32_t duration_ms = 0;
    dual_arm_local::JointMask selection = 0;
    std::array<double, kJointCount> targets{};
    ModeLayer mode_layer = ModeLayer::Control;
    std::uint32_t stream_hz = 0;
};

constexpr std::string_view kProtocolTag = "NOETIX_ARM_BIN_V1";

bool decodeFrame(std::string_view datagram, Command &command, std::string &error);
std::optional<std::string> encodeFrame(const Command &command);
std::optional<std::string> encodeAck(std::uint64_t sequence, std::uint64_t accepted_ms,
                                     std::uint64_t deadline_ms,
                                     std::string_view message);
std::optional<std::string> encodeResult(std::uint64_t sequence, std::uint32_t code,
                                        std::uint64_t finished_ms,
                                        std::string_view message);
std::optional<std::string> encodeError(std::uint64_t sequence, ErrorCode code,
                                       std::string_view message);
std::optional<std::string> encodePong(std::uint64_t request_id,
                                      std::uint64_t monotonic_ms);
std::optional<std::string> encodeState(std::uint64_t request_id,
                                       const dual_arm_local::Snapshot &snapshot);

const char *toString(FrameKind kind) noexcept;

std::uint32_t checksum(std::string_view bytes) noexcept;

std::string formatErrorText(ErrorCode code) noexcept;
} // namespace noetix::dual_arm_udp