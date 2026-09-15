#include "dual_arm_udp/Protocol.h"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace
{
using noetix::dual_arm_udp::Command;
using noetix::dual_arm_udp::FrameKind;

void require(bool condition, const std::string &message)
{
    if (!condition)
    {
        std::cerr << "TEST FAILURE: " << message << '\n';
        std::exit(1);
    }
}

Command roundTrip(const Command &source)
{
    const auto frame = noetix::dual_arm_udp::encodeFrame(source);
    require(frame.has_value(), "request could not be encoded");
    Command decoded;
    std::string error;
    require(noetix::dual_arm_udp::decodeFrame(*frame, decoded, error),
            "decode failed: " + error);
    return decoded;
}
} // namespace

int main()
{
    Command ping;
    ping.kind = FrameKind::Ping;
    ping.sequence = 7;
    const Command decoded_ping = roundTrip(ping);
    require(decoded_ping.kind == FrameKind::Ping && decoded_ping.sequence == 7,
            "PING fields are wrong");

    Command control;
    control.kind = FrameKind::Control;
    control.sequence = 8;
    control.ttl_ms = 3000;
    control.selection = noetix::dual_arm_local::kBothArmsMask;
    const Command decoded_control = roundTrip(control);
    require(decoded_control.kind == FrameKind::Control &&
                decoded_control.ttl_ms == 3000 &&
                decoded_control.selection == noetix::dual_arm_local::kBothArmsMask,
            "CONTROL fields are wrong");

    Command target;
    target.kind = FrameKind::Target;
    target.sequence = 9;
    target.ttl_ms = 5000;
    target.duration_ms = 800;
    for (std::size_t index = 0; index < target.targets.size(); ++index)
        target.targets[index] = static_cast<double>(index + 1);
    const Command decoded_target = roundTrip(target);
    require(decoded_target.kind == FrameKind::Target &&
                decoded_target.ttl_ms == 5000 &&
                decoded_target.duration_ms == 800 &&
                decoded_target.targets.front() == 1.0 &&
                decoded_target.targets.back() == 14.0,
            "TARGET fields are wrong");

    Command mode;
    mode.kind = FrameKind::Mode;
    mode.sequence = 10;
    mode.mode_layer = noetix::dual_arm_udp::ModeLayer::Status;
    mode.stream_hz = 100;
    const Command decoded_mode = roundTrip(mode);
    require(decoded_mode.mode_layer == noetix::dual_arm_udp::ModeLayer::Status &&
                decoded_mode.stream_hz == 100,
            "MODE fields are wrong");

    Command invalid_sequence = ping;
    invalid_sequence.sequence = 0;
    require(!noetix::dual_arm_udp::encodeFrame(invalid_sequence).has_value(),
            "zero sequence was accepted");

    const auto valid_frame = noetix::dual_arm_udp::encodeFrame(target);
    require(valid_frame.has_value(), "TARGET frame missing");
    std::string corrupted = *valid_frame;
    corrupted.back() ^= 0x01;
    Command rejected;
    std::string error;
    require(!noetix::dual_arm_udp::decodeFrame(corrupted, rejected, error) &&
                error == "checksum mismatch",
            "bad checksum was accepted");

    target.targets[13] = std::numeric_limits<double>::quiet_NaN();
    const auto nan_frame = noetix::dual_arm_udp::encodeFrame(target);
    require(nan_frame.has_value(), "NaN test frame was not encoded");
    error.clear();
    require(!noetix::dual_arm_udp::decodeFrame(*nan_frame, rejected, error) &&
                error == "all absolute targets must be finite",
            "NaN target was accepted");

    std::cout << "dual_arm_udp_protocol_test: PASS\n";
    return 0;
}
