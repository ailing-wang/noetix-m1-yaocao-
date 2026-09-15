#include "dual_arm_local/Controller.h"
#include "dual_arm_local/Profile.h"
#include "dual_arm_udp/Protocol.h"
#include "Xbox360.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <endian.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>

namespace
{
using noetix::dual_arm_local::CommandResult;
using noetix::dual_arm_local::Controller;
using noetix::dual_arm_local::ControllerOptions;
using noetix::dual_arm_local::Snapshot;
using noetix::dual_arm_local::SubmitResult;
using noetix::dual_arm_local::kJointCount;
using noetix::dual_arm_udp::Command;
using noetix::dual_arm_udp::FrameKind;

volatile std::sig_atomic_t g_stop_requested = 0;
std::atomic<std::uint32_t> g_stream_hz{0};

// Legacy-compatible Xbox joystick state feeding the 8888 stream (arm_pos[7]
// gripper stick + 20-int32 button map).  The service never *acts* on joystick
// input; values are forwarded verbatim so downstream collectors keep working
// with the original wire format.
std::mutex g_joystick_mutex;
xbox_map_t g_joystick_map{}; // protected by g_joystick_mutex

bool joystickDevicePresent()
{
    for (int index = 0; index < 8; ++index)
    {
        char path[64];
        std::snprintf(path, sizeof(path), "/dev/input/js%d", index);
        if (access(path, R_OK) == 0)
            return true;
    }
    return false;
}

void joystickLoop()
{
    Xbox360Joystick joystick;
    bool announced_absent = false;
    while (!g_stop_requested)
    {
        if (!joystick.isConnected())
        {
            if (!joystickDevicePresent())
            {
                if (!announced_absent)
                {
                    std::cout << "No joystick found; 8888 stream carries zeroed "
                                 "gripper/map until one is connected\n";
                    announced_absent = true;
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            if (!joystick.initialize())
            {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            announced_absent = false;
            std::cout << "Joystick connected (8888 legacy stream source)\n";
        }
        xbox_map_t local{};
        {
            std::lock_guard<std::mutex> lock(g_joystick_mutex);
            local = g_joystick_map;
        }
        joystick.update(local); // blocks up to 1 s on poll
        {
            std::lock_guard<std::mutex> lock(g_joystick_mutex);
            g_joystick_map = local;
        }
    }
}

void signalHandler(int)
{
    g_stop_requested = 1;
}

struct Options
{
    bool bench_confirmed = false;
    std::string joint_config = "dual_arm_joints.cfg";
    std::string bind_ip = "0.0.0.0";
    std::string peer_ip;
    std::uint16_t port = 8890;
    std::uint16_t telemetry_port = 8891;
    std::uint32_t telemetry_interval_ms = 0;
    std::uint16_t stream_port = 8888;
    std::uint32_t stream_interval_ms = 0;
    std::uint32_t monitor_period_ms = 0;
    std::uint32_t observe_period_ms = 0;
    std::uint32_t status_period_ms = 4;
};

struct CachedRequest
{
    std::string fingerprint;
    std::string acknowledgement;
    std::string result;
    sockaddr_in destination{};
    FrameKind kind = FrameKind::Ping;
    bool pending = false;
};

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program
              << " --bench-confirmed --peer-ip IPV4"
                 " [--bind-ip IPV4] [--port 8890]"
                 " [--joint-config dual_arm_joints.cfg]"
                 " [--telemetry-port 8891] [--telemetry-ms 0]"
                 " [--stream-port 8888] [--stream-ms 0=500Hz legacy default]"
                 " [--monitor-ms 0] [--observe-ms 0] [--status-ms 4]\n";
}

bool parsePort(const std::string &text, std::uint16_t &port)
{
    try
    {
        std::size_t parsed = 0;
        const unsigned long value = std::stoul(text, &parsed, 10);
        if (parsed != text.size() || value == 0 || value > 65535)
            return false;
        port = static_cast<std::uint16_t>(value);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool parseInterval(const std::string &text, std::uint32_t &interval)
{
    try
    {
        const unsigned long value = std::stoul(text, nullptr, 10);
        if (value > 600000)
            return false;
        interval = static_cast<std::uint32_t>(value);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool parseOptions(int argc, char **argv, Options &options)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--bench-confirmed")
            options.bench_confirmed = true;
        else if (argument == "--joint-config" && index + 1 < argc)
            options.joint_config = argv[++index];
        else if (argument == "--bind-ip" && index + 1 < argc)
            options.bind_ip = argv[++index];
        else if (argument == "--peer-ip" && index + 1 < argc)
            options.peer_ip = argv[++index];
        else if (argument == "--port" && index + 1 < argc &&
                 parsePort(argv[index + 1], options.port))
            ++index;
        else if (argument == "--telemetry-port" && index + 1 < argc &&
                 parsePort(argv[index + 1], options.telemetry_port))
            ++index;
        else if (argument == "--telemetry-ms" && index + 1 < argc &&
                 parseInterval(argv[index + 1], options.telemetry_interval_ms))
            ++index;
        else if (argument == "--stream-port" && index + 1 < argc &&
                 parsePort(argv[index + 1], options.stream_port))
            ++index;
        else if (argument == "--stream-ms" && index + 1 < argc &&
                 parseInterval(argv[index + 1], options.stream_interval_ms))
            ++index;
        else if (argument == "--monitor-ms" && index + 1 < argc &&
                 parseInterval(argv[index + 1], options.monitor_period_ms))
            ++index;
        else if (argument == "--observe-ms" && index + 1 < argc &&
                 parseInterval(argv[index + 1], options.observe_period_ms))
            ++index;
        else if (argument == "--status-ms" && index + 1 < argc &&
                 parseInterval(argv[index + 1], options.status_period_ms))
            ++index;
        else if (argument == "--help")
            return false;
        else
            return false;
    }
    return options.bench_confirmed && !options.peer_ip.empty();
}

std::string safeToken(const std::string &message)
{
    std::string result;
    result.reserve(message.size());
    for (const unsigned char character : message)
    {
        if ((character >= 'a' && character <= 'z') ||
            (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') || character == '.' ||
            character == ':' || character == '/' || character == '-' ||
            character == '_')
            result.push_back(static_cast<char>(character));
        else
            result.push_back('_');
    }
    return result.empty() ? "none" : result;
}

std::string canonicalFingerprint(const std::string &datagram)
{
    return datagram;
}

bool sendResponse(int socket_fd, const sockaddr_in &destination,
                  const std::string &response)
{
    const ssize_t sent = sendto(socket_fd, response.data(), response.size(), 0,
                                reinterpret_cast<const sockaddr *>(&destination),
                                sizeof(destination));
    if (sent != static_cast<ssize_t>(response.size()))
    {
        std::cerr << "UDP reply failed: " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}

bool sendError(int socket_fd, const sockaddr_in &destination,
               std::uint64_t sequence, noetix::dual_arm_udp::ErrorCode code,
               const std::string &message)
{
    const auto encoded = noetix::dual_arm_udp::encodeError(sequence, code, message);
    return encoded && sendResponse(socket_fd, destination, *encoded);
}

std::vector<std::uint8_t> buildLegacyStream(std::uint64_t sequence,
                                            const Snapshot &snapshot)
{
    // Wire-compatible with the legacy master program (yaocao/src/main.cpp):
    //   header "!IIII": magic=0x4E4F4554 ("NOET"), version=1,
    //   data_size=224, checksum=sum(payload)&0xFFFFFFFF; each big-endian.
    //   payload "!QQ16d20i": sequence, type=1 (NOETIX_SERIAL_DATA),
    //   left_arm_pos[8] + right_arm_pos[8] in legacy RADIANS around raw
    //   center 2048 (index 7 = gripper stick from the local Xbox joystick,
    //   mapped to +-pi), then the 20-int32 xbox map in legacy order
    //   [time,a,b,x,y,lb,rb,start,back,home,lo,ro,lx,ly,rx,ry,lt,rt,xx,yy].
    //   The controller holds logical degrees with per-joint direction, so
    //   raw radians = absolute_degrees * pi/180 * direction.  The original
    //   master binary additionally applies the legacy wire sign table from
    //   its cfg.txt (LJ2/LJ4/RJ2/RJ4 negated); we reproduce it verbatim so
    //   downstream collectors see the exact legacy convention.
    constexpr std::uint32_t kMagic = 0x4E4F4554;
    constexpr std::uint32_t kVersion = 1;
    constexpr std::uint32_t kDataSize = 224;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kStickToRad = M_PI / 32767.0;
    // Legacy wire signs from yaocao cfg.txt (0=1,1=-1,2=1,3=-1,4=1,...
    // 8=-1,10=-1): LJ2/LJ4 and RJ2/RJ4 are negated on the wire.
    //
    // LJ5/RJ5 deviate from the raw legacy table: dual_arm_joints.cfg gives
    // them direction=-1 while cfg.txt says +1, which would make the stream
    // the NEGATIVE of the logical angle — inverting the slave-follows-master
    // chain (and causing a pose jump at mode switches).  The stream must
    // equal the logical angle (the frame the transfer matrix proved the
    // slave tracks with gain +1.0), so both joints use legacy sign -1.
    constexpr double kLegacySign[14] = {1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0,
                                        1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0};
    std::vector<std::uint8_t> payload(kDataSize, 0);
    std::size_t offset = 0;

    const auto appendU64 = [&payload, &offset](std::uint64_t value)
    {
        const std::uint64_t big = htobe64(value);
        std::memcpy(payload.data() + offset, &big, sizeof(big));
        offset += sizeof(big);
    };
    const auto appendDouble = [&payload, &offset](double value)
    {
        std::uint64_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        const std::uint64_t big = htobe64(bits);
        std::memcpy(payload.data() + offset, &big, sizeof(big));
        offset += sizeof(big);
    };

    xbox_map_t stick;
    {
        std::lock_guard<std::mutex> lock(g_joystick_mutex);
        stick = g_joystick_map;
    }

    appendU64(sequence);
    appendU64(1); // NOETIX_SERIAL_DATA
    // left arm: joints 0..6 (LJ1..LJ7) as legacy radians, index 7 = gripper
    for (std::size_t index = 0; index < 7; ++index)
        appendDouble(snapshot.joints[index].absolute_degrees * kDegToRad *
                     static_cast<double>(snapshot.joints[index].direction) *
                     kLegacySign[index]);
    appendDouble(static_cast<double>(stick.ly) * kStickToRad);
    // right arm: joints 7..13 (RJ1..RJ7), index 7 = gripper
    for (std::size_t index = 7; index < 14; ++index)
        appendDouble(snapshot.joints[index].absolute_degrees * kDegToRad *
                     static_cast<double>(snapshot.joints[index].direction) *
                     kLegacySign[index]);
    appendDouble(static_cast<double>(stick.ry) * kStickToRad);
    // 20 int32 xbox map values, big-endian, legacy field order.
    const std::int32_t map_values[20] = {
        stick.time,  stick.a,  stick.b,  stick.x,     stick.y,
        stick.lb,    stick.rb, stick.start, stick.back, stick.home,
        stick.lo,    stick.ro, stick.lx, stick.ly,    stick.rx,
        stick.ry,    stick.lt, stick.rt, stick.xx,    stick.yy,
    };
    for (const std::int32_t value : map_values)
    {
        const std::uint32_t big = htobe32(static_cast<std::uint32_t>(value));
        std::memcpy(payload.data() + offset, &big, sizeof(big));
        offset += sizeof(big);
    }

    std::uint32_t checksum = 0;
    for (const std::uint8_t byte : payload)
        checksum += byte;

    std::vector<std::uint8_t> frame;
    frame.reserve(16 + kDataSize);
    const auto appendU32 = [&frame](std::uint32_t value)
    {
        const std::uint32_t big = htobe32(value);
        for (std::size_t index = 0; index < sizeof(big); ++index)
            frame.push_back(reinterpret_cast<const std::uint8_t *>(&big)[index]);
    };
    appendU32(kMagic);
    appendU32(kVersion);
    appendU32(kDataSize);
    appendU32(checksum);
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

bool sendLegacyStream(int socket_fd, const sockaddr_in &destination,
                      const std::vector<std::uint8_t> &frame)
{
    const ssize_t sent = sendto(socket_fd, frame.data(), frame.size(), 0,
                                reinterpret_cast<const sockaddr *>(&destination),
                                sizeof(destination));
    if (sent != static_cast<ssize_t>(frame.size()))
    {
        std::cerr << "UDP stream failed: " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}

SubmitResult submitCommand(Controller &controller, const Command &command)
{
    switch (command.kind)
    {
    case FrameKind::Control:
        return controller.submitControl(command.sequence,
                                        std::chrono::milliseconds(command.ttl_ms),
                                        command.selection);
    case FrameKind::Target:
        return controller.submitTrajectory(command.sequence,
                                           std::chrono::milliseconds(command.ttl_ms),
                                           std::chrono::milliseconds(command.duration_ms),
                                           command.targets);
    case FrameKind::StreamTarget:
        return controller.submitStreamTarget(
            command.sequence, std::chrono::milliseconds(command.ttl_ms),
            std::chrono::milliseconds(command.duration_ms), command.targets);
    case FrameKind::Hold:
        return controller.submitHold(command.sequence,
                                     std::chrono::milliseconds(command.ttl_ms));
    case FrameKind::Heartbeat:
        return controller.submitHeartbeat(command.sequence,
                                          std::chrono::milliseconds(command.ttl_ms));
    case FrameKind::Stop:
        return controller.submitStop(command.sequence);
    case FrameKind::FadeStop:
        return controller.submitFadeStop(
            command.sequence,
            std::chrono::milliseconds(command.duration_ms));
    case FrameKind::Reset:
        return controller.submitReset(command.sequence,
                                      std::chrono::milliseconds(command.ttl_ms));
    case FrameKind::Ping:
    case FrameKind::Status:
    case FrameKind::Mode:
        break;
    }
    SubmitResult invalid;
    invalid.sequence = command.sequence;
    invalid.message = "query/configuration command cannot be submitted to controller";
    return invalid;
}

bool commandPending(const std::map<std::uint64_t, CachedRequest> &requests)
{
    return std::any_of(requests.begin(), requests.end(),
                       [](const auto &item) { return item.second.pending; });
}

void collectResults(int socket_fd, Controller &controller,
                    std::map<std::uint64_t, CachedRequest> &requests)
{
    for (auto &[sequence, cached] : requests)
    {
        if (!cached.pending)
            continue;
        CommandResult result;
        if (!controller.waitForResult(sequence, std::chrono::milliseconds(0), result))
            continue;
        cached.pending = false;
        if (const auto encoded = noetix::dual_arm_udp::encodeResult(
                result.sequence, static_cast<std::uint32_t>(result.code),
                result.finished_at_ms, result.message))
            sendResponse(socket_fd, cached.destination, *encoded);
        std::cout << "UDP RESULT sequence=" << sequence
                  << " code=" << noetix::dual_arm_local::toString(result.code)
                  << " message=" << result.message << '\n';
    }

    while (requests.size() > 64)
    {
        const auto removable = std::find_if(
            requests.begin(), requests.end(),
            [](const auto &item) { return !item.second.pending; });
        if (removable == requests.end())
            break;
        requests.erase(removable);
    }
}

void controlledShutdown(Controller &controller)
{
    const Snapshot snapshot = controller.snapshot();
    if (!snapshot.initialized)
    {
        controller.shutdown();
        return;
    }
    if (snapshot.last_submitted_sequence < std::numeric_limits<std::uint64_t>::max())
    {
        const std::uint64_t sequence = snapshot.last_submitted_sequence + 1;
        const SubmitResult stop = controller.submitStop(sequence);
        if (stop.accepted && stop.deadline_at_ms != 0)
        {
            CommandResult result;
            controller.waitForResult(sequence, std::chrono::seconds(6), result);
        }
    }
    controller.shutdown();
}
} // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!parseOptions(argc, argv, options))
    {
        printUsage(argv[0]);
        return 2;
    }

    sockaddr_in bind_address{};
    bind_address.sin_family = AF_INET;
    bind_address.sin_port = htons(options.port);
    if (inet_pton(AF_INET, options.bind_ip.c_str(), &bind_address.sin_addr) != 1)
    {
        std::cerr << "Invalid --bind-ip IPv4 address\n";
        return 2;
    }
    in_addr allowed_peer{};
    if (inet_pton(AF_INET, options.peer_ip.c_str(), &allowed_peer) != 1)
    {
        std::cerr << "Invalid --peer-ip IPv4 address\n";
        return 2;
    }

    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        std::cerr << "UDP socket creation failed: " << std::strerror(errno) << '\n';
        return 1;
    }
    if (bind(socket_fd, reinterpret_cast<const sockaddr *>(&bind_address),
             sizeof(bind_address)) != 0)
    {
        std::cerr << "UDP bind failed: " << std::strerror(errno) << '\n';
        close(socket_fd);
        return 1;
    }

    ControllerOptions controller_options;
    if (options.monitor_period_ms != 0)
        controller_options.control_period =
            std::chrono::milliseconds(options.monitor_period_ms);
    if (options.observe_period_ms != 0)
        controller_options.observe_period =
            std::chrono::milliseconds(options.observe_period_ms);
    if (options.status_period_ms != 0)
        controller_options.status_period =
            std::chrono::milliseconds(options.status_period_ms);
    else
        controller_options.status_period = std::chrono::milliseconds(0);
    std::string configuration_error;
    if (!noetix::dual_arm_local::loadJointConfiguration(
            options.joint_config, controller_options, configuration_error))
    {
        std::cerr << "Cannot load joint configuration: "
                  << configuration_error << '\n';
        close(socket_fd);
        return 2;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    Controller controller(controller_options);
    std::string start_error;
    if (!controller.start(start_error))
    {
        std::cerr << "Controller start failed: " << start_error << '\n';
        close(socket_fd);
        return 1;
    }

    std::cout << "DUAL-ARM UDP BINARY CONTROL\n"
              << "bind=" << options.bind_ip << ':' << options.port
              << " allowed_peer=" << options.peer_ip << "\n"
              << "Protocol=NOETIX_ARM_BIN_V1 binary frames, "
                 "positions=logical absolute degrees, order=LJ1..LJ7,RJ1..RJ7\n"
              << "Legacy 8888 stream: radians around raw-center 2048 with "
                 "xbox gripper/map, 500 Hz default (original-program "
                 "compatible).\n"
              << "MODES: MODE frame layer=STATUS hz=N starts the 8888 status "
                 "stream; layer=CONTROL stops it. Torque untouched.\n"
              << "One command may be in flight; sender must wait for RESULT. "
                 "STOP and STREAM_TARGET always preempt a waited reply.\n"
              << "STREAM_TARGET: fixed-rate position stream, immediate ACK, "
                 "no RESULT, 10 deg/s per-frame rate limit, renews control lease.\n"
<< "Modbus is shared between the controller owner thread and the per-bus\n"
               << "status reader threads; bus access is serialized per port.\n"
               << "Configured trajectory policy is reported by STATUS.\n"
               << "High-frequency status readers: one thread per bus, every "
               << options.status_period_ms << " ms (0 disables).\n";
    sockaddr_in telemetry_destination{};
    bool telemetry_enabled = options.telemetry_interval_ms != 0;
    if (telemetry_enabled)
    {
        telemetry_destination.sin_family = AF_INET;
        telemetry_destination.sin_port = htons(options.telemetry_port);
        telemetry_destination.sin_addr = allowed_peer;
        std::cout << "Telemetry: every " << options.telemetry_interval_ms
                  << " ms to " << options.peer_ip << ':'
                  << options.telemetry_port << '\n';
    }
    std::uint64_t last_telemetry_ms = 0;
    sockaddr_in stream_destination{};
    stream_destination.sin_family = AF_INET;
    stream_destination.sin_port = htons(options.stream_port);
    stream_destination.sin_addr = allowed_peer;
    // Legacy-compatible default: the original master program streams at a
    // fixed 500 Hz.  --stream-ms 0 keeps that default; an explicit N selects
    // 1000/N Hz.  MODE STATUS/CONTROL still switches the stream at runtime.
    g_stream_hz.store(options.stream_interval_ms != 0
                          ? 1000u / options.stream_interval_ms
                          : 500u);
    std::cout << "NOET status stream to " << options.peer_ip << ':'
              << options.stream_port << " (MODE STATUS/CONTROL switches; "
              << "startup " << g_stream_hz.load() << " Hz)\n";
    std::uint64_t last_stream_ms = 0;
    std::uint64_t stream_sequence = 0;

    std::thread joystick_thread(joystickLoop);

    std::thread stream_thread([&]
                              {
                                  while (!g_stop_requested)
                                  {
                                      const std::uint32_t hz = g_stream_hz.load();
                                      if (hz != 0)
                                      {
                                          const Snapshot current =
                                              controller.snapshot();
                                          const std::uint64_t now =
                                              noetix::dual_arm_local::monotonicMilliseconds();
                                          if (static_cast<std::int64_t>(now) -
                                                  static_cast<std::int64_t>(last_stream_ms) >=
                                              1000u / hz)
                                          {
                                              last_stream_ms = now;
                                              sendLegacyStream(
                                                  socket_fd, stream_destination,
                                                  buildLegacyStream(
                                                      ++stream_sequence,
                                                      current));
                                          }
                                      }
                                      std::this_thread::sleep_for(
                                          std::chrono::microseconds(
                                              hz != 0 ? 1000u : 10000u));
                                  }
                              });

    std::map<std::uint64_t, CachedRequest> requests;
    sockaddr_in last_peer{};
    bool have_last_peer = false;
    std::string reported_fault;
    while (!g_stop_requested)
    {
        collectResults(socket_fd, controller, requests);

        const Snapshot current = controller.snapshot();
        if (telemetry_enabled)
        {
            const std::uint64_t now =
                noetix::dual_arm_local::monotonicMilliseconds();
            if (now - last_telemetry_ms >= options.telemetry_interval_ms)
            {
                last_telemetry_ms = now;
                if (const auto encoded = noetix::dual_arm_udp::encodeState(
                        0, current))
                    sendResponse(socket_fd, telemetry_destination, *encoded);
            }
        }
        if (current.fault_latched && current.fault_reason != reported_fault)
        {
            reported_fault = current.fault_reason;
            std::cerr << "FAULT LATCHED: " << reported_fault << '\n';
            if (have_last_peer)
                if (const auto encoded = noetix::dual_arm_udp::encodeState(
                        current.last_result_sequence, current))
                    sendResponse(socket_fd, last_peer, *encoded);
        }

        pollfd descriptor{};
        descriptor.fd = socket_fd;
        descriptor.events = POLLIN;
        const int poll_result = poll(&descriptor, 1, 50);
        if (poll_result < 0)
        {
            if (errno == EINTR)
                continue;
            std::cerr << "UDP poll failed: " << std::strerror(errno) << '\n';
            break;
        }
        if (poll_result == 0 || !(descriptor.revents & POLLIN))
            continue;

        std::array<char, noetix::dual_arm_udp::kMaximumDatagramSize + 1> buffer{};
        sockaddr_in source{};
        socklen_t source_size = sizeof(source);
        const ssize_t received = recvfrom(
            socket_fd, buffer.data(), buffer.size(), MSG_TRUNC,
            reinterpret_cast<sockaddr *>(&source), &source_size);
        if (received < 0)
        {
            if (errno != EINTR)
                std::cerr << "UDP receive failed: " << std::strerror(errno) << '\n';
            continue;
        }
        if (source.sin_addr.s_addr != allowed_peer.s_addr)
        {
            char address[INET_ADDRSTRLEN]{};
            inet_ntop(AF_INET, &source.sin_addr, address, sizeof(address));
            std::cerr << "Ignored datagram from unauthorized peer " << address << '\n';
            continue;
        }
        last_peer = source;
        have_last_peer = true;
        if (received > static_cast<ssize_t>(noetix::dual_arm_udp::kMaximumDatagramSize))
        {
            sendError(socket_fd, source, 0,
                      noetix::dual_arm_udp::ErrorCode::TooLarge,
                      "datagram exceeds 1024 bytes");
            continue;
        }

        const std::string datagram(buffer.data(), static_cast<std::size_t>(received));
        Command command;
        std::string parse_error;
        if (!noetix::dual_arm_udp::decodeFrame(datagram, command, parse_error))
        {
            sendError(socket_fd, source, 0,
                      noetix::dual_arm_udp::ErrorCode::BadPacket, parse_error);
            continue;
        }

        if (command.kind == FrameKind::Ping)
        {
            if (const auto encoded = noetix::dual_arm_udp::encodePong(
                    command.sequence,
                    noetix::dual_arm_local::monotonicMilliseconds()))
                sendResponse(socket_fd, source, *encoded);
            continue;
        }
        if (command.kind == FrameKind::Status)
        {
            if (const auto encoded = noetix::dual_arm_udp::encodeState(
                    command.sequence, controller.snapshot()))
                sendResponse(socket_fd, source, *encoded);
            continue;
        }
        if (command.kind == FrameKind::Mode)
        {
            const std::uint32_t target_hz =
                command.mode_layer == noetix::dual_arm_udp::ModeLayer::Status
                    ? command.stream_hz
                    : 0;
            g_stream_hz.store(target_hz);
            std::cout << "MODE switch thread " << command.sequence
                      << " layer="
                      << (target_hz != 0 ? "STATUS" : "CONTROL")
                      << " hz=" << target_hz << '\n';
            if (const auto encoded = noetix::dual_arm_udp::encodeAck(
                    command.sequence,
                    noetix::dual_arm_local::monotonicMilliseconds(), 0,
                    target_hz != 0 ? "status layer enabled" : "control layer"))
                sendResponse(socket_fd, source, *encoded);
            continue;
        }

        const std::string fingerprint = canonicalFingerprint(datagram);
        const auto duplicate = requests.find(command.sequence);
        if (duplicate != requests.end())
        {
            duplicate->second.destination = source;
            if (duplicate->second.fingerprint != fingerprint)
            {
                sendError(socket_fd, source, command.sequence,
                          noetix::dual_arm_udp::ErrorCode::SequenceReuse,
                          "same sequence used with different content");
            }
            else
            {
                sendResponse(socket_fd, source,
                             duplicate->second.result.empty()
                                 ? duplicate->second.acknowledgement
                                 : duplicate->second.result);
            }
            continue;
        }

        if (command.kind != FrameKind::Stop &&
            command.kind != FrameKind::FadeStop &&
            command.kind != FrameKind::StreamTarget &&
            commandPending(requests))
        {
            sendError(socket_fd, source, command.sequence,
                      noetix::dual_arm_udp::ErrorCode::Busy,
                      "wait for the outstanding RESULT or send STOP");
            continue;
        }

        const SubmitResult submission = submitCommand(controller, command);
        if (!submission.accepted)
        {
            sendError(socket_fd, source, command.sequence,
                      noetix::dual_arm_udp::ErrorCode::Rejected,
                      submission.message);
            continue;
        }

        CachedRequest cached;
        cached.fingerprint = fingerprint;
        cached.acknowledgement = *noetix::dual_arm_udp::encodeAck(
            submission.sequence, submission.accepted_at_ms,
            submission.deadline_at_ms, submission.message);
        cached.destination = source;
        cached.kind = command.kind;
        cached.pending = submission.deadline_at_ms != 0 &&
                         command.kind != FrameKind::StreamTarget;
        requests.emplace(command.sequence, cached);
        sendResponse(socket_fd, source, cached.acknowledgement);
        std::cout << "UDP ACCEPTED command="
                  << noetix::dual_arm_udp::toString(command.kind)
                  << " sequence=" << command.sequence << '\n';
        if (!cached.pending && command.kind != FrameKind::StreamTarget)
        {
            sendError(socket_fd, source, command.sequence,
                      noetix::dual_arm_udp::ErrorCode::OutOfBandStop,
                      "safety stop requested without sequenced result");
        }
    }

    std::cout << "Stopping UDP receiver; torque-off attempt in progress.\n";
    if (stream_thread.joinable())
        stream_thread.join();
    if (joystick_thread.joinable())
        joystick_thread.join();
    controlledShutdown(controller);
    close(socket_fd);
    std::cout << "UDP receiver exited.\n";
    return 0;
}
