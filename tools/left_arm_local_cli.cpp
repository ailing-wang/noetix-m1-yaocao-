#include "left_arm_local/Controller.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <string>
#include <unistd.h>

namespace
{
using noetix::left_arm_local::CommandResult;
using noetix::left_arm_local::Controller;
using noetix::left_arm_local::ControllerOptions;
using noetix::left_arm_local::Mode;
using noetix::left_arm_local::Snapshot;
using noetix::left_arm_local::SubmitResult;
using noetix::left_arm_local::kJointCount;

volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int)
{
    g_stop_requested = 1;
}

bool loadDirections(const std::string &path,
                    std::array<int, kJointCount> &directions)
{
    std::ifstream input(path);
    if (!input)
    {
        std::cerr << "Cannot open direction config: " << path << '\n';
        return false;
    }

    std::array<bool, kJointCount> seen{};
    std::string line;
    while (std::getline(input, line))
    {
        const std::size_t separator = line.find('=');
        if (separator == std::string::npos)
            continue;
        try
        {
            const int index = std::stoi(line.substr(0, separator));
            const int value = std::stoi(line.substr(separator + 1));
            if (index >= 0 && index < static_cast<int>(kJointCount))
            {
                if (value != -1 && value != 1)
                {
                    std::cerr << "Direction " << index << " must be -1 or 1.\n";
                    return false;
                }
                directions[static_cast<std::size_t>(index)] = value;
                seen[static_cast<std::size_t>(index)] = true;
            }
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid direction config line: " << line << '\n';
            return false;
        }
    }

    for (std::size_t index = 0; index < seen.size(); ++index)
    {
        if (!seen[index])
        {
            std::cerr << "Direction config is missing index " << index << ".\n";
            return false;
        }
    }
    return true;
}

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program
              << " --bench-confirmed [--direction-config cfg.txt]\n";
}

void printHelp()
{
    std::cout
        << "Commands (all hardware operations run on the single owner thread):\n"
        << "  status\n"
        << "      Print the latest cached snapshot; does not access Modbus.\n"
        << "  control SEQ TTL_MS\n"
        << "      Align all targets=current, enable seven joints, start control lease.\n"
        << "  target SEQ TTL_MS DURATION_MS J1 J2 J3 J4 J5 J6 J7\n"
        << "      Seven absolute logical-degree targets around raw=2048.\n"
        << "  hold SEQ TTL_MS\n"
        << "      Capture all current positions and renew the control lease.\n"
        << "  heartbeat SEQ TTL_MS\n"
        << "      Validate state and renew the control lease without moving.\n"
        << "  stop SEQ\n"
        << "      Preempt queued commands, HOLD, and confirm torque OFF for all joints.\n"
        << "  reset SEQ TTL_MS\n"
        << "      Clear a latched fault only when communication and safety checks pass.\n"
        << "  help\n"
        << "  quit\n";
}

void printSnapshot(const Snapshot &snapshot)
{
    const std::int64_t age = std::max<std::int64_t>(
        0, noetix::left_arm_local::monotonicMilliseconds() -
               snapshot.monotonic_timestamp_ms);
    std::cout << "mode=" << noetix::left_arm_local::toString(snapshot.mode)
              << " initialized=" << (snapshot.initialized ? "yes" : "no")
              << " snapshot_age_ms=" << age
              << " lease_ms=" << snapshot.control_lease_remaining_ms
              << " queue=" << snapshot.queue_depth
              << " seq{submitted=" << snapshot.last_submitted_sequence
              << ",active=" << snapshot.active_sequence
              << ",result=" << snapshot.last_result_sequence << "}"
              << " span_ms=" << snapshot.last_command_span_ms << '\n';
    if (snapshot.fault_latched || !snapshot.fault_reason.empty())
        std::cout << "fault=" << snapshot.fault_reason << '\n';
    if (snapshot.last_result_sequence != 0)
    {
        std::cout << "last_result="
                  << noetix::left_arm_local::toString(snapshot.last_result_code)
                  << " sequence=" << snapshot.last_result_sequence
                  << " message=" << snapshot.last_result_message << '\n';
    }

    std::cout << "joint bus id dir status  raw  abs_deg target target_deg moving torque temp voltage current\n";
    for (const auto &joint : snapshot.joints)
    {
        const int bus = joint.joint <= 3 ? 1 : 2;
        std::cout << 'J' << joint.joint << "    L" << bus << "  " << joint.slave_id
                  << "  " << std::setw(2) << joint.direction
                  << "  0x" << std::hex << std::setw(4) << std::setfill('0')
                  << joint.status_word << std::dec << std::setfill(' ')
                  << "  " << std::setw(4) << joint.raw_position
                  << "  " << std::fixed << std::setprecision(3) << std::setw(7)
                  << joint.absolute_degrees
                  << "  " << std::setw(4) << joint.raw_target
                  << "  " << std::setw(9) << joint.target_degrees
                  << "   " << (joint.moving ? "yes" : " no")
                  << "    " << (joint.torque_enabled ? "ON " : "OFF")
                  << "   " << std::setw(4) << joint.temperature_c
                  << "  " << std::setw(5) << joint.voltage_v
                  << "  " << joint.current_a << '\n';
    }
    std::cout << "absolute_target_template: target "
              << (snapshot.last_submitted_sequence + 1)
              << " 5000 500";
    for (const auto &joint : snapshot.joints)
        std::cout << ' ' << std::fixed << std::setprecision(6)
                  << joint.absolute_degrees;
    std::cout << '\n';
}

void printSubmission(const SubmitResult &result)
{
    if (!result.accepted)
    {
        std::cerr << "REJECTED sequence=" << result.sequence
                  << " reason=" << result.message << '\n';
        return;
    }
    std::cout << "ACCEPTED sequence=" << result.sequence
              << " accepted_at_ms=" << result.accepted_at_ms
              << " deadline_at_ms=" << result.deadline_at_ms
              << " message=" << result.message << '\n';
}

bool hasExtraToken(std::istringstream &stream)
{
    std::string extra;
    return static_cast<bool>(stream >> extra);
}

bool parseSequence(std::istringstream &stream, std::uint64_t &sequence)
{
    std::string token;
    if (!(stream >> token) || token.empty())
        return false;
    if (!std::all_of(token.begin(), token.end(),
                     [](char character)
                     { return character >= '0' && character <= '9'; }))
        return false;
    try
    {
        std::size_t parsed = 0;
        const unsigned long long value = std::stoull(token, &parsed, 10);
        if (parsed != token.size() || value == 0 ||
            value > std::numeric_limits<std::uint64_t>::max())
            return false;
        sequence = static_cast<std::uint64_t>(value);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}
} // namespace

int main(int argc, char **argv)
{
    bool bench_confirmed = false;
    std::string direction_config = "cfg.txt";
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--bench-confirmed")
            bench_confirmed = true;
        else if (argument == "--direction-config" && index + 1 < argc)
            direction_config = argv[++index];
        else if (argument == "--help")
        {
            printUsage(argv[0]);
            return 0;
        }
        else
        {
            printUsage(argv[0]);
            return 2;
        }
    }

    if (!bench_confirmed)
    {
        printUsage(argv[0]);
        return 2;
    }

    ControllerOptions options;
    if (!loadDirections(direction_config, options.directions))
        return 2;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "LEFT-ARM LOCAL ABSOLUTE CONTROL (NO UDP)\n"
              << "Canonical coordinates: raw=2048 is 0 deg; cfg.txt applies logical direction.\n"
              << "Limits: startup workspace +/-3 deg, command delta <=1 deg, speed <=5 deg/s.\n"
              << "One owner thread serializes all L1/L2 reads and writes; no reconnect/replay.\n";

    Controller controller(options);
    std::string error;
    if (!controller.start(error))
    {
        std::cerr << "Controller start failed: " << error << '\n';
        return 1;
    }

    printSnapshot(controller.snapshot());
    printHelp();

    bool prompt_visible = false;
    std::uint64_t printed_result_sequence = 0;
    std::string printed_fault;
    while (!g_stop_requested)
    {
        const Snapshot current = controller.snapshot();
        if (current.last_result_sequence != 0 &&
            current.last_result_sequence != printed_result_sequence)
        {
            if (prompt_visible)
                std::cout << '\n';
            std::cout << "RESULT sequence=" << current.last_result_sequence
                      << " code=" << noetix::left_arm_local::toString(current.last_result_code)
                      << " message=" << current.last_result_message << '\n';
            printed_result_sequence = current.last_result_sequence;
            prompt_visible = false;
        }
        if (current.fault_latched && current.fault_reason != printed_fault)
        {
            if (prompt_visible)
                std::cout << '\n';
            std::cerr << "FAULT LATCHED: " << current.fault_reason << '\n';
            printed_fault = current.fault_reason;
            prompt_visible = false;
        }

        if (!prompt_visible)
        {
            std::cout << "local-arm> " << std::flush;
            prompt_visible = true;
        }

        pollfd input{};
        input.fd = STDIN_FILENO;
        input.events = POLLIN | POLLHUP | POLLERR;
        const int poll_result = poll(&input, 1, 100);
        if (poll_result < 0 && errno != EINTR)
        {
            std::cerr << "stdin poll failed\n";
            break;
        }
        if (poll_result <= 0)
            continue;
        if (input.revents & (POLLHUP | POLLERR))
            break;
        if (!(input.revents & POLLIN))
            continue;

        std::string line;
        if (!std::getline(std::cin, line))
            break;
        prompt_visible = false;
        std::istringstream stream(line);
        std::string command;
        stream >> command;
        if (command.empty())
            continue;

        if (command == "status")
        {
            if (hasExtraToken(stream))
                std::cerr << "Usage: status\n";
            else
                printSnapshot(controller.snapshot());
        }
        else if (command == "control")
        {
            std::uint64_t sequence = 0;
            long ttl_ms = 0;
            if (!parseSequence(stream, sequence) || !(stream >> ttl_ms) ||
                hasExtraToken(stream))
                std::cerr << "Usage: control SEQ TTL_MS\n";
            else
                printSubmission(controller.submitControl(
                    sequence, std::chrono::milliseconds(ttl_ms)));
        }
        else if (command == "target")
        {
            std::uint64_t sequence = 0;
            long ttl_ms = 0;
            long duration_ms = 0;
            std::array<double, kJointCount> targets{};
            bool valid = parseSequence(stream, sequence) &&
                         static_cast<bool>(stream >> ttl_ms >> duration_ms);
            for (double &target : targets)
                valid = valid && static_cast<bool>(stream >> target);
            if (!valid || hasExtraToken(stream))
            {
                std::cerr << "Usage: target SEQ TTL_MS DURATION_MS J1 J2 J3 J4 J5 J6 J7\n";
            }
            else
            {
                printSubmission(controller.submitTrajectory(
                    sequence, std::chrono::milliseconds(ttl_ms),
                    std::chrono::milliseconds(duration_ms), targets));
            }
        }
        else if (command == "hold")
        {
            std::uint64_t sequence = 0;
            long ttl_ms = 0;
            if (!parseSequence(stream, sequence) || !(stream >> ttl_ms) ||
                hasExtraToken(stream))
                std::cerr << "Usage: hold SEQ TTL_MS\n";
            else
                printSubmission(controller.submitHold(
                    sequence, std::chrono::milliseconds(ttl_ms)));
        }
        else if (command == "heartbeat")
        {
            std::uint64_t sequence = 0;
            long ttl_ms = 0;
            if (!parseSequence(stream, sequence) || !(stream >> ttl_ms) ||
                hasExtraToken(stream))
                std::cerr << "Usage: heartbeat SEQ TTL_MS\n";
            else
                printSubmission(controller.submitHeartbeat(
                    sequence, std::chrono::milliseconds(ttl_ms)));
        }
        else if (command == "stop")
        {
            std::uint64_t sequence = 0;
            if (!parseSequence(stream, sequence) || hasExtraToken(stream))
                std::cerr << "Usage: stop SEQ\n";
            else
                printSubmission(controller.submitStop(sequence));
        }
        else if (command == "reset")
        {
            std::uint64_t sequence = 0;
            long ttl_ms = 0;
            if (!parseSequence(stream, sequence) || !(stream >> ttl_ms) ||
                hasExtraToken(stream))
                std::cerr << "Usage: reset SEQ TTL_MS\n";
            else
                printSubmission(controller.submitReset(
                    sequence, std::chrono::milliseconds(ttl_ms)));
        }
        else if (command == "help")
        {
            printHelp();
        }
        else if (command == "quit" || command == "exit")
        {
            break;
        }
        else
        {
            std::cerr << "Unknown command. Type 'help'.\n";
        }
    }

    controller.shutdown();
    std::cout << "Exiting local absolute controller with torque-off attempt complete.\n";
    return g_stop_requested ? 130 : 0;
}
