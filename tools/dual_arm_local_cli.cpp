#include "dual_arm_local/Calibration.h"
#include "dual_arm_local/Controller.h"
#include "dual_arm_local/Profile.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
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
using noetix::dual_arm_local::CommandResult;
using noetix::dual_arm_local::Controller;
using noetix::dual_arm_local::ControllerOptions;
using noetix::dual_arm_local::JointMask;
using noetix::dual_arm_local::JointLimitDraft;
using noetix::dual_arm_local::Mode;
using noetix::dual_arm_local::SoftLimitEndpoint;
using noetix::dual_arm_local::Snapshot;
using noetix::dual_arm_local::SubmitResult;
using noetix::dual_arm_local::kJointCount;
using noetix::dual_arm_local::kBothArmsMask;
using noetix::dual_arm_local::kLeftArmMask;
using noetix::dual_arm_local::kRightArmMask;

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
              << " --bench-confirmed [--joint-config dual_arm_joints.cfg]"
                 " [--direction-config cfg.txt]\n";
}

void printHelp()
{
    std::cout
        << "Commands (all hardware operations run on the single owner thread):\n"
        << "  status\n"
        << "      Print the latest cached snapshot; does not access Modbus.\n"
        << "  control SEQ TTL_MS SCOPE\n"
        << "      SCOPE is LEFT, RIGHT, BOTH, LJ1..LJ7, or RJ1..RJ7.\n"
        << "      Align selected targets=current, enable only the selection, start lease.\n"
        << "  target SEQ TTL_MS DURATION_MS L1 L2 L3 L4 L5 L6 L7 R1 R2 R3 R4 R5 R6 R7\n"
        << "      Fourteen absolute logical-degree targets; inactive values are ignored.\n"
        << "  target-one SEQ TTL_MS DURATION_MS JOINT ABS_DEG\n"
        << "      Absolute target for the single selected LJ1..LJ7 or RJ1..RJ7.\n"
        << "  probe-all SEQ TTL_MS DURATION_MS DELTA_DEG\n"
        << "      Test helper in BOTH control: capture 14 absolute positions, move by\n"
        << "      DELTA_DEG, then return to the captured absolute positions.\n"
        << "      Uses SEQ and SEQ+1; the next command must use at least SEQ+2.\n"
        << "  hold SEQ TTL_MS\n"
        << "      Capture selected current positions and renew the control lease.\n"
        << "  heartbeat SEQ TTL_MS\n"
        << "      Validate state and renew the control lease without moving.\n"
        << "  stop SEQ\n"
        << "      Preempt queued commands, HOLD active joints, torque OFF all 14 joints.\n"
        << "  reset SEQ TTL_MS\n"
        << "      Clear a latched fault only when communication and safety checks pass.\n"
        << "  capture-limit JOINT MIN|MAX MARGIN_DEG\n"
        << "      OBSERVE/OFF only: record a manually positioned endpoint in RAM.\n"
        << "      The proposed soft limit is moved inward by MARGIN_DEG.\n"
        << "  calibration-draft\n"
        << "      Print captured endpoints and reviewed candidate config rows.\n"
        << "  clear-calibration JOINT|ALL\n"
        << "      Clear RAM-only endpoint captures; never edits the config file.\n"
        << "  help\n"
        << "  quit\n";
}

bool parseScope(std::string text, JointMask &mask)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char character)
                   { return static_cast<char>(std::toupper(character)); });
    if (text == "LEFT")
        mask = kLeftArmMask;
    else if (text == "RIGHT")
        mask = kRightArmMask;
    else if (text == "BOTH")
        mask = kBothArmsMask;
    else if (text.size() == 3 && (text[0] == 'L' || text[0] == 'R') &&
             text[1] == 'J' && text[2] >= '1' && text[2] <= '7')
    {
        const std::size_t arm_offset = text[0] == 'L' ? 0 : 7;
        const std::size_t index = arm_offset + static_cast<std::size_t>(text[2] - '1');
        mask = static_cast<JointMask>(JointMask{1} << index);
    }
    else
        return false;
    return true;
}

bool parseSingleJoint(const std::string &text, std::size_t &index)
{
    JointMask mask = 0;
    if (!parseScope(text, mask) || mask == 0 ||
        (mask & static_cast<JointMask>(mask - 1)) != 0)
        return false;
    for (std::size_t candidate = 0; candidate < kJointCount; ++candidate)
    {
        if ((mask & static_cast<JointMask>(JointMask{1} << candidate)) != 0)
        {
            index = candidate;
            return true;
        }
    }
    return false;
}

bool calibrationCaptureAllowed(const Snapshot &snapshot,
                               std::size_t joint_index,
                               std::string &error)
{
    const std::int64_t age = std::max<std::int64_t>(
        0, noetix::dual_arm_local::monotonicMilliseconds() -
               snapshot.monotonic_timestamp_ms);
    if (!snapshot.initialized || snapshot.mode != Mode::Observe ||
        snapshot.fault_latched || snapshot.active_mask != 0)
        error = "capture requires healthy OBSERVE mode with active_mask=0";
    else if (age > 2000)
        error = "cached snapshot is older than 2000 ms";
    else if (std::any_of(snapshot.joints.begin(), snapshot.joints.end(),
                         [](const auto &joint)
                         { return joint.torque_enabled; }))
        error = "capture requires torque OFF for all 14 joints";
    else if (snapshot.joints[joint_index].moving ||
             snapshot.joints[joint_index].speed_raw != 0)
        error = "selected joint must be stationary before capture";
    return error.empty();
}

void printLimitCapture(const std::string &joint_name,
                       const char *endpoint,
                       const noetix::dual_arm_local::SoftLimitCapture &capture)
{
    std::cout << "CAPTURED " << joint_name << ' ' << endpoint
              << " raw=" << capture.raw_position
              << " measured_deg=" << std::fixed << std::setprecision(6)
              << capture.measured_degrees
              << " margin_deg=" << capture.margin_degrees
              << " proposed_deg=" << capture.proposed_degrees << '\n';
}

void printCalibrationDrafts(
    const Snapshot &snapshot,
    const std::array<JointLimitDraft, kJointCount> &drafts)
{
    bool any = false;
    for (std::size_t index = 0; index < drafts.size(); ++index)
    {
        const JointLimitDraft &draft = drafts[index];
        if (!draft.minimum.captured && !draft.maximum.captured)
            continue;
        any = true;
        const std::string name = noetix::dual_arm_local::jointName(index);
        if (draft.minimum.captured)
            printLimitCapture(name, "MIN", draft.minimum);
        if (draft.maximum.captured)
            printLimitCapture(name, "MAX", draft.maximum);

        std::string line;
        std::string error;
        if (noetix::dual_arm_local::formatCalibratedJointLine(
                snapshot.joints[index], draft, line, error))
            std::cout << "CANDIDATE_REVIEW_REQUIRED " << line << '\n';
        else
            std::cout << "INCOMPLETE " << name << ": " << error << '\n';
    }
    if (!any)
        std::cout << "No RAM-only calibration endpoints captured.\n";
}

void printSnapshot(const Snapshot &snapshot)
{
    const std::int64_t age = std::max<std::int64_t>(
        0, noetix::dual_arm_local::monotonicMilliseconds() -
               snapshot.monotonic_timestamp_ms);
    std::cout << "mode=" << noetix::dual_arm_local::toString(snapshot.mode)
              << " initialized=" << (snapshot.initialized ? "yes" : "no")
              << " snapshot_age_ms=" << age
              << " active_mask=0x" << std::hex << std::setw(4)
              << std::setfill('0') << snapshot.active_mask << std::dec
              << std::setfill(' ')
              << " lease_ms=" << snapshot.control_lease_remaining_ms
              << " queue=" << snapshot.queue_depth
              << " seq{submitted=" << snapshot.last_submitted_sequence
              << ",active=" << snapshot.active_sequence
              << ",result=" << snapshot.last_result_sequence << "}"
              << " span_ms=" << snapshot.last_command_span_ms
              << " position_policy="
              << noetix::dual_arm_local::toString(snapshot.position_limit_policy)
              << " trajectory_policy="
              << noetix::dual_arm_local::toString(
                     snapshot.trajectory_execution_policy)
              << '\n';
    if (snapshot.fault_latched || !snapshot.fault_reason.empty())
        std::cout << "fault=" << snapshot.fault_reason << '\n';
    if (snapshot.last_result_sequence != 0)
    {
        std::cout << "last_result="
                  << noetix::dual_arm_local::toString(snapshot.last_result_code)
                  << " sequence=" << snapshot.last_result_sequence
                  << " message=" << snapshot.last_result_message << '\n';
    }

    std::cout << "motion_feedback:\n"
              << "joint bus id sel status  raw target err_st  pos_deg tgt_deg err_deg speed pwm move tq temp volt current\n";
    for (const auto &joint : snapshot.joints)
    {
        const int arm_bus = joint.arm_joint <= 3 ? 1 : 2;
        std::cout << joint.arm << 'J' << joint.arm_joint << "   "
                  << joint.arm << arm_bus << "  " << joint.slave_id
                  << "   " << (joint.selected ? "yes" : " no")
                  << "  0x" << std::hex << std::setw(4) << std::setfill('0')
                  << joint.status_word << std::dec << std::setfill(' ')
                  << "  " << std::setw(4) << joint.raw_position
                  << "  " << std::setw(4) << joint.raw_target
                  << "  " << std::setw(6) << joint.tracking_error_steps
                  << "  " << std::fixed << std::setprecision(3)
                  << std::setw(7) << joint.absolute_degrees
                  << " " << std::setw(7) << joint.target_degrees
                  << " " << std::setw(7) << joint.tracking_error_degrees
                  << "  " << std::setw(5) << joint.speed_raw
                  << " " << std::setw(4) << joint.pwm_raw
                  << "  " << (joint.moving ? "yes" : " no")
                  << "  " << (joint.torque_enabled ? "ON " : "OFF")
                  << " " << std::setw(4) << joint.temperature_c
                  << " " << std::setw(4) << joint.voltage_v
                  << " " << joint.current_a << '\n';
    }
    std::cout << "engineering_profile_and_runtime_readback:\n"
              << "joint dir zero cal soft_min soft_max session max_delta max_deg_s accel speed torque PID(P/D/I) lock\n";
    for (const auto &joint : snapshot.joints)
    {
        std::cout << joint.arm << 'J' << joint.arm_joint
                  << "    " << std::setw(2) << joint.direction
                  << " " << std::setw(4) << joint.zero_raw
                  << "  " << (joint.soft_limits_calibrated ? "yes" : " no")
                  << " " << std::fixed << std::setprecision(3)
                  << std::setw(8) << joint.soft_min_degrees
                  << " " << std::setw(8) << joint.soft_max_degrees
                  << " " << std::setw(7) << joint.session_workspace_degrees
                  << " " << std::setw(9) << joint.maximum_command_delta_degrees
                  << " " << std::setw(9)
                  << joint.maximum_trajectory_speed_degrees_per_second
                  << " " << std::setw(5) << joint.acceleration_raw
                  << " " << std::setw(5) << joint.speed_limit_raw
                  << " " << std::setw(6) << joint.torque_limit_raw
                  << " " << joint.position_p_raw << '/' << joint.position_d_raw
                  << '/' << joint.position_i_raw
                  << "    " << joint.pid_lock_raw << '\n';
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

bool waitAndPrintResult(Controller &controller,
                        const SubmitResult &submission,
                        std::chrono::milliseconds timeout,
                        CommandResult &result)
{
    printSubmission(submission);
    if (!submission.accepted)
        return false;
    if (!controller.waitForResult(submission.sequence, timeout, result))
    {
        std::cerr << "RESULT TIMEOUT sequence=" << submission.sequence << '\n';
        return false;
    }
    std::cout << "RESULT sequence=" << result.sequence
              << " code=" << noetix::dual_arm_local::toString(result.code)
              << " message=" << result.message << '\n';
    return result.code == noetix::dual_arm_local::ResultCode::Completed;
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
    std::string joint_config = "dual_arm_joints.cfg";
    std::string direction_config;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--bench-confirmed")
            bench_confirmed = true;
        else if (argument == "--joint-config" && index + 1 < argc)
            joint_config = argv[++index];
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
    std::string configuration_error;
    if (!noetix::dual_arm_local::loadJointConfiguration(
            joint_config, options, configuration_error))
    {
        std::cerr << "Cannot load joint configuration: "
                  << configuration_error << '\n';
        return 2;
    }
    if (!direction_config.empty())
    {
        const bool any_calibrated = std::any_of(
            options.engineering_profiles.begin(), options.engineering_profiles.end(),
            [](const noetix::dual_arm_local::JointEngineeringProfile &profile)
            { return profile.soft_limits_calibrated; });
        if (any_calibrated)
        {
            std::cerr << "Legacy --direction-config is disabled when calibrated "
                         "soft limits are active. Put directions in the joint config.\n";
            return 2;
        }
        if (!loadDirections(direction_config, options.directions))
            return 2;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "DUAL-ARM LOCAL ABSOLUTE CONTROL (NO UDP)\n"
              << "Joint engineering config: " << joint_config << "\n"
              << "Each joint has an independent zero, direction, soft-limit state, "
                 "workspace, motion limits, and runtime parameters.\n"
              << "One owner thread serializes all L1/L2/R1/R2 reads and writes; no reconnect/replay.\n"
              << "Uncalibrated joints retain startup-relative limits. Configured PID is runtime-only "
                 "(lock=1), verified, and restored on normal exit.\n";

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
    std::array<JointLimitDraft, kJointCount> calibration_drafts{};
    while (!g_stop_requested)
    {
        const Snapshot current = controller.snapshot();
        if (current.last_result_sequence != 0 &&
            current.last_result_sequence != printed_result_sequence)
        {
            if (prompt_visible)
                std::cout << '\n';
            std::cout << "RESULT sequence=" << current.last_result_sequence
                      << " code=" << noetix::dual_arm_local::toString(current.last_result_code)
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
            std::cout << "dual-arm> " << std::flush;
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
            std::string scope;
            JointMask selection = 0;
            if (!parseSequence(stream, sequence) || !(stream >> ttl_ms) ||
                !(stream >> scope) || !parseScope(scope, selection) ||
                hasExtraToken(stream))
                std::cerr << "Usage: control SEQ TTL_MS LEFT|RIGHT|BOTH|LJ1..LJ7|RJ1..RJ7\n";
            else
                printSubmission(controller.submitControl(
                    sequence, std::chrono::milliseconds(ttl_ms), selection));
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
                std::cerr << "Usage: target SEQ TTL_MS DURATION_MS "
                             "L1 L2 L3 L4 L5 L6 L7 R1 R2 R3 R4 R5 R6 R7\n";
            }
            else
            {
                printSubmission(controller.submitTrajectory(
                    sequence, std::chrono::milliseconds(ttl_ms),
                    std::chrono::milliseconds(duration_ms), targets));
            }
        }
        else if (command == "target-one")
        {
            std::uint64_t sequence = 0;
            long ttl_ms = 0;
            long duration_ms = 0;
            std::string joint_name;
            double absolute_degrees = 0.0;
            JointMask selection = 0;
            const bool parsed = parseSequence(stream, sequence) &&
                                static_cast<bool>(stream >> ttl_ms >> duration_ms >>
                                                  joint_name >> absolute_degrees) &&
                                parseScope(joint_name, selection) &&
                                (selection & static_cast<JointMask>(selection - 1)) == 0;
            if (!parsed || hasExtraToken(stream))
            {
                std::cerr << "Usage: target-one SEQ TTL_MS DURATION_MS "
                             "LJ1..LJ7|RJ1..RJ7 ABS_DEG\n";
            }
            else
            {
                const Snapshot target_snapshot = controller.snapshot();
                if (target_snapshot.mode != Mode::Control ||
                    target_snapshot.active_mask != selection)
                {
                    std::cerr << "target-one requires CONTROL with exactly the same "
                                 "single joint selected.\n";
                }
                else
                {
                    std::array<double, kJointCount> targets{};
                    std::size_t selected_index = 0;
                    for (std::size_t index = 0; index < targets.size(); ++index)
                    {
                        targets[index] = target_snapshot.joints[index].absolute_degrees;
                        if (noetix::dual_arm_local::jointSelected(selection, index))
                            selected_index = index;
                    }
                    targets[selected_index] = absolute_degrees;
                    printSubmission(controller.submitTrajectory(
                        sequence, std::chrono::milliseconds(ttl_ms),
                        std::chrono::milliseconds(duration_ms), targets));
                }
            }
        }
        else if (command == "probe-all")
        {
            std::uint64_t sequence = 0;
            long ttl_ms = 0;
            long duration_ms = 0;
            double delta_degrees = 0.0;
            const bool parsed = parseSequence(stream, sequence) &&
                                static_cast<bool>(stream >> ttl_ms >> duration_ms >>
                                                  delta_degrees) &&
                                std::isfinite(delta_degrees) &&
                                sequence < std::numeric_limits<std::uint64_t>::max();
            if (!parsed || hasExtraToken(stream))
            {
                std::cerr << "Usage: probe-all SEQ TTL_MS DURATION_MS DELTA_DEG\n";
            }
            else
            {
                const Snapshot baseline_snapshot = controller.snapshot();
                if (baseline_snapshot.mode != Mode::Control ||
                    baseline_snapshot.active_mask != kBothArmsMask)
                {
                    std::cerr << "probe-all requires CONTROL with BOTH selected.\n";
                }
                else
                {
                    std::array<double, kJointCount> baseline{};
                    std::array<double, kJointCount> forward{};
                    for (std::size_t index = 0; index < baseline.size(); ++index)
                    {
                        baseline[index] =
                            baseline_snapshot.joints[index].absolute_degrees;
                        forward[index] = baseline[index] + delta_degrees;
                    }
                    const auto wait_timeout = std::chrono::milliseconds(
                        std::max<long>(8000, duration_ms + 5000));
                    CommandResult forward_result;
                    const bool forward_ok = waitAndPrintResult(
                        controller,
                        controller.submitTrajectory(
                            sequence, std::chrono::milliseconds(ttl_ms),
                            std::chrono::milliseconds(duration_ms), forward),
                        wait_timeout, forward_result);
                    if (forward_result.sequence != 0)
                        printed_result_sequence = forward_result.sequence;

                    if (forward_ok)
                    {
                        CommandResult return_result;
                        const bool return_ok = waitAndPrintResult(
                            controller,
                            controller.submitTrajectory(
                                sequence + 1,
                                std::chrono::milliseconds(ttl_ms),
                                std::chrono::milliseconds(duration_ms), baseline),
                            wait_timeout, return_result);
                        if (return_result.sequence != 0)
                            printed_result_sequence = return_result.sequence;
                        if (return_ok)
                            std::cout << "PROBE_ALL PASS: returned to captured absolute targets.\n";
                        else
                            std::cerr << "PROBE_ALL return failed; use STOP with a sequence >= "
                                      << sequence + 2 << ".\n";
                    }
                    else
                    {
                        std::cerr << "PROBE_ALL forward move failed; use STOP with a sequence >= "
                                  << sequence + 2 << ".\n";
                    }
                }
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
        else if (command == "capture-limit")
        {
            std::string joint_name;
            std::string endpoint_text;
            double margin_degrees = 0.0;
            std::size_t joint_index = 0;
            if (!(stream >> joint_name >> endpoint_text >> margin_degrees) ||
                !parseSingleJoint(joint_name, joint_index) || hasExtraToken(stream))
            {
                std::cerr << "Usage: capture-limit LJ1..LJ7|RJ1..RJ7 "
                             "MIN|MAX MARGIN_DEG\n";
                continue;
            }
            std::transform(endpoint_text.begin(), endpoint_text.end(),
                           endpoint_text.begin(),
                           [](unsigned char character)
                           { return static_cast<char>(std::toupper(character)); });
            SoftLimitEndpoint endpoint;
            if (endpoint_text == "MIN")
                endpoint = SoftLimitEndpoint::Minimum;
            else if (endpoint_text == "MAX")
                endpoint = SoftLimitEndpoint::Maximum;
            else
            {
                std::cerr << "Endpoint must be MIN or MAX.\n";
                continue;
            }

            const Snapshot capture_snapshot = controller.snapshot();
            std::string capture_error;
            if (!calibrationCaptureAllowed(capture_snapshot, joint_index,
                                           capture_error) ||
                !noetix::dual_arm_local::captureSoftLimit(
                    capture_snapshot.joints[joint_index], endpoint,
                    margin_degrees, calibration_drafts[joint_index],
                    capture_error))
            {
                std::cerr << "Capture rejected: " << capture_error << '\n';
                continue;
            }
            const auto &capture = endpoint == SoftLimitEndpoint::Minimum
                                      ? calibration_drafts[joint_index].minimum
                                      : calibration_drafts[joint_index].maximum;
            printLimitCapture(noetix::dual_arm_local::jointName(joint_index),
                              endpoint_text.c_str(), capture);
        }
        else if (command == "calibration-draft")
        {
            if (hasExtraToken(stream))
                std::cerr << "Usage: calibration-draft\n";
            else
                printCalibrationDrafts(controller.snapshot(), calibration_drafts);
        }
        else if (command == "clear-calibration")
        {
            std::string scope;
            if (!(stream >> scope) || hasExtraToken(stream))
            {
                std::cerr << "Usage: clear-calibration LJ1..LJ7|RJ1..RJ7|ALL\n";
                continue;
            }
            std::string normalized = scope;
            std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                           [](unsigned char character)
                           { return static_cast<char>(std::toupper(character)); });
            if (normalized == "ALL")
            {
                calibration_drafts = {};
                std::cout << "Cleared all RAM-only calibration captures.\n";
            }
            else
            {
                std::size_t joint_index = 0;
                if (!parseSingleJoint(scope, joint_index))
                {
                    std::cerr << "Usage: clear-calibration "
                                 "LJ1..LJ7|RJ1..RJ7|ALL\n";
                    continue;
                }
                calibration_drafts[joint_index] = {};
                std::cout << "Cleared RAM-only captures for "
                          << noetix::dual_arm_local::jointName(joint_index) << ".\n";
            }
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
            const bool looks_like_bare_angle = !command.empty() &&
                                               (std::isdigit(
                                                    static_cast<unsigned char>(command[0])) ||
                                                command[0] == '-' || command[0] == '+');
            if (looks_like_bare_angle)
                std::cerr << "A bare angle list is not a command. Copy the complete "
                             "line beginning with 'target', or type 'help'.\n";
            else
                std::cerr << "Unknown command. Type 'help'.\n";
        }
    }

    controller.shutdown();
    std::cout << "Exiting local absolute controller with torque-off attempt complete.\n";
    return g_stop_requested ? 130 : 0;
}
