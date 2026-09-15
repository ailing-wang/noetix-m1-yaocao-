#include "dual_arm_local/Calibration.h"
#include "dual_arm_local/Controller.h"
#include "dual_arm_local/Profile.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace
{
using noetix::dual_arm_local::Controller;
using noetix::dual_arm_local::ControllerOptions;
using noetix::dual_arm_local::JointExtrema;
using noetix::dual_arm_local::JointLimitDraft;
using noetix::dual_arm_local::JointSnapshot;
using noetix::dual_arm_local::Mode;
using noetix::dual_arm_local::Snapshot;
using noetix::dual_arm_local::kJointCount;

constexpr double kMinimumUsableSpanDegrees = 1.0;
constexpr double kMinimumSafetyMarginDegrees = 0.5;
constexpr double kMaximumSafetyMarginDegrees = 30.0;
volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int)
{
    g_stop_requested = 1;
}

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program
              << " --bench-confirmed [--joint-config dual_arm_joints.cfg]"
                 " [--output-dir logs] [--resume-csv prior.csv]\n";
}

void printHelp()
{
    std::cout
        << "Commands (this recorder has no torque-enable or motion command):\n"
        << "  start\n"
        << "      Reset extrema and begin recording all 14 joints.\n"
        << "  progress\n"
        << "      Show raw/degree minima, maxima, span, and zero coverage.\n"
        << "  status | diagnostics\n"
        << "      Show controller mode, exact fault reason, and per-joint status.\n"
        << "  finish MARGIN_DEG\n"
        << "      Stop recording and write CSV plus a reviewed candidate config.\n"
        << "      Each proposed limit is moved inward by MARGIN_DEG.\n"
        << "  abort\n"
        << "      Stop recording and keep only the CSV; write no candidate config.\n"
        << "  help\n"
        << "  quit\n";
}

bool hasExtraToken(std::istringstream &stream)
{
    std::string extra;
    return static_cast<bool>(stream >> extra);
}

std::string sessionToken()
{
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&now, &local_time);
    char buffer[32]{};
    if (std::strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", &local_time) == 0)
        return "unknown-time-" + std::to_string(static_cast<long>(getpid()));
    return std::string(buffer) + '-' +
           std::to_string(static_cast<long>(getpid()));
}

bool snapshotSafeForRecording(const Snapshot &snapshot, std::string &error)
{
    error.clear();
    const std::int64_t age = std::max<std::int64_t>(
        0, noetix::dual_arm_local::monotonicMilliseconds() -
               snapshot.monotonic_timestamp_ms);
    if (!snapshot.initialized)
        error = "controller snapshot is not initialized";
    else if (snapshot.mode != Mode::Observe)
    {
        error = "controller mode=" +
                std::string(noetix::dual_arm_local::toString(snapshot.mode));
        if (!snapshot.fault_reason.empty())
            error += " fault=" + snapshot.fault_reason;
    }
    else if (snapshot.fault_latched)
        error = "controller fault=" + snapshot.fault_reason;
    else if (snapshot.active_mask != 0)
    {
        std::ostringstream message;
        message << "active_mask=0x" << std::hex << snapshot.active_mask
                << " but recorder requires 0";
        error = message.str();
    }
    else if (age > 2000)
        error = "cached snapshot is older than 2000 ms";
    else if (std::any_of(snapshot.joints.begin(), snapshot.joints.end(),
                         [](const JointSnapshot &joint)
                         { return joint.torque_enabled; }))
        error = "recorder requires torque OFF for all 14 joints";
    return error.empty();
}

std::string csvHeader()
{
    std::ostringstream output;
    output << "monotonic_ms";
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        const std::string name = noetix::dual_arm_local::jointName(index);
        output << ',' << name << "_raw," << name << "_deg";
    }
    return output.str();
}

void writeCsvHeader(std::ofstream &output)
{
    output << csvHeader() << '\n';
}

std::vector<std::string> splitCsv(const std::string &line)
{
    std::vector<std::string> fields;
    std::istringstream input(line);
    std::string field;
    while (std::getline(input, field, ','))
        fields.push_back(field);
    return fields;
}

bool parseInteger(const std::string &text, long long &value)
{
    try
    {
        std::size_t parsed = 0;
        value = std::stoll(text, &parsed, 10);
        return parsed == text.size();
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool parseDouble(const std::string &text, double &value)
{
    try
    {
        std::size_t parsed = 0;
        value = std::stod(text, &parsed);
        return parsed == text.size() && std::isfinite(value);
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool loadRecordedExtrema(const std::filesystem::path &path,
                         const ControllerOptions &options,
                         std::array<JointExtrema, kJointCount> &extrema,
                         std::string &error)
{
    error.clear();
    std::ifstream input(path);
    if (!input)
    {
        error = "cannot open resume CSV: " + path.string();
        return false;
    }
    std::string line;
    if (!std::getline(input, line))
    {
        error = "resume CSV is empty";
        return false;
    }
    if (!line.empty() && line.back() == '\r')
        line.pop_back();
    if (line != csvHeader())
    {
        error = "resume CSV header does not match the 14-joint recorder format";
        return false;
    }

    std::array<JointExtrema, kJointCount> candidate{};
    std::size_t row_count = 0;
    std::size_t line_number = 1;
    while (std::getline(input, line))
    {
        ++line_number;
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.empty())
            continue;
        const std::vector<std::string> fields = splitCsv(line);
        if (fields.size() != 1 + 2 * kJointCount)
        {
            error = "resume CSV line " + std::to_string(line_number) +
                    " has the wrong field count";
            return false;
        }
        long long timestamp = 0;
        if (!parseInteger(fields[0], timestamp) || timestamp < 0)
        {
            error = "resume CSV line " + std::to_string(line_number) +
                    " has an invalid timestamp";
            return false;
        }
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            long long raw = 0;
            double degrees = 0.0;
            if (!parseInteger(fields[1 + 2 * index], raw) ||
                raw < 0 || raw > 4095 ||
                !parseDouble(fields[2 + 2 * index], degrees))
            {
                error = "resume CSV line " + std::to_string(line_number) +
                        " has invalid data for " +
                        noetix::dual_arm_local::jointName(index);
                return false;
            }
            JointSnapshot joint;
            joint.arm = index < 7 ? 'L' : 'R';
            joint.arm_joint = static_cast<int>(index % 7) + 1;
            joint.direction = options.directions[index];
            joint.zero_raw = options.engineering_profiles[index].zero_raw;
            joint.raw_position = static_cast<int>(raw);
            joint.absolute_degrees = degrees;
            if (!noetix::dual_arm_local::updateJointExtrema(
                    joint, candidate[index], error))
                return false;
        }
        ++row_count;
    }
    if (input.bad())
    {
        error = "failed while reading resume CSV";
        return false;
    }
    if (row_count == 0)
    {
        error = "resume CSV contains no samples";
        return false;
    }
    extrema = candidate;
    return true;
}

bool appendPriorCsvRows(const std::filesystem::path &path,
                        std::ofstream &output,
                        std::string &error)
{
    std::ifstream input(path);
    std::string line;
    if (!input || !std::getline(input, line))
    {
        error = "cannot reopen resume CSV: " + path.string();
        return false;
    }
    while (std::getline(input, line))
    {
        if (!line.empty())
            output << line << '\n';
    }
    if (input.bad() || !output)
    {
        error = "failed while copying resume CSV rows";
        return false;
    }
    return true;
}

bool recordSnapshot(const Snapshot &snapshot,
                    std::array<JointExtrema, kJointCount> &extrema,
                    std::ofstream &csv,
                    std::int64_t &last_timestamp,
                    std::string &error)
{
    if (!snapshotSafeForRecording(snapshot, error))
        return false;
    if (snapshot.monotonic_timestamp_ms == last_timestamp)
        return true;

    for (std::size_t index = 0; index < extrema.size(); ++index)
    {
        if (!noetix::dual_arm_local::updateJointExtrema(
                snapshot.joints[index], extrema[index], error))
            return false;
    }

    csv << snapshot.monotonic_timestamp_ms;
    for (const JointSnapshot &joint : snapshot.joints)
        csv << ',' << joint.raw_position << ',' << std::fixed
            << std::setprecision(6) << joint.absolute_degrees;
    csv << '\n';
    csv.flush();
    if (!csv)
    {
        error = "failed while writing recording CSV";
        return false;
    }
    last_timestamp = snapshot.monotonic_timestamp_ms;
    return true;
}

void printProgress(const std::array<JointExtrema, kJointCount> &extrema)
{
    std::cout << "joint samples min_raw max_raw min_deg max_deg span_deg zero_seen\n";
    for (std::size_t index = 0; index < extrema.size(); ++index)
    {
        const JointExtrema &value = extrema[index];
        std::cout << std::setw(4) << noetix::dual_arm_local::jointName(index)
                  << ' ' << std::setw(7) << value.samples;
        if (!value.observed)
        {
            std::cout << " not-observed\n";
            continue;
        }
        const double span = value.maximum_degrees - value.minimum_degrees;
        const bool zero_seen = value.minimum_degrees <= 0.0 &&
                               value.maximum_degrees >= 0.0;
        std::cout << ' ' << std::setw(7) << value.minimum_raw
                  << ' ' << std::setw(7) << value.maximum_raw
                  << ' ' << std::fixed << std::setprecision(3)
                  << std::setw(8) << value.minimum_degrees
                  << ' ' << std::setw(8) << value.maximum_degrees
                  << ' ' << std::setw(8) << span
                  << ' ' << (zero_seen ? "yes" : "no") << '\n';
    }
}

std::string formatCurrentJointLine(const JointSnapshot &joint)
{
    std::ostringstream output;
    output << joint.arm << 'J' << joint.arm_joint << '=' << joint.direction
           << ' ' << joint.zero_raw
           << ' ' << (joint.soft_limits_calibrated ? 1 : 0)
           << ' ' << std::fixed << std::setprecision(6)
           << joint.soft_min_degrees << ' ' << joint.soft_max_degrees
           << ' ' << joint.session_workspace_degrees
           << ' ' << joint.maximum_command_delta_degrees
           << ' ' << joint.maximum_trajectory_speed_degrees_per_second
           << ' ' << joint.acceleration_raw
           << ' ' << joint.speed_limit_raw
           << ' ' << joint.torque_limit_raw
           << ' ' << joint.position_p_raw
           << ' ' << joint.position_d_raw
           << ' ' << joint.position_i_raw;
    return output.str();
}

bool writeCandidateConfiguration(
    const std::filesystem::path &path,
    const Snapshot &snapshot,
    const std::array<JointExtrema, kJointCount> &extrema,
    double margin_degrees,
    std::size_t &updated_count,
    std::string &error)
{
    error.clear();
    updated_count = 0;
    const std::filesystem::path temporary = path.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    if (!output)
    {
        error = "cannot create candidate config: " + temporary.string();
        return false;
    }

    output << "# REVIEW REQUIRED: automatically recorded extrema; not deployed.\n"
           << "# Safety margin: " << std::fixed << std::setprecision(6)
           << margin_degrees << " deg. Zero convention: raw 2048 = 0 deg.\n"
           << "version=2\n";
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        JointLimitDraft draft;
        std::string joint_error;
        std::string candidate_line;
        const bool ready = noetix::dual_arm_local::draftSoftLimitsFromExtrema(
                               snapshot.joints[index], extrema[index],
                               margin_degrees, kMinimumUsableSpanDegrees,
                               draft, joint_error) &&
                           noetix::dual_arm_local::formatCalibratedJointLine(
                               snapshot.joints[index], draft,
                               candidate_line, joint_error);
        if (ready)
        {
            ++updated_count;
            output << "# RECORDED measured_min=" << std::fixed
                   << std::setprecision(6) << extrema[index].minimum_degrees
                   << " measured_max=" << extrema[index].maximum_degrees
                   << " samples=" << extrema[index].samples << '\n'
                   << candidate_line << '\n';
        }
        else
        {
            output << "# NOT_UPDATED "
                   << noetix::dual_arm_local::jointName(index)
                   << ": " << joint_error << '\n'
                   << formatCurrentJointLine(snapshot.joints[index]) << '\n';
        }
    }
    output.close();
    if (!output)
    {
        error = "failed while writing candidate config";
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
    }

    ControllerOptions validation_options;
    std::string validation_error;
    if (!noetix::dual_arm_local::loadJointConfiguration(
            temporary.string(), validation_options, validation_error))
    {
        error = "generated candidate failed validation: " + validation_error;
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
    }

    std::error_code rename_error;
    std::filesystem::rename(temporary, path, rename_error);
    if (rename_error)
    {
        error = "cannot finalize candidate config: " + rename_error.message();
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
    }
    return true;
}
} // namespace

int main(int argc, char **argv)
{
    bool bench_confirmed = false;
    std::string joint_config = "dual_arm_joints.cfg";
    std::filesystem::path output_directory = "logs";
    std::filesystem::path resume_csv;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--bench-confirmed")
            bench_confirmed = true;
        else if (argument == "--joint-config" && index + 1 < argc)
            joint_config = argv[++index];
        else if (argument == "--output-dir" && index + 1 < argc)
            output_directory = argv[++index];
        else if (argument == "--resume-csv" && index + 1 < argc)
            resume_csv = argv[++index];
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
    std::string error;
    if (!noetix::dual_arm_local::loadJointConfiguration(
            joint_config, options, error))
    {
        std::cerr << "Cannot load joint configuration: " << error << '\n';
        return 2;
    }
    options.observe_period = std::chrono::milliseconds(500);

    std::array<JointExtrema, kJointCount> extrema{};
    bool resume_loaded = false;
    if (!resume_csv.empty())
    {
        if (!loadRecordedExtrema(resume_csv, options, extrema, error))
        {
            std::cerr << "Cannot resume recording: " << error << '\n';
            return 2;
        }
        resume_loaded = true;
        std::cout << "Loaded prior extrema from " << resume_csv << ".\n";
        printProgress(extrema);
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "DUAL-ARM PASSIVE LIMIT RECORDER\n"
              << "No torque-enable or motion command is available.\n"
              << "All 14 joints must remain torque OFF. Pause at every endpoint "
                 "for at least 2 seconds.\n";
    Controller controller(options);
    if (!controller.start(error))
    {
        std::cerr << "Recorder start failed: " << error << '\n';
        return 1;
    }

    Snapshot initial = controller.snapshot();
    if (!snapshotSafeForRecording(initial, error))
    {
        std::cerr << "Unsafe initial state: " << error << '\n';
        controller.shutdown();
        return 1;
    }
    std::cout << "Ready in OBSERVE with all torque OFF. Type 'start'.\n";
    printHelp();

    bool recording = false;
    std::ofstream csv;
    std::filesystem::path csv_path;
    std::int64_t last_recorded_timestamp = -1;
    bool prompt_visible = false;

    while (!g_stop_requested)
    {
        if (recording)
        {
            const Snapshot current = controller.snapshot();
            std::string sample_error;
            if (!recordSnapshot(current, extrema, csv,
                                last_recorded_timestamp, sample_error))
            {
                std::cerr << "Recording aborted: " << sample_error << '\n';
                csv.close();
                recording = false;
                prompt_visible = false;
            }
        }

        if (!prompt_visible)
        {
            std::cout << "limit-recorder> " << std::flush;
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

        if (command == "start")
        {
            if (hasExtraToken(stream))
            {
                std::cerr << "Usage: start\n";
                continue;
            }
            if (recording)
            {
                std::cerr << "Recording is already active.\n";
                continue;
            }
            const Snapshot current = controller.snapshot();
            if (!snapshotSafeForRecording(current, error))
            {
                std::cerr << "Cannot start: " << error << '\n';
                continue;
            }
            std::error_code directory_error;
            std::filesystem::create_directories(output_directory,
                                                directory_error);
            if (directory_error)
            {
                std::cerr << "Cannot create output directory: "
                          << directory_error.message() << '\n';
                continue;
            }
            const std::string token = "dual-arm-limits-" + sessionToken();
            csv_path = output_directory / (token + ".csv");
            csv.open(csv_path, std::ios::trunc);
            if (!csv)
            {
                std::cerr << "Cannot create " << csv_path << '\n';
                continue;
            }
            writeCsvHeader(csv);
            if (resume_loaded)
            {
                if (!appendPriorCsvRows(resume_csv, csv, error))
                {
                    std::cerr << "Cannot merge prior CSV: " << error << '\n';
                    csv.close();
                    std::error_code ignored;
                    std::filesystem::remove(csv_path, ignored);
                    continue;
                }
            }
            else
                extrema = {};
            last_recorded_timestamp = -1;
            recording = true;
            if (!recordSnapshot(current, extrema, csv,
                                last_recorded_timestamp, error))
            {
                std::cerr << "Cannot record initial sample: " << error << '\n';
                csv.close();
                recording = false;
                continue;
            }
            std::cout << "Recording all 14 joints to " << csv_path << "\n"
                      << (resume_loaded ? "Prior samples were merged into this session.\n"
                                        : "")
                      << "Move each joint slowly through both chosen endpoints; "
                         "pause at each endpoint for >=2 seconds.\n";
            resume_loaded = false;
        }
        else if (command == "progress")
        {
            if (hasExtraToken(stream))
                std::cerr << "Usage: progress\n";
            else
                printProgress(extrema);
        }
        else if (command == "diagnostics" || command == "status")
        {
            if (hasExtraToken(stream))
            {
                std::cerr << "Usage: " << command << "\n";
                continue;
            }
            const Snapshot current = controller.snapshot();
            std::cout << "mode=" << noetix::dual_arm_local::toString(current.mode)
                      << " initialized=" << (current.initialized ? "yes" : "no")
                      << " active_mask=0x" << std::hex << current.active_mask
                      << std::dec
                      << " fault_latched="
                      << (current.fault_latched ? "yes" : "no") << '\n';
            if (!current.fault_reason.empty())
                std::cout << "fault=" << current.fault_reason << '\n';
            for (const JointSnapshot &joint : current.joints)
            {
                std::cout << joint.arm << 'J' << joint.arm_joint
                          << " status=0x" << std::hex << joint.status_word
                          << std::dec << " raw=" << joint.raw_position
                          << " torque=" << (joint.torque_enabled ? "ON" : "OFF")
                          << " temp=" << joint.temperature_c
                          << " voltage=" << joint.voltage_v << '\n';
            }
        }
        else if (command == "finish")
        {
            double margin_degrees = 0.0;
            if (!(stream >> margin_degrees) || hasExtraToken(stream) ||
                !std::isfinite(margin_degrees) ||
                margin_degrees < kMinimumSafetyMarginDegrees ||
                margin_degrees > kMaximumSafetyMarginDegrees)
            {
                std::cerr << "Usage: finish MARGIN_DEG, where MARGIN_DEG is "
                          << kMinimumSafetyMarginDegrees << ".."
                          << kMaximumSafetyMarginDegrees
                          << " and is applied inward.\n";
                continue;
            }
            if (!recording)
            {
                std::cerr << "No active recording.\n";
                continue;
            }
            const Snapshot current = controller.snapshot();
            if (!recordSnapshot(current, extrema, csv,
                                last_recorded_timestamp, error))
            {
                std::cerr << "Cannot finish safely: " << error << '\n';
                csv.close();
                recording = false;
                continue;
            }
            csv.close();
            recording = false;
            printProgress(extrema);

            std::filesystem::path candidate_path = csv_path;
            candidate_path.replace_extension(".candidate.cfg");
            std::size_t updated_count = 0;
            if (!writeCandidateConfiguration(candidate_path, current, extrema,
                                             margin_degrees, updated_count,
                                             error))
            {
                std::cerr << "Candidate generation failed: " << error << '\n';
                continue;
            }
            std::cout << "Recording CSV: " << csv_path << '\n'
                      << "Candidate config: " << candidate_path << '\n'
                      << "Candidate calibrated joints: " << updated_count
                      << "/14. The live config was NOT changed.\n";
        }
        else if (command == "abort")
        {
            if (hasExtraToken(stream))
            {
                std::cerr << "Usage: abort\n";
                continue;
            }
            if (!recording)
            {
                std::cerr << "No active recording.\n";
                continue;
            }
            csv.close();
            recording = false;
            std::cout << "Recording aborted; partial CSV kept at "
                      << csv_path << ". No candidate config was written.\n";
        }
        else if (command == "help")
            printHelp();
        else if (command == "quit" || command == "exit")
            break;
        else
            std::cerr << "Unknown command. Type 'help'.\n";
    }

    if (recording)
    {
        csv.close();
        std::cout << "Interrupted recording; partial CSV kept at "
                  << csv_path << ".\n";
    }
    controller.shutdown();
    std::cout << "Recorder exited with torque-off shutdown complete.\n";
    return g_stop_requested ? 130 : 0;
}
