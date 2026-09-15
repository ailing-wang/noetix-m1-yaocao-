#include "dual_arm_local/Controller.h"
#include "dual_arm_local/Profile.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>

namespace
{
using namespace std::chrono_literals;
using noetix::dual_arm_local::CommandResult;
using noetix::dual_arm_local::Controller;
using noetix::dual_arm_local::ControllerOptions;
using noetix::dual_arm_local::ResultCode;
using noetix::dual_arm_local::Snapshot;
using noetix::dual_arm_local::SubmitResult;
using noetix::dual_arm_local::kJointCount;

constexpr int kLegacyZeroRaw = 2048;
constexpr int kResultToleranceSteps = 8;
volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int)
{
    g_stop_requested = 1;
}

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program
              << " --bench-confirmed --legacy-zero-confirmed"
                 " [--joint-config dual_arm_joints.cfg]\n";
}

void printPositions(const Snapshot &snapshot, const char *prefix)
{
    int maximum_error = 0;
    std::cout << prefix;
    for (std::size_t index = 0; index < snapshot.joints.size(); ++index)
    {
        const auto &joint = snapshot.joints[index];
        const int error = std::abs(joint.raw_position - joint.zero_raw);
        maximum_error = std::max(maximum_error, error);
        std::cout << ' ' << noetix::dual_arm_local::jointName(index)
                  << '=' << joint.raw_position;
    }
    std::cout << " max_zero_error_steps=" << maximum_error << '\n';
}

bool allTorqueOff(const Snapshot &snapshot)
{
    return std::none_of(snapshot.joints.begin(), snapshot.joints.end(),
                        [](const auto &joint) { return joint.torque_enabled; });
}

bool allAtZero(const Snapshot &snapshot)
{
    return std::all_of(
        snapshot.joints.begin(), snapshot.joints.end(),
        [](const auto &joint)
        {
            return std::abs(joint.raw_position - joint.zero_raw) <=
                   kResultToleranceSteps;
        });
}
} // namespace

int main(int argc, char **argv)
{
    bool bench_confirmed = false;
    bool legacy_zero_confirmed = false;
    std::string joint_config = "dual_arm_joints.cfg";
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--bench-confirmed")
            bench_confirmed = true;
        else if (argument == "--legacy-zero-confirmed")
            legacy_zero_confirmed = true;
        else if (argument == "--joint-config" && index + 1 < argc)
            joint_config = argv[++index];
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
    if (!bench_confirmed || !legacy_zero_confirmed)
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
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        if (options.engineering_profiles[index].zero_raw != kLegacyZeroRaw)
        {
            std::cerr << noetix::dual_arm_local::jointName(index)
                      << " zero_raw is not the legacy value 2048.\n";
            return 2;
        }
    }
    options.allow_legacy_zero_command = true;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "DUAL-ARM DIRECT LEGACY ZERO (NO UDP)\n"
              << "All 14 targets are written once as raw=2048 and held until arrival.\n"
              << "Configured speed/acceleration/torque and runtime PID remain active.\n"
              << "Any joint fault triggers STOP ALL; no reconnect or replay.\n";

    Controller controller(options);
    if (!controller.start(error))
    {
        std::cerr << "Controller start failed: " << error << '\n';
        return 1;
    }
    printPositions(controller.snapshot(), "ZERO_START");

    constexpr std::uint64_t zero_sequence = 1;
    const SubmitResult submission = controller.submitLegacyZero(zero_sequence, 120s);
    if (!submission.accepted)
    {
        std::cerr << "Legacy zero rejected: " << submission.message << '\n';
        controller.shutdown();
        return 1;
    }

    CommandResult result;
    bool have_result = false;
    while (!g_stop_requested)
    {
        if (controller.waitForResult(zero_sequence, 500ms, result))
        {
            have_result = true;
            break;
        }
        const Snapshot snapshot = controller.snapshot();
        if (snapshot.fault_latched)
            break;
    }

    if (g_stop_requested)
    {
        constexpr std::uint64_t stop_sequence = 2;
        const SubmitResult stop = controller.submitStop(stop_sequence);
        CommandResult stop_result;
        if (stop.accepted)
            (void)controller.waitForResult(stop_sequence, 8s, stop_result);
    }

    const Snapshot final_snapshot = controller.snapshot();
    printPositions(final_snapshot, "ZERO_END");
    if (final_snapshot.fault_latched)
        std::cerr << "fault=" << final_snapshot.fault_reason << '\n';

    const bool final_still_at_zero = allAtZero(final_snapshot);
    const bool passed = have_result && result.code == ResultCode::Completed &&
                        allTorqueOff(final_snapshot);
    if (have_result)
        std::cout << "ZERO_COMMAND_RESULT code="
                  << noetix::dual_arm_local::toString(result.code)
                  << " message=" << result.message << '\n';
    if (passed && !final_still_at_zero)
        std::cout << "POST_STOP_DRIFT: one or more gravity-loaded joints moved after "
                     "torque OFF; the raw=2048 arrival was confirmed before STOP ALL.\n";
    controller.shutdown();

    if (!passed)
    {
        std::cerr << "LEGACY_ZERO_RESULT STOPPED\n";
        return g_stop_requested ? 130 : 1;
    }
    std::cout << "LEGACY_ZERO_RESULT PASS\n";
    return 0;
}
