#include "dual_arm_local/Profile.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

namespace
{
std::string makeConfiguration(bool omit_last = false,
                              bool duplicate_first = false,
                              bool invalid_calibrated_limit = false,
                              bool invalid_pid = false)
{
    std::ostringstream text;
    text << "# parser test\nversion=2\n";
    for (std::size_t index = 0;
         index < noetix::dual_arm_local::kJointCount - (omit_last ? 1U : 0U);
         ++index)
    {
        const int direction = index % 2U == 0U ? 1 : -1;
        const bool invalid = invalid_calibrated_limit && index == 0U;
        text << noetix::dual_arm_local::jointName(index) << '=' << direction
             << " 2048 " << (invalid ? 1 : 0)
             << (invalid ? " -300 300" : " -120 120")
             << " 3 1 5 2 2 " << (100 + index)
             << ' ' << (invalid_pid && index == 0U ? 255 : 32)
             << " 32 0\n";
    }
    if (duplicate_first)
        text << "LJ1=1 2048 0 -120 120 3 1 5 2 2 100 32 32 0\n";
    return text.str();
}

std::string makeSoftOnlyConfiguration(bool include_policy = true,
                                      bool calibrated = true,
                                      bool nonzero_session = false,
                                      bool include_trajectory_policy = true,
                                      bool include_pid_schedule = false)
{
    std::ostringstream text;
    text << "version=3\n";
    if (include_policy)
        text << "position_policy=calibrated_soft_only\n";
    if (include_trajectory_policy)
        text << "trajectory_policy=device_profiled\n";
    for (std::size_t index = 0; index < noetix::dual_arm_local::kJointCount;
         ++index)
    {
        text << noetix::dual_arm_local::jointName(index)
             << "=1 2048 " << (calibrated ? 1 : 0)
             << " -120 120 " << (nonzero_session ? "3 1" : "0 0")
             << " 10 3 2 300 32 32 0\n";
    }
    if (include_pid_schedule)
        text << "pid_schedule.LJ1=-60 28 32 28 30 0 32 34 30 32 "
                "60 36 38 34 36 2 2 50 400\n";
    return text.str();
}

std::string writeTemporary(const std::string &contents, int suffix)
{
    const std::string path = "/tmp/noetix_dual_arm_profile_" +
                             std::to_string(static_cast<long>(getpid())) + '_' +
                             std::to_string(suffix) + ".cfg";
    std::ofstream output(path, std::ios::trunc);
    assert(output);
    output << contents;
    output.close();
    assert(output);
    return path;
}

void removeTemporary(const std::string &path)
{
    assert(std::remove(path.c_str()) == 0);
}
} // namespace

int main(int argc, char **argv)
{
    using noetix::dual_arm_local::ControllerOptions;
    using noetix::dual_arm_local::loadJointConfiguration;

    assert(argc == 1 || argc == 2);
    if (argc == 2)
    {
        ControllerOptions deployed;
        std::string deployed_error;
        assert(loadJointConfiguration(argv[1], deployed, deployed_error));
        for (const auto &profile : deployed.engineering_profiles)
        {
            if (deployed.position_limit_policy ==
                noetix::dual_arm_local::PositionLimitPolicy::CalibratedSoftOnly)
            {
                assert(profile.soft_limits_calibrated);
                assert(profile.session_workspace_degrees == 0.0);
                assert(profile.maximum_command_delta_degrees == 0.0);
            }
            else
                assert(!profile.soft_limits_calibrated);
        }
        assert(deployed.engineering_profiles[7].position_p <= 254);
        assert(deployed.engineering_profiles[7].position_d <= 254);
        assert(deployed.engineering_profiles[7].position_i <= 254);
        if (deployed.position_limit_policy ==
            noetix::dual_arm_local::PositionLimitPolicy::CalibratedSoftOnly)
            assert(deployed.trajectory_execution_policy ==
                   noetix::dual_arm_local::TrajectoryExecutionPolicy::DeviceProfiled);
    }

    ControllerOptions options;
    std::string error;
    const std::string valid_path = writeTemporary(makeConfiguration(), 1);
    assert(loadJointConfiguration(valid_path, options, error));
    assert(error.empty());
    assert(options.directions[0] == 1);
    assert(options.directions[1] == -1);
    assert(options.engineering_profiles[13].zero_raw == 2048);
    assert(!options.engineering_profiles[13].soft_limits_calibrated);
    assert(options.safe_torque_limit[13] == 113);
    assert(options.engineering_profiles[0].position_p == 32);
    removeTemporary(valid_path);

    const std::string soft_only_path =
        writeTemporary(makeSoftOnlyConfiguration(), 6);
    assert(loadJointConfiguration(soft_only_path, options, error));
    assert(error.empty());
    assert(options.position_limit_policy ==
           noetix::dual_arm_local::PositionLimitPolicy::CalibratedSoftOnly);
    assert(options.trajectory_execution_policy ==
           noetix::dual_arm_local::TrajectoryExecutionPolicy::DeviceProfiled);
    assert(options.engineering_profiles[0].soft_limits_calibrated);
    assert(options.engineering_profiles[0].session_workspace_degrees == 0.0);
    assert(options.engineering_profiles[0].maximum_command_delta_degrees == 0.0);
    removeTemporary(soft_only_path);

    const std::string scheduled_path =
        writeTemporary(makeSoftOnlyConfiguration(true, true, false, true, true), 11);
    assert(loadJointConfiguration(scheduled_path, options, error));
    assert(error.empty());
    const auto &schedule = options.engineering_profiles[0].pid_schedule;
    assert(schedule.enabled);
    assert(schedule.knots[2].move.p == 36);
    assert(schedule.knots[2].hold.d == 36);
    assert(schedule.maximum_p_step == 2);
    assert(schedule.step_interval.count() == 50);
    const auto low_hold = noetix::dual_arm_local::scheduledPidGain(
        options.engineering_profiles[0], -80.0, false);
    assert(low_hold.p == 28 && low_hold.d == 30);
    const auto middle_move = noetix::dual_arm_local::scheduledPidGain(
        options.engineering_profiles[0], 0.0, true);
    assert(middle_move.p == 32 && middle_move.d == 34);
    const auto interpolated = noetix::dual_arm_local::scheduledPidGain(
        options.engineering_profiles[0], 30.0, true);
    assert(interpolated.p == 34 && interpolated.d == 36);
    removeTemporary(scheduled_path);

    const std::string scheduled_pdi_path = writeTemporary(
        makeSoftOnlyConfiguration(true, true, false, true, false) +
            "pid_schedule.LJ1=-60 65 90 5 28 30 0 "
            "0 65 90 5 32 34 1 "
            "60 65 90 5 34 36 3 2 2 1 50 400\n",
        13);
    assert(loadJointConfiguration(scheduled_pdi_path, options, error));
    assert(error.empty());
    const auto &pdi_schedule = options.engineering_profiles[0].pid_schedule;
    assert(pdi_schedule.maximum_i_step == 1);
    const auto pdi_low_hold = noetix::dual_arm_local::scheduledPidGain(
        options.engineering_profiles[0], -60.0, false);
    assert(pdi_low_hold.p == 28 && pdi_low_hold.d == 30 && pdi_low_hold.i == 0);
    const auto pdi_high_hold = noetix::dual_arm_local::scheduledPidGain(
        options.engineering_profiles[0], 60.0, false);
    assert(pdi_high_hold.p == 34 && pdi_high_hold.d == 36 && pdi_high_hold.i == 3);
    const auto pdi_move = noetix::dual_arm_local::scheduledPidGain(
        options.engineering_profiles[0], 30.0, true);
    assert(pdi_move.p == 65 && pdi_move.d == 90 && pdi_move.i == 5);
    const auto pdi_support = noetix::dual_arm_local::scheduledSupportPidGain(
        options.engineering_profiles[0], 30.0);
    assert(pdi_support.p == 49 && pdi_support.d == 62 && pdi_support.i == 2);
    assert(!noetix::dual_arm_local::pidScheduleRequiresMove(0.0, 0.1));
    assert(noetix::dual_arm_local::pidScheduleRequiresMove(0.0, 0.3));

    removeTemporary(scheduled_pdi_path);

    const std::string u_shaped_hold_path = writeTemporary(
        makeSoftOnlyConfiguration(true, true, false, true, false) +
            "pid_schedule.LJ1=-60 65 90 5 40 50 0 "
            "0 65 90 5 24 28 0 "
            "60 65 90 5 45 55 1 2 2 1 50 400\n"
            "pid_support_ratio.LJ1=0\n",
        14);
    assert(loadJointConfiguration(u_shaped_hold_path, options, error));
    assert(error.empty());
    const auto u_hold_zero = noetix::dual_arm_local::scheduledPidGain(
        options.engineering_profiles[0], 0.0, false);
    assert(u_hold_zero.p == 24 && u_hold_zero.d == 28 && u_hold_zero.i == 0);
    const auto zero_support = noetix::dual_arm_local::scheduledSupportPidGain(
        options.engineering_profiles[0], 0.0);
    assert(zero_support.p == 24 && zero_support.d == 28 && zero_support.i == 0);
    removeTemporary(u_shaped_hold_path);

    const auto check_rejected_without_commit =
        [](const std::string &contents, int suffix)
    {
        ControllerOptions candidate;
        candidate.safe_torque_limit[0] = 777;
        std::string parse_error;
        const std::string path = writeTemporary(contents, suffix);
        const bool loaded = loadJointConfiguration(path, candidate, parse_error);
        if (loaded)
            std::cerr << "unexpectedly accepted suffix " << suffix << '\n';
        assert(!loaded);
        assert(!parse_error.empty());
        assert(candidate.safe_torque_limit[0] == 777);
        removeTemporary(path);
    };

    check_rejected_without_commit(makeConfiguration(true, false, false), 2);
    check_rejected_without_commit(makeConfiguration(false, true, false), 3);
    check_rejected_without_commit(makeConfiguration(false, false, true), 4);
    check_rejected_without_commit(makeConfiguration(false, false, false, true), 5);
    check_rejected_without_commit(makeSoftOnlyConfiguration(false), 7);
    check_rejected_without_commit(makeSoftOnlyConfiguration(true, false), 8);
    check_rejected_without_commit(makeSoftOnlyConfiguration(true, true, true), 9);
    check_rejected_without_commit(
        makeSoftOnlyConfiguration(true, true, false, false), 10);
    check_rejected_without_commit(
        makeSoftOnlyConfiguration(true, true, false, true, false) +
            "pid_schedule.LJ1=0 28 32 28 32 0 30 34 28 32 "
            "60 32 36 30 34 2 2 50 400\n",
        12);

    assert(noetix::dual_arm_local::jointName(0) == "LJ1");
    assert(noetix::dual_arm_local::jointName(13) == "RJ7");
    assert(noetix::dual_arm_local::jointName(14) == "INVALID");
    std::cout << "dual_arm_profile_test: PASS\n";
    return 0;
}
