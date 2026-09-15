#include "dual_arm_local/Controller.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

extern "C" void fake_modbus_reset();
extern "C" void fake_modbus_disconnect_bus(int bus_index);
extern "C" void fake_modbus_set_position_on_bus(int bus, int slave,
                                                  int position);
extern "C" void fake_modbus_set_temperature_on_bus(int bus, int slave,
                                                     int temperature);
extern "C" int fake_modbus_torque_on_bus(int bus, int slave);
extern "C" void fake_modbus_set_runtime_on_bus(int bus, int slave,
                                                 int acceleration, int speed,
                                                 int torque_limit);
extern "C" void fake_modbus_set_pid_on_bus(int bus, int slave,
                                             int position_p, int position_d,
                                             int position_i, int lock_flag);
extern "C" int fake_modbus_pid_value_on_bus(int bus, int slave, int field);
extern "C" int fake_modbus_target_write_count();

namespace
{
using namespace std::chrono_literals;
using noetix::dual_arm_local::CommandResult;
using noetix::dual_arm_local::Controller;
using noetix::dual_arm_local::ControllerOptions;
using noetix::dual_arm_local::JointMask;
using noetix::dual_arm_local::Mode;
using noetix::dual_arm_local::ResultCode;
using noetix::dual_arm_local::SubmitResult;
using noetix::dual_arm_local::jointSelected;
using noetix::dual_arm_local::kBothArmsMask;
using noetix::dual_arm_local::kJointCount;
using noetix::dual_arm_local::kLeftArmMask;

[[noreturn]] void fail(const std::string &message)
{
    std::cerr << "TEST FAILURE: " << message << '\n';
    std::exit(1);
}

void require(bool condition, const std::string &message)
{
    if (!condition)
        fail(message);
}

CommandResult waitResult(Controller &controller, std::uint64_t sequence,
                         std::chrono::milliseconds timeout = 8s)
{
    CommandResult result;
    require(controller.waitForResult(sequence, timeout, result),
            "timed out waiting for sequence " + std::to_string(sequence));
    return result;
}

void requireCompleted(Controller &controller, const SubmitResult &submission,
                      std::chrono::milliseconds timeout = 8s)
{
    require(submission.accepted, "submission rejected: " + submission.message);
    const CommandResult result = waitResult(controller, submission.sequence, timeout);
    require(result.code == ResultCode::Completed,
            "sequence " + std::to_string(result.sequence) + " ended " +
                noetix::dual_arm_local::toString(result.code) + ": " + result.message);
}

bool waitForMode(Controller &controller, Mode mode,
                 std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (controller.snapshot().mode == mode)
            return true;
        std::this_thread::sleep_for(20ms);
    }
    return false;
}

int busForJoint(std::size_t index)
{
    const std::size_t arm_index = index % 7;
    const int offset = index < 7 ? 0 : 2;
    return offset + (arm_index < 3 ? 0 : 1);
}

int slaveForJoint(std::size_t index)
{
    return static_cast<int>(index % 7 + 1);
}

void requireTorqueMask(JointMask expected)
{
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        const int actual = fake_modbus_torque_on_bus(
            busForJoint(index), slaveForJoint(index));
        require(actual == (jointSelected(expected, index) ? 1 : 0),
                "unexpected torque state at global joint " +
                    std::to_string(index + 1));
    }
}

std::array<double, kJointCount> positionsFromSnapshot(
    const noetix::dual_arm_local::Snapshot &snapshot)
{
    std::array<double, kJointCount> positions{};
    for (std::size_t index = 0; index < positions.size(); ++index)
        positions[index] = snapshot.joints[index].absolute_degrees;
    return positions;
}
} // namespace

int main()
{
    fake_modbus_reset();
    fake_modbus_set_pid_on_bus(2, 1, 31, 30, 1, 0);
    ControllerOptions options;
    options.require_startup_inactive = false;
    options.observe_period = 50ms;
    options.control_period = 50ms;
    options.trajectory_tick = 100ms;
    options.maximum_queue_depth = 2;
    options.safe_acceleration[0] = 3;
    options.safe_speed[0] = 4;
    options.safe_torque_limit[0] = 125;
    options.engineering_profiles[13].zero_raw = 2800;
    options.engineering_profiles[13].soft_limits_calibrated = true;
    options.engineering_profiles[13].soft_min_degrees = -5.0;
    options.engineering_profiles[13].soft_max_degrees = 5.0;
    options.engineering_profiles[7].position_p = 28;

    Controller controller(options);
    std::string error;
    require(controller.start(error), "start failed: " + error);
    require(controller.snapshot().mode == Mode::Observe,
            "startup mode is not OBSERVE");
    const auto startup = controller.snapshot();
    require(startup.joints[0].acceleration_raw == 3 &&
                startup.joints[0].speed_limit_raw == 4 &&
                startup.joints[0].torque_limit_raw == 125,
            "per-joint runtime parameter readback is wrong");
    require(startup.joints[0].pwm_raw == 0 &&
                startup.joints[0].tracking_error_steps == 0,
            "feedback PWM or tracking error was not published");
    require(startup.joints[7].position_p_raw == 28 &&
                startup.joints[7].position_d_raw == 32 &&
                startup.joints[7].position_i_raw == 0 &&
                startup.joints[7].pid_lock_raw == 1,
            "RJ1 runtime-only PID readback is wrong");
    require(fake_modbus_pid_value_on_bus(2, 1, 0) == 28 &&
                fake_modbus_pid_value_on_bus(2, 1, 3) == 1,
            "RJ1 runtime-only PID was not written to fake hardware");
    require(startup.joints[13].zero_raw == 2800 &&
                startup.joints[13].soft_limits_calibrated &&
                std::abs(startup.joints[13].absolute_degrees - 3.516) < 0.01,
            "calibrated zero/soft-limit profile was not applied");
    requireTorqueMask(0);

    require(!controller.submitControl(1, 5s, 0).accepted,
            "zero control mask was accepted");
    requireCompleted(controller, controller.submitControl(1, 5s, kLeftArmMask));
    require(controller.snapshot().active_mask == kLeftArmMask,
            "left-arm selection was not published");
    requireTorqueMask(kLeftArmMask);

    const auto left_start = controller.snapshot();
    auto targets = positionsFromSnapshot(left_start);
    for (std::size_t index = 0; index < 7; ++index)
        targets[index] += 0.4;
    for (std::size_t index = 7; index < targets.size(); ++index)
        targets[index] = 999.0; // Inactive-arm values must never reach hardware.
    requireCompleted(controller,
                     controller.submitTrajectory(2, 3s, 500ms, targets));
    const auto left_reached = controller.snapshot();
    for (std::size_t index = 0; index < 7; ++index)
        require(std::abs(left_reached.joints[index].target_degrees - targets[index]) < 0.1,
                "left absolute target rounding too large");
    for (std::size_t index = 7; index < kJointCount; ++index)
        require(left_reached.joints[index].raw_target ==
                    left_start.joints[index].raw_target,
                "inactive right-arm target changed");
    requireCompleted(controller, controller.submitStop(3));
    requireTorqueMask(0);

    constexpr JointMask right_wrist = static_cast<JointMask>(1U << 13U);
    requireCompleted(controller, controller.submitControl(4, 5s, right_wrist));
    requireTorqueMask(right_wrist);
    const auto wrist_start = controller.snapshot();
    targets = positionsFromSnapshot(wrist_start);
    targets[13] += 0.5;
    requireCompleted(controller,
                     controller.submitTrajectory(5, 3s, 500ms, targets));
    requireCompleted(controller, controller.submitHold(6, 3s));
    requireCompleted(controller, controller.submitStop(7));
    requireTorqueMask(0);

    requireCompleted(controller, controller.submitControl(8, 8s, kBothArmsMask));
    requireTorqueMask(kBothArmsMask);
    const auto both_start = controller.snapshot();
    targets = positionsFromSnapshot(both_start);
    for (double &target : targets)
        target += 0.3;
    requireCompleted(controller,
                     controller.submitTrajectory(9, 4s, 700ms, targets));
    targets = positionsFromSnapshot(both_start);
    requireCompleted(controller,
                     controller.submitTrajectory(10, 4s, 700ms, targets));
    const auto both_returned = controller.snapshot();
    for (std::size_t index = 0; index < kJointCount; ++index)
        require(both_returned.joints[index].raw_target ==
                    both_start.joints[index].raw_position,
                "dual-arm absolute return drifted");
    requireCompleted(controller, controller.submitStop(11));
    requireTorqueMask(0);

    requireCompleted(controller, controller.submitControl(12, 1s, kBothArmsMask));
    require(waitForMode(controller, Mode::Fault, 3s),
            "control lease expiry did not latch FAULT");
    requireTorqueMask(0);
    requireCompleted(controller, controller.submitReset(13, 2s));

    requireCompleted(controller, controller.submitControl(14, 5s, kBothArmsMask));
    fake_modbus_set_temperature_on_bus(2, 3, 70);
    require(waitForMode(controller, Mode::Fault, 3s),
            "right-arm temperature fault did not latch FAULT");
    requireTorqueMask(0);
    fake_modbus_set_temperature_on_bus(2, 3, 38);
    requireCompleted(controller, controller.submitReset(15, 2s));

    requireCompleted(controller, controller.submitControl(16, 5s, kBothArmsMask));
    fake_modbus_set_runtime_on_bus(0, 1, 2, 4, 125);
    require(waitForMode(controller, Mode::Fault, 3s),
            "runtime parameter mismatch did not latch FAULT");
    requireTorqueMask(0);
    fake_modbus_set_runtime_on_bus(0, 1, 3, 4, 125);
    requireCompleted(controller, controller.submitReset(17, 2s));

    requireCompleted(controller, controller.submitControl(18, 5s, kBothArmsMask));
    fake_modbus_set_pid_on_bus(2, 1, 27, 32, 0, 1);
    require(waitForMode(controller, Mode::Fault, 3s),
            "runtime PID mismatch did not latch FAULT");
    requireTorqueMask(0);
    fake_modbus_set_pid_on_bus(2, 1, 28, 32, 0, 1);
    requireCompleted(controller, controller.submitReset(19, 2s));

    controller.shutdown();
    require(fake_modbus_pid_value_on_bus(2, 1, 0) == 31 &&
                fake_modbus_pid_value_on_bus(2, 1, 1) == 30 &&
                fake_modbus_pid_value_on_bus(2, 1, 2) == 1 &&
                fake_modbus_pid_value_on_bus(2, 1, 3) == 0,
            "normal shutdown did not restore RJ1 startup PID and lock");

    // STREAM_TARGET must make only the joint with a changed target stiff.
    // Once the same target is stable for settle_delay it must return to HOLD;
    // unchanged selected joints must remain at HOLD throughout.
    fake_modbus_reset();
    ControllerOptions stream_options;
    stream_options.require_startup_inactive = false;
    stream_options.observe_period = 50ms;
    stream_options.control_period = 50ms;
    stream_options.status_period = 0ms;
    for (std::size_t index : {std::size_t{0}, std::size_t{1}})
    {
        auto &schedule = stream_options.engineering_profiles[index].pid_schedule;
        schedule.enabled = true;
        schedule.knots[0] = {-90.0, {40, 20, 0}, {4, 1, 0}};
        schedule.knots[1] = {0.0, {40, 20, 0}, {4, 1, 0}};
        schedule.knots[2] = {90.0, {40, 20, 0}, {4, 1, 0}};
        schedule.maximum_p_step = 8;
        schedule.maximum_d_step = 8;
        schedule.maximum_i_step = 1;
        schedule.step_interval = 20ms;
        schedule.settle_delay = 100ms;
    }
    Controller stream_controller(stream_options);
    require(stream_controller.start(error),
            "stream controller start failed: " + error);
    requireCompleted(stream_controller,
                     stream_controller.submitControl(100, 5s, kLeftArmMask));
    require(fake_modbus_pid_value_on_bus(0, 1, 0) == 4 &&
                fake_modbus_pid_value_on_bus(0, 2, 0) == 4,
            "CONTROL did not apply per-joint HOLD gains");
    auto stream_targets = positionsFromSnapshot(stream_controller.snapshot());
    stream_targets[0] += 1.0;
    requireCompleted(stream_controller,
                     stream_controller.submitStreamTarget(
                         101, 5s, 500ms, stream_targets));
    require(fake_modbus_pid_value_on_bus(0, 1, 0) == 40,
            "changed stream joint did not enter MOVE");
    require(fake_modbus_pid_value_on_bus(0, 2, 0) == 4,
            "unchanged stream joint left HOLD");
    requireCompleted(stream_controller,
                     stream_controller.submitStreamTarget(
                         102, 5s, 500ms, stream_targets));
    std::this_thread::sleep_for(120ms);
    requireCompleted(stream_controller,
                     stream_controller.submitStreamTarget(
                         103, 5s, 500ms, stream_targets));
    require(fake_modbus_pid_value_on_bus(0, 1, 0) == 4,
            "stable stream joint did not return to HOLD");
    require(fake_modbus_pid_value_on_bus(0, 2, 0) == 4,
            "unchanged stream joint did not remain in HOLD");

    // Compliance/encoder noise on an unchanged joint must not wake it back
    // into MOVE.  The command target is still identical; feedback may only
    // delay a joint already in MOVE/SETTLING.
    const int lj2_target_raw = stream_controller.snapshot().joints[1].raw_target;
    fake_modbus_set_position_on_bus(0, 2, lj2_target_raw + 5);
    requireCompleted(stream_controller,
                     stream_controller.submitStreamTarget(
                         104, 5s, 500ms, stream_targets));
    require(fake_modbus_pid_value_on_bus(0, 2, 0) == 4,
            "unchanged stream joint feedback error woke HOLD into MOVE");
    requireCompleted(stream_controller, stream_controller.submitStop(105));
    stream_controller.shutdown();

    fake_modbus_reset();
    ControllerOptions legacy_zero_options;
    legacy_zero_options.require_startup_inactive = false;
    legacy_zero_options.observe_period = 50ms;
    legacy_zero_options.control_period = 50ms;
    Controller legacy_zero_disabled(legacy_zero_options);
    require(legacy_zero_disabled.start(error),
            "legacy-zero-disabled controller start failed: " + error);
    require(!legacy_zero_disabled.submitLegacyZero(1, 20s).accepted,
            "legacy zero was accepted without explicit capability");
    legacy_zero_disabled.shutdown();

    fake_modbus_reset();
    legacy_zero_options.allow_legacy_zero_command = true;
    Controller legacy_zero_controller(legacy_zero_options);
    require(legacy_zero_controller.start(error),
            "legacy-zero controller start failed: " + error);
    requireCompleted(legacy_zero_controller,
                     legacy_zero_controller.submitLegacyZero(1, 20s));
    requireTorqueMask(0);
    require(legacy_zero_controller.snapshot().mode == Mode::Observe,
            "legacy zero did not return to OBSERVE");
    legacy_zero_controller.shutdown();

    fake_modbus_reset();
    Controller disconnect_controller(options);
    require(disconnect_controller.start(error),
            "disconnect controller start failed: " + error);
    requireCompleted(disconnect_controller,
                     disconnect_controller.submitControl(1, 5s, kBothArmsMask));
    fake_modbus_disconnect_bus(3);
    require(waitForMode(disconnect_controller, Mode::Fault, 3s),
            "right-arm bus disconnect did not latch FAULT");
    const SubmitResult reset_after_disconnect =
        disconnect_controller.submitReset(2, 2s);
    require(reset_after_disconnect.accepted,
            "disconnect RESET submission rejected too early");
    require(waitResult(disconnect_controller, 2).code == ResultCode::Rejected,
            "terminal communication failure RESET was not rejected");

    disconnect_controller.shutdown();

    fake_modbus_reset();
    ControllerOptions soft_only_options;
    soft_only_options.require_startup_inactive = false;
    soft_only_options.observe_period = 50ms;
    soft_only_options.control_period = 50ms;
    soft_only_options.trajectory_tick = 50ms;
    soft_only_options.position_limit_policy =
        noetix::dual_arm_local::PositionLimitPolicy::CalibratedSoftOnly;
    soft_only_options.trajectory_execution_policy =
        noetix::dual_arm_local::TrajectoryExecutionPolicy::DeviceProfiled;
    for (auto &profile : soft_only_options.engineering_profiles)
    {
        profile.soft_limits_calibrated = true;
        profile.soft_min_degrees = -120.0;
        profile.soft_max_degrees = 120.0;
        profile.session_workspace_degrees = 0.0;
        profile.maximum_command_delta_degrees = 0.0;
        profile.maximum_trajectory_speed_degrees_per_second = 100.0;
    }
    Controller soft_only_controller(soft_only_options);
    require(soft_only_controller.start(error),
            "soft-only controller start failed: " + error);
    constexpr JointMask left_first = 1;
    requireCompleted(soft_only_controller,
                     soft_only_controller.submitControl(1, 5s, left_first));
    auto soft_only_targets =
        positionsFromSnapshot(soft_only_controller.snapshot());
    soft_only_targets[0] += 40.0;
    const int writes_before_device_target = fake_modbus_target_write_count();
    requireCompleted(soft_only_controller,
                     soft_only_controller.submitTrajectory(
                         2, 4s, 500ms, soft_only_targets));
    require(fake_modbus_target_write_count() - writes_before_device_target == 1,
            "device-profiled TARGET did not write exactly one final target");
    requireCompleted(soft_only_controller,
                     soft_only_controller.submitStop(3));
    requireTorqueMask(0);
    soft_only_controller.shutdown();

    std::cout << "dual_arm_local_service_test: PASS\n";
    return 0;
}
