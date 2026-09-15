#include "left_arm_local/Controller.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

extern "C" void fake_modbus_reset();
extern "C" void fake_modbus_disconnect_bus(int bus_index);
extern "C" void fake_modbus_set_temperature(int slave, int temperature);
extern "C" int fake_modbus_torque(int slave);

namespace
{
using namespace std::chrono_literals;
using noetix::left_arm_local::CommandResult;
using noetix::left_arm_local::Controller;
using noetix::left_arm_local::ControllerOptions;
using noetix::left_arm_local::Mode;
using noetix::left_arm_local::ResultCode;
using noetix::left_arm_local::SubmitResult;
using noetix::left_arm_local::kJointCount;

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
                         std::chrono::milliseconds timeout = 5s)
{
    CommandResult result;
    require(controller.waitForResult(sequence, timeout, result),
            "timed out waiting for sequence " + std::to_string(sequence));
    return result;
}

void requireCompleted(Controller &controller, const SubmitResult &submission,
                      std::chrono::milliseconds timeout = 5s)
{
    require(submission.accepted, "submission rejected: " + submission.message);
    const CommandResult result = waitResult(controller, submission.sequence, timeout);
    require(result.code == ResultCode::Completed,
            "sequence " + std::to_string(result.sequence) + " ended " +
                noetix::left_arm_local::toString(result.code) + ": " + result.message);
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

void requireAllTorque(bool expected)
{
    for (int id = 1; id <= 7; ++id)
        require(fake_modbus_torque(id) == (expected ? 1 : 0),
                "unexpected torque state for ID " + std::to_string(id));
}
} // namespace

int main()
{
    fake_modbus_reset();
    ControllerOptions options;
    options.require_startup_inactive = false;
    options.observe_period = 50ms;
    options.control_period = 50ms;
    options.trajectory_tick = 50ms;
    options.maximum_queue_depth = 2;

    Controller controller(options);
    std::string error;
    require(controller.start(error), "start failed: " + error);
    require(controller.snapshot().mode == Mode::Observe, "startup mode is not OBSERVE");
    requireAllTorque(false);

    requireCompleted(controller, controller.submitControl(1, 5s));
    require(controller.snapshot().mode == Mode::Control, "CONTROL mode was not entered");
    requireAllTorque(true);

    const SubmitResult stale = controller.submitHeartbeat(1, 5s);
    require(!stale.accepted, "duplicate sequence was accepted");

    auto initial = controller.snapshot();
    std::array<double, kJointCount> targets{};
    for (std::size_t index = 0; index < targets.size(); ++index)
        targets[index] = initial.joints[index].absolute_degrees + 2.0;
    const SubmitResult unsafe_target = controller.submitTrajectory(2, 3s, 500ms, targets);
    require(unsafe_target.accepted, "unsafe target did not reach owner-thread validation");
    require(waitResult(controller, 2).code == ResultCode::Rejected,
            "unsafe absolute delta was not rejected");

    for (std::size_t index = 0; index < targets.size(); ++index)
        targets[index] = initial.joints[index].absolute_degrees + 0.4;
    requireCompleted(controller, controller.submitTrajectory(3, 3s, 500ms, targets));
    const auto reached = controller.snapshot();
    for (std::size_t index = 0; index < targets.size(); ++index)
        require(std::abs(reached.joints[index].target_degrees - targets[index]) < 0.1,
                "absolute target rounding too large at J" + std::to_string(index + 1));

    requireCompleted(controller, controller.submitHold(4, 3s));
    for (std::size_t index = 0; index < targets.size(); ++index)
        targets[index] = initial.joints[index].absolute_degrees;
    requireCompleted(controller, controller.submitTrajectory(5, 3s, 500ms, targets));
    const auto returned = controller.snapshot();
    for (std::size_t index = 0; index < targets.size(); ++index)
        require(returned.joints[index].raw_target == initial.joints[index].raw_position,
                "absolute return target drifted at J" + std::to_string(index + 1));

    requireCompleted(controller, controller.submitStop(6));
    require(controller.snapshot().mode == Mode::Observe, "STOP did not return OBSERVE");
    requireAllTorque(false);
    const SubmitResult stale_stop = controller.submitStop(6);
    require(stale_stop.accepted,
            "stale STOP did not enter the out-of-band safety path");
    require(stale_stop.message.find("out-of-band safety STOP") != std::string::npos,
            "stale STOP did not explain its unsequenced safety semantics");
    std::this_thread::sleep_for(250ms);
    require(controller.snapshot().mode == Mode::Observe,
            "out-of-band stale STOP changed the safe mode");
    requireAllTorque(false);

    requireCompleted(controller, controller.submitControl(7, 10s));
    const auto before_long = controller.snapshot();
    for (std::size_t index = 0; index < targets.size(); ++index)
        targets[index] = before_long.joints[index].absolute_degrees + 0.8;
    const SubmitResult long_trajectory = controller.submitTrajectory(8, 6s, 4s, targets);
    require(long_trajectory.accepted, "long trajectory was rejected");
    std::this_thread::sleep_for(150ms);
    require(controller.submitHeartbeat(9, 5s).accepted, "first queued heartbeat rejected");
    require(controller.submitHeartbeat(10, 5s).accepted, "second queued heartbeat rejected");
    require(!controller.submitHeartbeat(11, 5s).accepted,
            "command queue accepted more than configured capacity");
    requireCompleted(controller, controller.submitStop(12));
    const CommandResult interrupted = waitResult(controller, 8);
    require(interrupted.code == ResultCode::Cancelled,
            "active trajectory was not cancelled by STOP");
    require(waitResult(controller, 9).code == ResultCode::Cancelled,
            "queued command 9 was not cancelled by STOP");
    require(waitResult(controller, 10).code == ResultCode::Cancelled,
            "queued command 10 was not cancelled by STOP");
    requireAllTorque(false);

    requireCompleted(controller, controller.submitControl(13, 1s));
    require(waitForMode(controller, Mode::Fault, 2s),
            "control lease expiry did not latch FAULT");
    requireAllTorque(false);
    requireCompleted(controller, controller.submitReset(14, 2s));
    require(controller.snapshot().mode == Mode::Observe, "RESET did not restore OBSERVE");

    requireCompleted(controller, controller.submitControl(15, 5s));
    fake_modbus_set_temperature(3, 70);
    require(waitForMode(controller, Mode::Fault, 2s),
            "temperature fault did not latch FAULT");
    requireAllTorque(false);
    fake_modbus_set_temperature(3, 38);
    requireCompleted(controller, controller.submitReset(16, 2s));

    requireCompleted(controller, controller.submitControl(17, 5s));
    fake_modbus_disconnect_bus(1);
    require(waitForMode(controller, Mode::Fault, 2s),
            "bus disconnect did not latch FAULT");
    const SubmitResult reset_after_disconnect = controller.submitReset(18, 2s);
    require(reset_after_disconnect.accepted, "disconnect RESET submission rejected too early");
    const CommandResult reset_result = waitResult(controller, 18);
    require(reset_result.code == ResultCode::Rejected,
            "terminal communication failure RESET was not rejected");

    controller.shutdown();
    std::cout << "left_arm_local_service_test: PASS\n";
    return 0;
}
