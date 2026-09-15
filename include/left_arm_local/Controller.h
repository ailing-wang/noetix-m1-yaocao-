#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace noetix::left_arm_local
{
constexpr std::size_t kJointCount = 7;

enum class Mode
{
    Starting,
    Observe,
    Control,
    Fault,
    Stopped
};

enum class ResultCode
{
    Completed,
    Rejected,
    Expired,
    Cancelled,
    Faulted
};

struct JointSnapshot
{
    int joint = 0;
    int slave_id = 0;
    int direction = 1;
    int raw_position = 0;
    int raw_target = 0;
    double absolute_degrees = 0.0;
    double target_degrees = 0.0;
    std::uint16_t status_word = 0;
    int speed_raw = 0;
    double voltage_v = 0.0;
    double temperature_c = 0.0;
    double current_a = 0.0;
    bool moving = false;
    bool torque_enabled = false;
};

struct Snapshot
{
    Mode mode = Mode::Starting;
    bool initialized = false;
    bool fault_latched = false;
    std::string fault_reason;
    std::int64_t monotonic_timestamp_ms = 0;
    std::int64_t control_lease_remaining_ms = 0;
    std::uint64_t last_submitted_sequence = 0;
    std::uint64_t active_sequence = 0;
    std::uint64_t last_result_sequence = 0;
    ResultCode last_result_code = ResultCode::Completed;
    std::string last_result_message;
    std::size_t queue_depth = 0;
    long last_command_span_ms = 0;
    std::array<JointSnapshot, kJointCount> joints{};
};

struct CommandResult
{
    std::uint64_t sequence = 0;
    ResultCode code = ResultCode::Rejected;
    std::string message;
    std::int64_t finished_at_ms = 0;
};

struct SubmitResult
{
    bool accepted = false;
    std::uint64_t sequence = 0;
    std::int64_t accepted_at_ms = 0;
    std::int64_t deadline_at_ms = 0;
    std::string message;
};

struct ControllerOptions
{
    std::array<int, kJointCount> directions{{1, -1, 1, -1, 1, 1, 1}};
    std::string l1_device = "/dev/noetix_arm_l1";
    std::string l2_device = "/dev/noetix_arm_l2";
    std::string startup_service = "startup.service";
    bool require_startup_inactive = true;

    double session_workspace_degrees = 3.0;
    double maximum_command_delta_degrees = 1.0;
    double maximum_trajectory_speed_degrees_per_second = 5.0;
    std::chrono::milliseconds trajectory_tick{100};
    std::chrono::milliseconds observe_period{500};
    std::chrono::milliseconds control_period{250};
    std::size_t maximum_queue_depth = 8;

    std::uint16_t safe_acceleration = 2;
    std::uint16_t safe_speed = 2;
    std::uint16_t safe_torque_limit = 100;
};

class Controller
{
public:
    explicit Controller(ControllerOptions options);
    ~Controller();

    Controller(const Controller &) = delete;
    Controller &operator=(const Controller &) = delete;

    bool start(std::string &error);
    void shutdown();

    SubmitResult submitControl(std::uint64_t sequence,
                               std::chrono::milliseconds ttl);
    SubmitResult submitTrajectory(
        std::uint64_t sequence,
        std::chrono::milliseconds ttl,
        std::chrono::milliseconds duration,
        const std::array<double, kJointCount> &absolute_degrees);
    SubmitResult submitHold(std::uint64_t sequence,
                            std::chrono::milliseconds ttl);
    SubmitResult submitHeartbeat(std::uint64_t sequence,
                                 std::chrono::milliseconds ttl);
    SubmitResult submitStop(std::uint64_t sequence);
    SubmitResult submitFadeStop(std::uint64_t sequence,
                                std::chrono::milliseconds fade);
    SubmitResult submitReset(std::uint64_t sequence,
                             std::chrono::milliseconds ttl);

    Snapshot snapshot() const;
    bool waitForResult(std::uint64_t sequence,
                       std::chrono::milliseconds timeout,
                       CommandResult &result) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

const char *toString(Mode mode) noexcept;
const char *toString(ResultCode code) noexcept;
std::int64_t monotonicMilliseconds() noexcept;
double rawToAbsoluteDegrees(int raw, int direction) noexcept;
int absoluteDegreesToRaw(double degrees, int direction) noexcept;
} // namespace noetix::left_arm_local
