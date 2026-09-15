#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace noetix::dual_arm_local
{
constexpr std::size_t kArmJointCount = 7;
constexpr std::size_t kJointCount = 14;
using JointMask = std::uint16_t;

constexpr JointMask kLeftArmMask = 0x007F;
constexpr JointMask kRightArmMask = 0x3F80;
constexpr JointMask kBothArmsMask = kLeftArmMask | kRightArmMask;

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

enum class PositionLimitPolicy
{
    SessionAndSoft,
    CalibratedSoftOnly
};

enum class TrajectoryExecutionPolicy
{
    HostInterpolated,
    DeviceProfiled
};

struct JointSnapshot
{
    int joint = 0;
    int arm_joint = 0;
    char arm = 'L';
    int bus_index = 0;
    int slave_id = 0;
    int direction = 1;
    int raw_position = 0;
    int raw_target = 0;
    int tracking_error_steps = 0;
    int zero_raw = 2048;
    double absolute_degrees = 0.0;
    double target_degrees = 0.0;
    double tracking_error_degrees = 0.0;
    std::uint16_t status_word = 0;
    int speed_raw = 0;
    int pwm_raw = 0;
    std::uint16_t acceleration_raw = 0;
    std::uint16_t speed_limit_raw = 0;
    std::uint16_t torque_limit_raw = 0;
    std::uint16_t position_p_raw = 0;
    std::uint16_t position_d_raw = 0;
    std::uint16_t position_i_raw = 0;
    std::uint16_t pid_lock_raw = 0;
    double voltage_v = 0.0;
    double temperature_c = 0.0;
    double current_a = 0.0;
    bool soft_limits_calibrated = false;
    double soft_min_degrees = -180.0;
    double soft_max_degrees = 180.0;
    double session_workspace_degrees = 3.0;
    double maximum_command_delta_degrees = 1.0;
    double maximum_trajectory_speed_degrees_per_second = 5.0;
    bool moving = false;
    bool torque_enabled = false;
    bool selected = false;
};

struct Snapshot
{
    Mode mode = Mode::Starting;
    bool initialized = false;
    bool fault_latched = false;
    std::string fault_reason;
    JointMask active_mask = 0;
    std::int64_t monotonic_timestamp_ms = 0;
    std::int64_t control_lease_remaining_ms = 0;
    std::uint64_t last_submitted_sequence = 0;
    std::uint64_t active_sequence = 0;
    std::uint64_t last_result_sequence = 0;
    ResultCode last_result_code = ResultCode::Completed;
    std::string last_result_message;
    std::size_t queue_depth = 0;
    long last_command_span_ms = 0;
    PositionLimitPolicy position_limit_policy =
        PositionLimitPolicy::SessionAndSoft;
    TrajectoryExecutionPolicy trajectory_execution_policy =
        TrajectoryExecutionPolicy::HostInterpolated;
    std::array<JointSnapshot, kJointCount> joints{};
};

struct PidGain
{
    std::uint16_t p = 32;
    std::uint16_t d = 32;
    std::uint16_t i = 0;
};

struct PidScheduleKnot
{
    double angle_degrees = 0.0;
    PidGain move{};
    PidGain hold{};
};

constexpr std::size_t kPidScheduleKnotCount = 3;

struct PidGainSchedule
{
    bool enabled = false;
    std::array<PidScheduleKnot, kPidScheduleKnotCount> knots{};
    std::uint16_t maximum_p_step = 2;
    std::uint16_t maximum_d_step = 2;
    std::uint16_t maximum_i_step = 1;
    std::chrono::milliseconds step_interval{50};
    std::chrono::milliseconds settle_delay{400};
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

struct JointEngineeringProfile
{
    int zero_raw = 2048;
    bool soft_limits_calibrated = false;
    double soft_min_degrees = -180.0;
    double soft_max_degrees = 180.0;
    double session_workspace_degrees = 3.0;
    double maximum_command_delta_degrees = 1.0;
    double maximum_trajectory_speed_degrees_per_second = 5.0;
    std::uint16_t position_p = 32;
    std::uint16_t position_d = 32;
    std::uint16_t position_i = 0;
    std::uint16_t pid_support_ratio_percent = 50;
    PidGainSchedule pid_schedule{};
};

struct ControllerOptions
{
    std::array<int, kJointCount> directions{{
        1, -1, 1, -1, 1, 1, 1,
        1, -1, 1, -1, 1, 1, 1}};
    std::array<std::string, 4> devices{{
        "/dev/noetix_arm_l1", "/dev/noetix_arm_l2",
        "/dev/noetix_arm_r1", "/dev/noetix_arm_r2"}};
    std::string startup_service = "startup.service";
    bool require_startup_inactive = true;
    // Recovery-only capability used by the standalone legacy-zero tool. The
    // normal local controller never enables this command.
    bool allow_legacy_zero_command = false;
    PositionLimitPolicy position_limit_policy =
        PositionLimitPolicy::SessionAndSoft;
    TrajectoryExecutionPolicy trajectory_execution_policy =
        TrajectoryExecutionPolicy::HostInterpolated;

    std::chrono::milliseconds trajectory_tick{200};
    std::chrono::milliseconds observe_period{1000};
    std::chrono::milliseconds control_period{500};
    // High-frequency joint position refresh: one dedicated thread per bus
    // publishes the latest raw position / status word while the slow
    // control loop only snapshots the full state (yaocao-style per-bus
    // reader threads).  0 disables the fast path.
    std::chrono::milliseconds status_period{4};
    std::size_t maximum_queue_depth = 8;

    std::array<std::uint16_t, kJointCount> safe_acceleration{{
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}};
    std::array<std::uint16_t, kJointCount> safe_speed{{
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}};
    std::array<std::uint16_t, kJointCount> safe_torque_limit{{
        100, 100, 100, 100, 100, 100, 100,
        100, 100, 100, 100, 100, 100, 100}};
    std::array<JointEngineeringProfile, kJointCount> engineering_profiles{};
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
                               std::chrono::milliseconds ttl,
                               JointMask selection);
    SubmitResult submitTrajectory(
        std::uint64_t sequence,
        std::chrono::milliseconds ttl,
        std::chrono::milliseconds duration,
        const std::array<double, kJointCount> &absolute_degrees);
    SubmitResult submitStreamTarget(
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
    SubmitResult submitLegacyZero(std::uint64_t sequence,
                                  std::chrono::milliseconds ttl);

    Snapshot snapshot() const;
    bool waitForResult(std::uint64_t sequence,
                       std::chrono::milliseconds timeout,
                       CommandResult &result) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

constexpr bool jointSelected(JointMask mask, std::size_t index) noexcept
{
    return index < kJointCount &&
           (mask & static_cast<JointMask>(JointMask{1} << index)) != 0;
}

const char *toString(Mode mode) noexcept;
const char *toString(ResultCode code) noexcept;
const char *toString(PositionLimitPolicy policy) noexcept;
const char *toString(TrajectoryExecutionPolicy policy) noexcept;
std::int64_t monotonicMilliseconds() noexcept;
double rawToAbsoluteDegrees(int raw, int direction) noexcept;
int absoluteDegreesToRaw(double degrees, int direction) noexcept;
} // namespace noetix::dual_arm_local
