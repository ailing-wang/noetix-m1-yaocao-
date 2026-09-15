#include "dual_arm_local/Profile.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace noetix::dual_arm_local
{
namespace
{
constexpr double kStepsPerRevolution = 4095.0;
constexpr double kSpeedRegisterStepsPerSecond = 50.0;

std::string trim(const std::string &text)
{
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return {};
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

bool parseJointIndex(const std::string &name, std::size_t &index)
{
    if (name.size() != 3 || (name[0] != 'L' && name[0] != 'R') ||
        name[1] != 'J' || name[2] < '1' || name[2] > '7')
        return false;
    index = (name[0] == 'L' ? 0U : kArmJointCount) +
            static_cast<std::size_t>(name[2] - '1');
    return true;
}

bool parseScheduleKey(const std::string &key, std::size_t &index)
{
    constexpr const char prefix[] = "pid_schedule.";
    const std::string prefix_text(prefix);
    return key.compare(0, prefix_text.size(), prefix_text) == 0 &&
           parseJointIndex(key.substr(prefix_text.size()), index);
}

bool parseSupportRatioKey(const std::string &key, std::size_t &index)
{
    constexpr const char prefix[] = "pid_support_ratio.";
    const std::string prefix_text(prefix);
    return key.compare(0, prefix_text.size(), prefix_text) == 0 &&
           parseJointIndex(key.substr(prefix_text.size()), index);
}

bool scheduleIsValid(const PidGainSchedule &schedule,
                     const JointEngineeringProfile &profile,
                     std::string &reason)
{
    if (!schedule.enabled)
        return true;
    for (std::size_t index = 0; index < schedule.knots.size(); ++index)
    {
        const PidScheduleKnot &knot = schedule.knots[index];
        if (!std::isfinite(knot.angle_degrees))
        {
            reason = "PID schedule angles must be finite";
            return false;
        }
        if (index != 0 &&
            knot.angle_degrees <= schedule.knots[index - 1].angle_degrees)
        {
            reason = "PID schedule angles must be strictly increasing";
            return false;
        }
        if (knot.move.p > 254 || knot.move.d > 254 || knot.move.i > 254 ||
            knot.hold.p > 254 || knot.hold.d > 254 || knot.hold.i > 254)
        {
            reason = "PID schedule gains must be 0..254";
            return false;
        }
        if (knot.hold.p > knot.move.p || knot.hold.d > knot.move.d ||
            knot.hold.i > knot.move.i)
        {
            reason = "each HOLD P/D must not exceed its MOVE P/D";
            return false;
        }
        if (index != 0)
        {
            const PidScheduleKnot &previous = schedule.knots[index - 1];
            if (knot.move.p < previous.move.p || knot.move.d < previous.move.d ||
                knot.move.i < previous.move.i)
            {
                reason = "PID schedule MOVE P/D/I must be nondecreasing with angle";
                return false;
            }
        }
    }
    if (profile.soft_limits_calibrated &&
        (schedule.knots.front().angle_degrees < profile.soft_min_degrees ||
         schedule.knots.back().angle_degrees > profile.soft_max_degrees))
    {
        reason = "PID schedule angles must remain inside calibrated soft limits";
        return false;
    }
    if (schedule.maximum_p_step == 0 || schedule.maximum_p_step > 8 ||
        schedule.maximum_d_step == 0 || schedule.maximum_d_step > 8 ||
        schedule.maximum_i_step == 0 || schedule.maximum_i_step > 8)
    {
        reason = "PID schedule P/D steps must be 1..8";
        return false;
    }
    if (schedule.step_interval < std::chrono::milliseconds(20) ||
        schedule.step_interval > std::chrono::milliseconds(1000))
    {
        reason = "PID schedule step interval must be 20..1000 ms";
        return false;
    }
    if (schedule.settle_delay < std::chrono::milliseconds(100) ||
        schedule.settle_delay > std::chrono::milliseconds(3000))
    {
        reason = "PID schedule settle delay must be 100..3000 ms";
        return false;
    }
    return true;
}

bool profileValuesAreValid(int direction, int zero_raw, int calibrated,
                           double soft_min, double soft_max,
                           double session_workspace, double maximum_delta,
                           double maximum_speed, int acceleration,
                           int speed, int torque, int position_p,
                           int position_d, int position_i,
                           PositionLimitPolicy position_policy,
                           TrajectoryExecutionPolicy trajectory_policy,
                           std::string &reason)
{
    if (direction != -1 && direction != 1)
        reason = "direction must be -1 or 1";
    else if (zero_raw < 0 || zero_raw > 4095)
        reason = "zero_raw must be 0..4095";
    else if (calibrated != 0 && calibrated != 1)
        reason = "calibrated must be 0 or 1";
    else if (!std::isfinite(soft_min) || !std::isfinite(soft_max) ||
             !std::isfinite(session_workspace) || !std::isfinite(maximum_delta) ||
             !std::isfinite(maximum_speed))
        reason = "floating-point fields must be finite";
    else if (soft_min >= soft_max)
        reason = "soft_min_deg must be less than soft_max_deg";
    else if (position_policy == PositionLimitPolicy::CalibratedSoftOnly &&
             calibrated != 1)
        reason = "calibrated-soft-only policy requires calibrated=1";
    else if (position_policy == PositionLimitPolicy::CalibratedSoftOnly &&
             (session_workspace != 0.0 || maximum_delta != 0.0))
        reason = "calibrated-soft-only policy requires session/max-delta fields to be 0";
    else if (position_policy == PositionLimitPolicy::SessionAndSoft &&
             (session_workspace <= 0.0 || session_workspace > 30.0))
        reason = "session_workspace_deg must be in (0,30]";
    else if (position_policy == PositionLimitPolicy::SessionAndSoft &&
             (maximum_delta <= 0.0 || maximum_delta > session_workspace))
        reason = "max_delta_deg must be in (0,session_workspace]";
    else if (maximum_speed <= 0.0 || maximum_speed > 450.0)
        reason = "max_speed_deg_s must be in (0,450]";
    else if (acceleration < 0 || acceleration > 254)
        reason = "acceleration_raw must be 0..254";
    else if (speed < 1 || speed > 254)
        reason = "speed_raw must be 1..254";
    else if (trajectory_policy == TrajectoryExecutionPolicy::DeviceProfiled &&
             static_cast<double>(speed) * kSpeedRegisterStepsPerSecond * 360.0 /
                     kStepsPerRevolution >
                 maximum_speed + 1e-9)
        reason = "device-profiled speed_raw exceeds max_speed_deg_s";
    else if (torque < 1 || torque > 1000)
        reason = "torque_limit_raw must be 1..1000";
    else if (position_p < 0 || position_p > 254)
        reason = "position_p must be 0..254";
    else if (position_d < 0 || position_d > 254)
        reason = "position_d must be 0..254";
    else if (position_i < 0 || position_i > 254)
        reason = "position_i must be 0..254";
    else if (calibrated == 1)
    {
        const long raw_a = std::lround(static_cast<double>(zero_raw) +
                                       soft_min * kStepsPerRevolution / 360.0 *
                                           static_cast<double>(direction));
        const long raw_b = std::lround(static_cast<double>(zero_raw) +
                                       soft_max * kStepsPerRevolution / 360.0 *
                                           static_cast<double>(direction));
        if (raw_a < 0 || raw_a > 4095 || raw_b < 0 || raw_b > 4095)
            reason = "calibrated soft limits convert outside raw 0..4095";
    }
    return reason.empty();
}
} // namespace

std::string jointName(std::size_t index)
{
    if (index >= kJointCount)
        return "INVALID";
    const char arm = index < kArmJointCount ? 'L' : 'R';
    return std::string(1, arm) + 'J' +
           std::to_string(index % kArmJointCount + 1);
}

PidGain scheduledPidGain(const JointEngineeringProfile &profile,
                         double angle_degrees, bool moving)
{
    if (!profile.pid_schedule.enabled)
        return {profile.position_p, profile.position_d, profile.position_i};
    const auto gain_at = [moving](const PidScheduleKnot &knot)
    {
        return moving ? knot.move : knot.hold;
    };
    const auto &knots = profile.pid_schedule.knots;
    if (angle_degrees <= knots.front().angle_degrees)
        return gain_at(knots.front());
    if (angle_degrees >= knots.back().angle_degrees)
        return gain_at(knots.back());
    for (std::size_t upper = 1; upper < knots.size(); ++upper)
    {
        if (angle_degrees > knots[upper].angle_degrees)
            continue;
        const PidScheduleKnot &low = knots[upper - 1];
        const PidScheduleKnot &high = knots[upper];
        const double x = (angle_degrees - low.angle_degrees) /
                         (high.angle_degrees - low.angle_degrees);
        const double smooth = x * x * (3.0 - 2.0 * x);
        const PidGain low_gain = gain_at(low);
        const PidGain high_gain = gain_at(high);
        return {
            static_cast<std::uint16_t>(std::lround(
                low_gain.p + (high_gain.p - low_gain.p) * smooth)),
            static_cast<std::uint16_t>(std::lround(
                low_gain.d + (high_gain.d - low_gain.d) * smooth)),
            static_cast<std::uint16_t>(std::lround(
                low_gain.i + (high_gain.i - low_gain.i) * smooth))};
    }
    return gain_at(knots.back());
}

PidGain scheduledSupportPidGain(const JointEngineeringProfile &profile,
                                double angle_degrees)
{
    const PidGain hold = scheduledPidGain(profile, angle_degrees, false);
    const PidGain move = scheduledPidGain(profile, angle_degrees, true);
    const unsigned int ratio = profile.pid_support_ratio_percent;
    const auto blend = [ratio](std::uint16_t low, std::uint16_t high)
    {
        return static_cast<std::uint16_t>(
            low + (static_cast<unsigned int>(high) - low) * ratio / 100U);
    };
    return {blend(hold.p, move.p),
            blend(hold.d, move.d),
            hold.i};
}

bool pidScheduleRequiresMove(double current_degrees,
                             double target_degrees) noexcept
{
    constexpr double tolerance_degrees =
        3.0 * 360.0 / kStepsPerRevolution;
    return std::isfinite(current_degrees) && std::isfinite(target_degrees) &&
           std::abs(target_degrees - current_degrees) > tolerance_degrees;
}

bool loadJointConfiguration(const std::string &path,
                            ControllerOptions &options,
                            std::string &error)
{
    error.clear();
    std::ifstream input(path);
    if (!input)
    {
        error = "cannot open joint configuration: " + path;
        return false;
    }

    ControllerOptions candidate = options;
    std::array<bool, kJointCount> seen{};
    std::array<bool, kJointCount> schedule_seen{};
    std::array<bool, kJointCount> support_ratio_seen{};
    bool version_seen = false;
    bool position_policy_seen = false;
    bool trajectory_policy_seen = false;
    int configuration_version = 0;
    std::string raw_line;
    std::size_t line_number = 0;
    while (std::getline(input, raw_line))
    {
        ++line_number;
        const std::size_t comment = raw_line.find('#');
        const std::string line = trim(raw_line.substr(0, comment));
        if (line.empty())
            continue;

        const std::size_t separator = line.find('=');
        if (separator == std::string::npos || line.find('=', separator + 1) != std::string::npos)
        {
            error = "joint configuration line " + std::to_string(line_number) +
                    " must contain exactly one '='";
            return false;
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string values = trim(line.substr(separator + 1));

        if (key == "version")
        {
            int version = 0;
            std::string extra;
            std::istringstream stream(values);
            if (version_seen || !(stream >> version) || (stream >> extra) ||
                (version != 2 && version != 3))
            {
                error = "joint configuration requires exactly one version=2 or version=3";
                return false;
            }
            version_seen = true;
            configuration_version = version;
            candidate.position_limit_policy = PositionLimitPolicy::SessionAndSoft;
            candidate.trajectory_execution_policy =
                TrajectoryExecutionPolicy::HostInterpolated;
            continue;
        }

        if (!version_seen)
        {
            error = "version must precede all other configuration fields";
            return false;
        }

        if (key == "position_policy")
        {
            if (configuration_version != 3 || position_policy_seen ||
                values != "calibrated_soft_only")
            {
                error = "version=3 requires exactly one "
                        "position_policy=calibrated_soft_only";
                return false;
            }
            candidate.position_limit_policy =
                PositionLimitPolicy::CalibratedSoftOnly;
            position_policy_seen = true;
            continue;
        }

        if (key == "trajectory_policy")
        {
            if (configuration_version != 3 || trajectory_policy_seen ||
                (values != "device_profiled" && values != "profiled"))
            {
                error = "version=3 requires exactly one "
                        "trajectory_policy=device_profiled";
                return false;
            }
            candidate.trajectory_execution_policy =
                values == "device_profiled"
                    ? TrajectoryExecutionPolicy::DeviceProfiled
                    : TrajectoryExecutionPolicy::HostInterpolated;
            trajectory_policy_seen = true;
            continue;
        }

        if (configuration_version == 3 &&
            (!position_policy_seen || !trajectory_policy_seen))
        {
            error = "version=3 policies must precede all joint rows";
            return false;
        }

        std::size_t support_index = 0;
        if (parseSupportRatioKey(key, support_index))
        {
            if (!seen[support_index])
            {
                error = "PID support ratio must follow joint row for " +
                        jointName(support_index);
                return false;
            }
            int ratio = -1;
            std::string extra;
            std::istringstream stream(values);
            if (support_ratio_seen[support_index] || !(stream >> ratio) ||
                (stream >> extra) || ratio < 0 || ratio > 100)
            {
                error = "PID support ratio must be one integer in 0..100 for " +
                        jointName(support_index);
                return false;
            }
            candidate.engineering_profiles[support_index]
                .pid_support_ratio_percent = static_cast<std::uint16_t>(ratio);
            support_ratio_seen[support_index] = true;
            continue;
        }

        std::size_t schedule_index = 0;
        if (parseScheduleKey(key, schedule_index))
        {
            if (!seen[schedule_index])
            {
                error = "PID schedule must follow joint row for " +
                        jointName(schedule_index);
                return false;
            }
            if (schedule_seen[schedule_index])
            {
                error = "duplicate PID schedule for " + jointName(schedule_index);
                return false;
            }
            PidGainSchedule schedule;
            schedule.enabled = true;
            JointEngineeringProfile &profile =
                candidate.engineering_profiles[schedule_index];
            std::istringstream stream(values);
            std::vector<double> fields;
            double field = 0.0;
            while (stream >> field)
                fields.push_back(field);
            if (!stream.eof() || (fields.size() != 19 && fields.size() != 26))
            {
                error = "invalid PID schedule field count for " +
                        jointName(schedule_index);
                return false;
            }
            const bool includes_i = fields.size() == 26;
            const std::size_t knot_stride = includes_i ? 7U : 5U;
            for (std::size_t field_index = 0; field_index < fields.size();
                 ++field_index)
            {
                const bool angle_field =
                    field_index < schedule.knots.size() * knot_stride &&
                    field_index % knot_stride == 0;
                const double value = fields[field_index];
                if (!std::isfinite(value) ||
                    (!angle_field &&
                     (std::trunc(value) != value || value < 0.0 ||
                      value > static_cast<double>(std::numeric_limits<int>::max()))))
                {
                    error = "PID schedule gains/timing must be finite integers for " +
                            jointName(schedule_index);
                    return false;
                }
            }
            std::size_t cursor = 0;
            for (PidScheduleKnot &knot : schedule.knots)
            {
                knot.angle_degrees = fields[cursor++];
                const int move_p = static_cast<int>(fields[cursor++]);
                const int move_d = static_cast<int>(fields[cursor++]);
                const int move_i = includes_i
                                       ? static_cast<int>(fields[cursor++])
                                       : profile.position_i;
                const int hold_p = static_cast<int>(fields[cursor++]);
                const int hold_d = static_cast<int>(fields[cursor++]);
                const int hold_i = includes_i
                                       ? static_cast<int>(fields[cursor++])
                                       : profile.position_i;
                if (move_p < 0 || move_p > 254 || move_d < 0 || move_d > 254 ||
                    move_i < 0 || move_i > 254 ||
                    hold_p < 0 || hold_p > 254 || hold_d < 0 || hold_d > 254 ||
                    hold_i < 0 || hold_i > 254)
                {
                    error = "PID schedule gains must be nonnegative for " +
                            jointName(schedule_index);
                    return false;
                }
                knot.move = {static_cast<std::uint16_t>(move_p),
                             static_cast<std::uint16_t>(move_d),
                             static_cast<std::uint16_t>(move_i)};
                knot.hold = {static_cast<std::uint16_t>(hold_p),
                             static_cast<std::uint16_t>(hold_d),
                             static_cast<std::uint16_t>(hold_i)};
            }
            const int p_step = static_cast<int>(fields[cursor++]);
            const int d_step = static_cast<int>(fields[cursor++]);
            const int i_step = includes_i ? static_cast<int>(fields[cursor++]) : 1;
            const int step_interval_ms = static_cast<int>(fields[cursor++]);
            const int settle_delay_ms = static_cast<int>(fields[cursor++]);
            if (cursor != fields.size() || p_step < 0 || d_step < 0 || i_step < 0 ||
                step_interval_ms < 0 || settle_delay_ms < 0)
            {
                error = "invalid PID schedule tail for " +
                        jointName(schedule_index);
                return false;
            }
            schedule.maximum_p_step = static_cast<std::uint16_t>(p_step);
            schedule.maximum_d_step = static_cast<std::uint16_t>(d_step);
            schedule.maximum_i_step = static_cast<std::uint16_t>(i_step);
            schedule.step_interval = std::chrono::milliseconds(step_interval_ms);
            schedule.settle_delay = std::chrono::milliseconds(settle_delay_ms);
            std::string reason;
            if (!scheduleIsValid(schedule, profile, reason))
            {
                error = jointName(schedule_index) + " PID schedule: " + reason;
                return false;
            }
            profile.pid_schedule = schedule;
            schedule_seen[schedule_index] = true;
            continue;
        }

        std::size_t index = 0;
        if (!parseJointIndex(key, index))
        {
            error = "unknown joint key on line " + std::to_string(line_number) +
                    ": " + key;
            return false;
        }
        if (seen[index])
        {
            error = "duplicate joint " + key + " on line " +
                    std::to_string(line_number);
            return false;
        }

        int direction = 0;
        int zero_raw = 0;
        int calibrated = 0;
        double soft_min = 0.0;
        double soft_max = 0.0;
        double session_workspace = 0.0;
        double maximum_delta = 0.0;
        double maximum_speed = 0.0;
        int acceleration = 0;
        int speed = 0;
        int torque = 0;
        int position_p = 0;
        int position_d = 0;
        int position_i = 0;
        std::string extra;
        std::istringstream stream(values);
        if (!(stream >> direction >> zero_raw >> calibrated >> soft_min >> soft_max >>
              session_workspace >> maximum_delta >> maximum_speed >> acceleration >>
              speed >> torque >> position_p >> position_d >> position_i) ||
            (stream >> extra))
        {
            error = "invalid field count or type for " + key + " on line " +
                    std::to_string(line_number);
            return false;
        }

        std::string reason;
        if (!profileValuesAreValid(direction, zero_raw, calibrated, soft_min,
                                   soft_max, session_workspace, maximum_delta,
                                   maximum_speed, acceleration, speed, torque,
                                   position_p, position_d, position_i,
                                   candidate.position_limit_policy,
                                   candidate.trajectory_execution_policy, reason))
        {
            error = key + " on line " + std::to_string(line_number) + ": " + reason;
            return false;
        }

        candidate.directions[index] = direction;
        JointEngineeringProfile &profile = candidate.engineering_profiles[index];
        profile.zero_raw = zero_raw;
        profile.soft_limits_calibrated = calibrated == 1;
        profile.soft_min_degrees = soft_min;
        profile.soft_max_degrees = soft_max;
        profile.session_workspace_degrees = session_workspace;
        profile.maximum_command_delta_degrees = maximum_delta;
        profile.maximum_trajectory_speed_degrees_per_second = maximum_speed;
        profile.position_p = static_cast<std::uint16_t>(position_p);
        profile.position_d = static_cast<std::uint16_t>(position_d);
        profile.position_i = static_cast<std::uint16_t>(position_i);
        candidate.safe_acceleration[index] = static_cast<std::uint16_t>(acceleration);
        candidate.safe_speed[index] = static_cast<std::uint16_t>(speed);
        candidate.safe_torque_limit[index] = static_cast<std::uint16_t>(torque);
        seen[index] = true;
    }

    if (input.bad())
    {
        error = "failed while reading joint configuration: " + path;
        return false;
    }

    if (!version_seen)
    {
        error = "joint configuration is missing version=2 or version=3";
        return false;
    }
    if (configuration_version == 3 && !position_policy_seen)
    {
        error = "version=3 is missing position_policy=calibrated_soft_only";
        return false;
    }
    if (configuration_version == 3 && !trajectory_policy_seen)
    {
        error = "version=3 is missing trajectory_policy=device_profiled";
        return false;
    }
    for (std::size_t index = 0; index < seen.size(); ++index)
    {
        if (!seen[index])
        {
            error = "joint configuration is missing " + jointName(index);
            return false;
        }
    }

    options = std::move(candidate);
    return true;
}
} // namespace noetix::dual_arm_local
