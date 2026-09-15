#include "dual_arm_local/Calibration.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace noetix::dual_arm_local
{
namespace
{
constexpr double kStepsPerRevolution = 4095.0;
constexpr double kMinimumMarginDegrees = 0.5;
constexpr double kMaximumMarginDegrees = 30.0;

std::string snapshotJointName(const JointSnapshot &joint)
{
    if ((joint.arm != 'L' && joint.arm != 'R') ||
        joint.arm_joint < 1 || joint.arm_joint > 7)
        return "INVALID";
    return std::string(1, joint.arm) + 'J' + std::to_string(joint.arm_joint);
}

bool jointSnapshotIsUsable(const JointSnapshot &joint, std::string &error)
{
    if (snapshotJointName(joint) == "INVALID")
        error = "invalid joint identity";
    else if (joint.direction != -1 && joint.direction != 1)
        error = "joint direction must be -1 or 1";
    else if (joint.zero_raw < 0 || joint.zero_raw > 4095)
        error = "zero_raw must be 0..4095";
    else if (joint.raw_position < 0 || joint.raw_position > 4095)
        error = "raw position must be 0..4095";
    else if (!std::isfinite(joint.absolute_degrees))
        error = "joint angle must be finite";
    return error.empty();
}

bool rawLimitIsValid(const JointSnapshot &joint, double degrees)
{
    const long raw = std::lround(
        static_cast<double>(joint.zero_raw) +
        degrees * kStepsPerRevolution / 360.0 *
            static_cast<double>(joint.direction));
    return raw >= 0 && raw <= 4095;
}
} // namespace

bool captureSoftLimit(const JointSnapshot &joint,
                      SoftLimitEndpoint endpoint,
                      double margin_degrees,
                      JointLimitDraft &draft,
                      std::string &error)
{
    error.clear();
    if (!jointSnapshotIsUsable(joint, error))
        return false;
    if (!std::isfinite(margin_degrees) ||
        margin_degrees < kMinimumMarginDegrees ||
        margin_degrees > kMaximumMarginDegrees)
    {
        error = "safety margin must be finite and in [0.5,30] degrees";
        return false;
    }

    SoftLimitCapture capture;
    capture.captured = true;
    capture.raw_position = joint.raw_position;
    capture.measured_degrees = joint.absolute_degrees;
    capture.margin_degrees = margin_degrees;
    capture.proposed_degrees =
        endpoint == SoftLimitEndpoint::Minimum
            ? capture.measured_degrees + margin_degrees
            : capture.measured_degrees - margin_degrees;

    if (!rawLimitIsValid(joint, capture.proposed_degrees))
    {
        error = "proposed soft limit converts outside raw 0..4095";
        return false;
    }

    JointLimitDraft candidate = draft;
    if (endpoint == SoftLimitEndpoint::Minimum)
        candidate.minimum = capture;
    else
        candidate.maximum = capture;

    if (candidate.minimum.captured && candidate.maximum.captured &&
        candidate.minimum.proposed_degrees >= candidate.maximum.proposed_degrees)
    {
        error = "captured endpoints and margins leave no valid soft-limit range";
        return false;
    }

    draft = candidate;
    return true;
}

bool updateJointExtrema(const JointSnapshot &joint,
                        JointExtrema &extrema,
                        std::string &error)
{
    error.clear();
    if (!jointSnapshotIsUsable(joint, error))
        return false;

    if (!extrema.observed)
    {
        extrema.observed = true;
        extrema.samples = 1;
        extrema.minimum_raw = joint.raw_position;
        extrema.maximum_raw = joint.raw_position;
        extrema.minimum_degrees = joint.absolute_degrees;
        extrema.maximum_degrees = joint.absolute_degrees;
        return true;
    }

    ++extrema.samples;
    if (joint.absolute_degrees < extrema.minimum_degrees)
    {
        extrema.minimum_degrees = joint.absolute_degrees;
        extrema.minimum_raw = joint.raw_position;
    }
    if (joint.absolute_degrees > extrema.maximum_degrees)
    {
        extrema.maximum_degrees = joint.absolute_degrees;
        extrema.maximum_raw = joint.raw_position;
    }
    return true;
}

bool draftSoftLimitsFromExtrema(const JointSnapshot &joint,
                                const JointExtrema &extrema,
                                double margin_degrees,
                                double minimum_usable_span_degrees,
                                JointLimitDraft &draft,
                                std::string &error)
{
    error.clear();
    if (!jointSnapshotIsUsable(joint, error))
        return false;
    if (!extrema.observed || extrema.samples < 2)
    {
        error = "at least two recorded samples are required";
        return false;
    }
    if (!std::isfinite(minimum_usable_span_degrees) ||
        minimum_usable_span_degrees <= 0.0)
    {
        error = "minimum usable span must be finite and positive";
        return false;
    }
    const double measured_span =
        extrema.maximum_degrees - extrema.minimum_degrees;
    if (!std::isfinite(measured_span) ||
        measured_span < 2.0 * margin_degrees + minimum_usable_span_degrees)
    {
        std::ostringstream message;
        message << "recorded span " << measured_span
                << " deg is too small for two margins plus "
                << minimum_usable_span_degrees << " deg usable range";
        error = message.str();
        return false;
    }

    JointSnapshot endpoint = joint;
    JointLimitDraft candidate;
    endpoint.raw_position = extrema.minimum_raw;
    endpoint.absolute_degrees = extrema.minimum_degrees;
    if (!captureSoftLimit(endpoint, SoftLimitEndpoint::Minimum,
                          margin_degrees, candidate, error))
        return false;
    endpoint.raw_position = extrema.maximum_raw;
    endpoint.absolute_degrees = extrema.maximum_degrees;
    if (!captureSoftLimit(endpoint, SoftLimitEndpoint::Maximum,
                          margin_degrees, candidate, error))
        return false;

    std::string ignored_line;
    if (!formatCalibratedJointLine(joint, candidate, ignored_line, error))
        return false;
    draft = candidate;
    return true;
}

bool formatCalibratedJointLine(const JointSnapshot &joint,
                               const JointLimitDraft &draft,
                               std::string &line,
                               std::string &error)
{
    line.clear();
    error.clear();
    if (!jointSnapshotIsUsable(joint, error))
        return false;
    if (!draft.minimum.captured || !draft.maximum.captured)
    {
        error = "both MIN and MAX endpoints must be captured";
        return false;
    }

    const double soft_min = draft.minimum.proposed_degrees;
    const double soft_max = draft.maximum.proposed_degrees;
    if (!std::isfinite(soft_min) || !std::isfinite(soft_max) ||
        soft_min >= soft_max)
    {
        error = "candidate soft-limit range is invalid";
        return false;
    }
    if (soft_min > 0.0 || soft_max < 0.0)
    {
        error = "candidate soft-limit range must contain configured zero (0 degrees)";
        return false;
    }
    if (!rawLimitIsValid(joint, soft_min) ||
        !rawLimitIsValid(joint, soft_max))
    {
        error = "candidate soft limits convert outside raw 0..4095";
        return false;
    }

    std::ostringstream output;
    output << snapshotJointName(joint) << '=' << joint.direction
           << ' ' << joint.zero_raw
           << " 1 " << std::fixed << std::setprecision(6)
           << soft_min << ' ' << soft_max
           << ' ' << joint.session_workspace_degrees
           << ' ' << joint.maximum_command_delta_degrees
           << ' ' << joint.maximum_trajectory_speed_degrees_per_second
           << ' ' << joint.acceleration_raw
           << ' ' << joint.speed_limit_raw
           << ' ' << joint.torque_limit_raw
           << ' ' << joint.position_p_raw
           << ' ' << joint.position_d_raw
           << ' ' << joint.position_i_raw;
    line = output.str();
    return true;
}
} // namespace noetix::dual_arm_local
