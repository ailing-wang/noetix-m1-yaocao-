#pragma once

#include "dual_arm_local/Controller.h"

#include <cstddef>
#include <string>

namespace noetix::dual_arm_local
{
std::string jointName(std::size_t index);

PidGain scheduledPidGain(const JointEngineeringProfile &profile,
                         double angle_degrees, bool moving);

// Gain used by a selected joint whose target is unchanged while another
// selected joint is moving.  It provides gravity support without applying
// the full MOVE stiffness during the complete trajectory.
PidGain scheduledSupportPidGain(const JointEngineeringProfile &profile,
                                double angle_degrees);

bool pidScheduleRequiresMove(double current_degrees,
                             double target_degrees) noexcept;

bool loadJointConfiguration(const std::string &path,
                            ControllerOptions &options,
                            std::string &error);
} // namespace noetix::dual_arm_local
