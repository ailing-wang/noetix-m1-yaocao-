#include "dual_arm_local/Calibration.h"

#include <cassert>
#include <iostream>
#include <string>

int main()
{
    using noetix::dual_arm_local::JointLimitDraft;
    using noetix::dual_arm_local::JointSnapshot;
    using noetix::dual_arm_local::JointExtrema;
    using noetix::dual_arm_local::SoftLimitEndpoint;
    using noetix::dual_arm_local::captureSoftLimit;
    using noetix::dual_arm_local::draftSoftLimitsFromExtrema;
    using noetix::dual_arm_local::formatCalibratedJointLine;
    using noetix::dual_arm_local::updateJointExtrema;

    JointSnapshot joint;
    joint.arm = 'R';
    joint.arm_joint = 7;
    joint.direction = 1;
    joint.zero_raw = 2048;
    joint.raw_position = 1024;
    joint.absolute_degrees = -90.0;
    joint.session_workspace_degrees = 3.0;
    joint.maximum_command_delta_degrees = 1.0;
    joint.maximum_trajectory_speed_degrees_per_second = 5.0;
    joint.acceleration_raw = 2;
    joint.speed_limit_raw = 2;
    joint.torque_limit_raw = 100;
    joint.position_p_raw = 32;
    joint.position_d_raw = 32;
    joint.position_i_raw = 0;

    JointLimitDraft draft;
    std::string error;
    assert(!captureSoftLimit(joint, SoftLimitEndpoint::Minimum,
                             0.49, draft, error));
    assert(!error.empty());
    assert(!draft.minimum.captured);

    error.clear();
    assert(captureSoftLimit(joint, SoftLimitEndpoint::Minimum,
                            5.0, draft, error));
    assert(error.empty());
    assert(draft.minimum.captured);
    assert(draft.minimum.raw_position == 1024);
    assert(draft.minimum.proposed_degrees == -85.0);

    std::string line;
    assert(!formatCalibratedJointLine(joint, draft, line, error));
    assert(line.empty());

    joint.raw_position = 3072;
    joint.absolute_degrees = 90.0;
    error.clear();
    assert(captureSoftLimit(joint, SoftLimitEndpoint::Maximum,
                            5.0, draft, error));
    assert(draft.maximum.proposed_degrees == 85.0);
    assert(formatCalibratedJointLine(joint, draft, line, error));
    assert(line == "RJ7=1 2048 1 -85.000000 85.000000 3.000000 "
                   "1.000000 5.000000 2 2 100 32 32 0");

    JointLimitDraft excludes_zero;
    joint.raw_position = 2300;
    joint.absolute_degrees = 22.0;
    assert(captureSoftLimit(joint, SoftLimitEndpoint::Minimum,
                            1.0, excludes_zero, error));
    joint.raw_position = 2500;
    joint.absolute_degrees = 40.0;
    assert(captureSoftLimit(joint, SoftLimitEndpoint::Maximum,
                            1.0, excludes_zero, error));
    assert(!formatCalibratedJointLine(joint, excludes_zero, line, error));
    assert(error.find("contain configured zero") != std::string::npos);

    JointLimitDraft reversed;
    joint.raw_position = 2500;
    joint.absolute_degrees = 40.0;
    assert(captureSoftLimit(joint, SoftLimitEndpoint::Minimum,
                            5.0, reversed, error));
    joint.raw_position = 2300;
    joint.absolute_degrees = 22.0;
    assert(!captureSoftLimit(joint, SoftLimitEndpoint::Maximum,
                             5.0, reversed, error));
    assert(!reversed.maximum.captured);

    JointExtrema extrema;
    joint.raw_position = 1024;
    joint.absolute_degrees = -90.0;
    assert(updateJointExtrema(joint, extrema, error));
    joint.raw_position = 2048;
    joint.absolute_degrees = 0.0;
    assert(updateJointExtrema(joint, extrema, error));
    joint.raw_position = 3072;
    joint.absolute_degrees = 90.0;
    assert(updateJointExtrema(joint, extrema, error));
    assert(extrema.samples == 3);
    assert(extrema.minimum_raw == 1024);
    assert(extrema.maximum_raw == 3072);
    JointLimitDraft recorded;
    assert(draftSoftLimitsFromExtrema(joint, extrema, 5.0, 1.0,
                                      recorded, error));
    assert(recorded.minimum.proposed_degrees == -85.0);
    assert(recorded.maximum.proposed_degrees == 85.0);

    JointExtrema insufficient;
    joint.raw_position = 2000;
    joint.absolute_degrees = -4.0;
    assert(updateJointExtrema(joint, insufficient, error));
    joint.raw_position = 2100;
    joint.absolute_degrees = 4.0;
    assert(updateJointExtrema(joint, insufficient, error));
    assert(!draftSoftLimitsFromExtrema(joint, insufficient, 5.0, 1.0,
                                       recorded, error));
    assert(error.find("too small") != std::string::npos);

    std::cout << "dual_arm_calibration_test: PASS\n";
    return 0;
}
