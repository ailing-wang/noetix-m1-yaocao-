#pragma once

#include "dual_arm_local/Controller.h"

#include <cstddef>
#include <string>

namespace noetix::dual_arm_local
{
enum class SoftLimitEndpoint
{
    Minimum,
    Maximum
};

struct SoftLimitCapture
{
    bool captured = false;
    int raw_position = 0;
    double measured_degrees = 0.0;
    double margin_degrees = 0.0;
    double proposed_degrees = 0.0;
};

struct JointLimitDraft
{
    SoftLimitCapture minimum;
    SoftLimitCapture maximum;
};

struct JointExtrema
{
    bool observed = false;
    std::size_t samples = 0;
    int minimum_raw = 0;
    int maximum_raw = 0;
    double minimum_degrees = 0.0;
    double maximum_degrees = 0.0;
};

bool updateJointExtrema(const JointSnapshot &joint,
                        JointExtrema &extrema,
                        std::string &error);

bool draftSoftLimitsFromExtrema(const JointSnapshot &joint,
                                const JointExtrema &extrema,
                                double margin_degrees,
                                double minimum_usable_span_degrees,
                                JointLimitDraft &draft,
                                std::string &error);

// Records one manually observed mechanical endpoint and moves the proposed
// software limit inward by margin_degrees. This function never accesses or
// commands hardware.
bool captureSoftLimit(const JointSnapshot &joint,
                      SoftLimitEndpoint endpoint,
                      double margin_degrees,
                      JointLimitDraft &draft,
                      std::string &error);

// Produces one version=2 joint configuration row with calibrated=1. Both
// endpoints must already be captured and the resulting range must contain the
// configured zero. The function only formats text; it never edits a file.
bool formatCalibratedJointLine(const JointSnapshot &joint,
                               const JointLimitDraft &draft,
                               std::string &line,
                               std::string &error);
} // namespace noetix::dual_arm_local
