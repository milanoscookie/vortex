#include "student_tracker.h"

#include <complex>
#include <vector>

namespace radar {

StudentTracker::StudentTracker(const dsp::ProblemDescription& description)
    : description_(description) {
    latest_estimate_.valid = false;

}

void StudentTracker::processChirp(const SignalBlock& block) {
    // =================================================================
    // STUDENT WORKSPACE: RADAR DSP PIPELINE
    // =================================================================
    (void)block;
}

TrackerResult StudentTracker::getLatestEstimate() const {
    return latest_estimate_;
}

} // namespace radar
