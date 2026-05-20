#pragma once

#include "radar.h"

namespace radar {

class StudentTracker {
  public:
    explicit StudentTracker(const dsp::ProblemDescription& description);

    // Processes a single coherent chirp block.
    // The student implements their DSP pipeline here.
    void processChirp(const SignalBlock& block);

    // Returns the current best estimate of the target's state.
    TrackerResult getLatestEstimate() const;

  private:
    dsp::ProblemDescription description_;
    TrackerResult latest_estimate_;

    // -----------------------------------------------------------------
    // PRECOMPUTED VARIABLES
    // -----------------------------------------------------------------
    // Hint: Store your window coefficients, FFT plans, and steering 
    // vectors here so you don't re-calculate them every chirp!
    //
    // Eigen::VectorXd window_;
    // Eigen::MatrixXcd steering_grid_; 
};

} // namespace radar
