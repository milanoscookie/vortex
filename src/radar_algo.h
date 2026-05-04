#pragma once

#include "problem_description.h"

#include <complex>
#include <cstddef>
#include <functional>
#include <span>

namespace radar_algo {

using Complex = std::complex<float>;
using ChirpCallback = std::function<void(std::size_t chirp_index,
                                         std::span<const Complex> tx_chirp,
                                         std::span<const Complex> rx_block)>;

std::size_t computeChirpCount(const problem::ProblemDescription &description);
problem::ProblemDescription
makeSingleTargetTrackingDescription(const problem::ProblemDescription &base);
void streamRadarChirps(const problem::ProblemDescription &description,
                       const ChirpCallback &callback);

} // namespace radar_algo
