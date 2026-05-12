#pragma once

#include "fmcw_tracker_types.h"

#include <array>
#include <memory>
#include <span>

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

namespace fmcw_tracker {

class StreamingTracker;

namespace detail {

using Vec3 = problem::Vec3;

struct CandidateScoreScratch {
    std::size_t doppler_bin = 0;
    std::size_t range_bin = 0;
    Real range_m = 0.0f;
    Real doppler_hz = 0.0f;
    Real score = 0.0f;
};

struct TrackState {
    std::size_t id = 0;
    TrackStatus status = TrackStatus::Tentative;
    bool initialized = false;
    bool deleted = false;
    Real time_s = 0.0f;
    Real range_m = 0.0f;
    Real radial_velocity_mps = 0.0f;
    Real doppler_hz = 0.0f;
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 direction = Vec3::UnitX();
    Real azimuth_deg = 0.0f;
    Real elevation_deg = 0.0f;
    Real azimuth_rate_degps = 0.0f;
    Real elevation_rate_degps = 0.0f;
    std::size_t hit_count = 0;
    std::size_t miss_count = 0;
    std::vector<std::uint8_t> recent_matches;
    std::vector<TrackReport> history;
};

struct SceneObservation {
    Real time_s = 0.0f;
    std::vector<MeasurementCandidate> measurements;
    std::vector<Complex> spec_cube;
    std::vector<Complex> rd_cube;
    std::size_t n_chirps = 0;
    std::size_t nfft_doppler = 0;
    std::size_t range_count = 0;
    std::size_t num_rx = 0;
};

struct SteeringGrid {
    std::vector<Complex> steering_conj;
    std::vector<Vec3> directions;
    std::vector<Real> azimuth_deg;
    std::vector<Real> elevation_deg;
};

struct KinematicEstimate {
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 direction = Vec3::UnitX();
    Real range_m = 0.0f;
    Real radial_velocity_mps = 0.0f;
    Real doppler_hz = 0.0f;
};

struct PeakInterpResult {
    Real delta = 0.0f;
    bool valid = true;
};

struct AssignmentSolution {
    std::vector<int> measurement_for_track;
    std::vector<int> track_for_measurement;
};

std::size_t computeChirpCount(const problem::ProblemDescription &description);
RadarConfig makeRadarConfig(const problem::ProblemDescription &description);
std::vector<Real> linspace(Real start, Real stop, std::size_t count);
std::vector<Real> hannWindow(std::size_t length);
std::vector<Real> fftFreq(std::size_t nfft, Real sample_period_s);
std::size_t centeredBinToFftIndex(std::size_t centered_bin, std::size_t nfft);
std::size_t cubeIndex(std::size_t outer,
                      std::size_t middle,
                      std::size_t inner,
                      std::size_t middle_size,
                      std::size_t inner_size);
std::vector<Vec3> elementPositions(const RadarConfig &cfg);
SteeringGrid makeSteeringGrid(const RadarConfig &cfg, const DetectionConfig &det);
std::vector<Complex> makeComplexWindow(const std::vector<Real> &real_window);
Real degrees(Real radians);
Real azimuthDeg(const Vec3 &direction);
Real elevationDeg(const Vec3 &direction);
Real wrapAngleDeg(Real angle_deg);
Real angleDifferenceDeg(Real lhs_deg, Real rhs_deg);
Vec3 normalizedOr(const Vec3 &value, const Vec3 &fallback);
KinematicEstimate makeKinematicEstimate(const RadarConfig &cfg,
                                        const Vec3 &position_m,
                                        const Vec3 &velocity_mps,
                                        const Vec3 &direction_fallback);
PeakInterpResult quadraticPeakOffsetChecked(Real power_minus, Real power_center, Real power_plus);
Real interpolateAxis(const std::vector<Real> &axis, std::size_t bin, Real offset);
std::size_t nearestAxisBin(const std::vector<Real> &axis, Real value);
std::vector<double> unwrapPhases(const std::vector<double> &phases);
std::vector<double> movingAverageSame(const std::vector<double> &signal, std::size_t window_size);
std::vector<double> gradient(const std::vector<double> &values, const std::vector<double> &times_s);
double estimateDominantCpiResidualFrequencyHz(const std::vector<BatchResult> &batch_results,
                                              double chirp_duration_s,
                                              std::vector<double> *times_s_out,
                                              std::vector<double> *residual_doppler_hz_out,
                                              std::vector<double> *filtered_doppler_hz_out,
                                              std::vector<double> *candidate_frequency_hz_out,
                                              std::vector<double> *candidate_power_out,
                                              double *peak_power_out);
double estimateDominantBatchDopplerFrequencyHz(const std::vector<BatchResult> &batch_results,
                                               double batch_period_s,
                                               const RadarConfig &radar_config,
                                               std::vector<double> *candidate_frequency_hz_out,
                                               std::vector<double> *candidate_power_out,
                                               double *peak_power_out,
                                               std::size_t *valid_batch_count_out);
Vec3 measurementPosition(const MeasurementCandidate &measurement);
Vec3 measurementVelocity(const MeasurementCandidate &measurement);
Real measurementAzimuthDeg(const MeasurementCandidate &measurement);
Real measurementElevationDeg(const MeasurementCandidate &measurement);

std::vector<DetectionCell> detectLocalPeakCells(const std::vector<Real> &rd_power,
                                                std::size_t range_count,
                                                std::size_t nfft_doppler,
                                                const DetectionConfig &config);
std::vector<std::vector<DetectionCell>>
clusterNearbyPeaks(const std::vector<DetectionCell> &detections, const DetectionConfig &config);
Real computeAssociationDistanceSq(const RadarConfig &radar_config,
                                  Real delta_range_m,
                                  Real delta_radial_velocity_mps,
                                  Real delta_azimuth_deg,
                                  Real delta_elevation_deg,
                                  const DetectionConfig &config);
bool shouldSuppressBirth(const MeasurementCandidate &measurement,
                         const std::vector<TrackReport> &scene_tracks,
                         const DetectionConfig &config,
                         const RadarConfig &radar_config);
bool shouldSuppressBirthFromGhost(const MeasurementCandidate &measurement,
                                  const std::vector<DeletedTrackGhost> &ghosts,
                                  Real time_s,
                                  const DetectionConfig &config,
                                  const RadarConfig &radar_config);
bool isBirthCandidateValid(const MeasurementCandidate &measurement, const DetectionConfig &config);
std::vector<MeasurementCandidate>
mergeNearbyMeasurements(const std::vector<MeasurementCandidate> &measurements,
                        const DetectionConfig &config);
std::vector<MeasurementCandidate>
selectTopMeasurements(std::vector<MeasurementCandidate> measurements, std::size_t max_count);
void updateRecentMatches(std::vector<std::uint8_t> &recent_matches,
                         bool matched,
                         std::size_t max_window);
std::size_t recentHitCount(const std::vector<std::uint8_t> &recent_matches);
AssignmentSolution solveAssignments(const std::vector<std::vector<Real>> &cost_matrix,
                                    Real miss_cost);
std::size_t nearestTruthCarIndex(const problem::ProblemDescription &description,
                                 Real time_s,
                                 const Vec3 &position_m);

} // namespace detail
} // namespace fmcw_tracker
