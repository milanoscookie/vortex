#pragma once

#include "fmcw_tracker_internal.h"
#include "fmcw_tracker_types.h"
#include "problem_description.h"
#include "utils/RingBuffer.h"

#include <array>
#include <memory>
#include <span>

namespace fmcw_tracker {

class StreamingTracker {
  public:
    static constexpr std::size_t kBlockSize = problem::RadarSettings::kRadarBlockSize;
    static constexpr std::size_t kNumRx = problem::RadarSettings::kProbeNumElements;
    static constexpr std::size_t kMaxCpiChirps = 256U;
    using ChirpBlock = std::array<Complex, kBlockSize * kNumRx>;

    explicit StreamingTracker(const problem::ProblemDescription &description,
                              DetectionConfig detection_config = {});

    void pushChirp(std::size_t chirp_index,
                   std::span<const Complex> tx_chirp,
                   std::span<const Complex> rx_block);

    const RadarConfig &radarConfig() const noexcept {
        return radar_config_;
    }

    const DetectionConfig &detectionConfig() const noexcept {
        return detection_config_;
    }

    const std::vector<SceneBatchResult> &sceneBatchResults() const noexcept {
        return scene_batch_results_;
    }

    const std::vector<int> &rangeIndices() const noexcept {
        return range_indices_;
    }

    TrackSummary buildSummary() const;
    SceneSummary buildSceneSummary() const;

    struct AoAPeak {
        Real azimuth_deg;
        Real elevation_deg;
        Real score;
        problem::Vec3 direction;
    };
    std::vector<AoAPeak> estimateTopAoAs(std::span<const Complex> snapshot, std::size_t n) const;
    std::size_t getRangeBin(Real range_m) const;
    Complex computeRangeFftBin(std::span<const Complex> dechirped_chirp, std::size_t rx, std::size_t range_bin) const;

  private:
    detail::SceneObservation processCurrentWindow(std::size_t start_chirp);
    void updateTracks(const detail::SceneObservation &observation);
    std::vector<problem::SimulationMetrics> truthSceneAtTime(Real time_s) const;

    problem::ProblemDescription description_;
    RadarConfig radar_config_;
    DetectionConfig detection_config_;
    std::vector<int> range_indices_;
    std::vector<Real> range_axis_sliced_m_;
    std::size_t range_bin_count_ = 0;
    std::vector<Complex> range_window_;
    std::vector<Complex> doppler_window_;
    std::vector<Real> doppler_axis_hz_;
    std::vector<Real> velocity_axis_mps_;
    std::vector<Complex> steering_conj_;
    std::vector<problem::Vec3> directions_;
    std::vector<Real> direction_azimuth_deg_;
    std::vector<Real> direction_elevation_deg_;
    Eigen::FFT<Real> fft_;
    std::vector<Complex> range_fft_input_;
    std::vector<Complex> range_fft_output_;
    std::vector<Complex> doppler_fft_input_;
    std::vector<Complex> doppler_fft_output_;
    std::vector<Complex> spec_scratch_;
    std::vector<Complex> rd_cube_scratch_;
    std::vector<Complex> clutter_mean_scratch_;
    std::vector<Real> rd_power_scratch_;
    std::vector<detail::CandidateScoreScratch> rd_candidates_scratch_;
    std::unique_ptr<RingBuffer<ChirpBlock, kMaxCpiChirps>> chirp_window_;
    std::vector<SceneBatchResult> scene_batch_results_;
    std::vector<detail::TrackState> active_tracks_;
    std::vector<detail::TrackState> finished_tracks_;
    std::vector<DeletedTrackGhost> deleted_track_ghosts_;
    std::size_t next_track_id_ = 1;
    std::size_t range_wrap_guard_samples_ = 0;
};

problem::SimulationMetrics
truthAtTime(const problem::ProblemDescription &description, Real time_s, std::size_t car_index = 0);

} // namespace fmcw_tracker
