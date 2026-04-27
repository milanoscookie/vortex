#include "problem_description.h"
#include "probe.h"
#include "simulation.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using Complex = RadarSimulator::Complex;
using Probe64 = Probe<problem::kProbeNumX, problem::kProbeNumY>;
using ElementVector64 =
    Eigen::Matrix<Complex, static_cast<int>(problem::kProbeNumElements), 1>;

void ensureParentDirectory(const std::filesystem::path &path) {
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

Complex chirpSample(float t_s, float chirp_slope_hz_per_s, float start_hz) {
  const float phase = 2.0f * problem::kPi *
                      (start_hz * t_s +
                       0.5f * chirp_slope_hz_per_s * t_s * t_s);
  return std::polar(1.0f, phase);
}

std::vector<Complex> makeChirpReference(float sample_rate_hz,
                                        float bandwidth_hz) {
  std::vector<Complex> chirp(RadarSimulator::kBlockSize);
  const float chirp_duration_s =
      static_cast<float>(RadarSimulator::kBlockSize) / sample_rate_hz;
  const float chirp_slope_hz_per_s = bandwidth_hz / chirp_duration_s;
  const float start_hz = -0.5f * bandwidth_hz;

  for (std::size_t n = 0; n < RadarSimulator::kBlockSize; ++n) {
    const float t_s = static_cast<float>(n) / sample_rate_hz;
    chirp[n] = chirpSample(t_s, chirp_slope_hz_per_s, start_hz);
  }

  return chirp;
}

Probe64 makeProbe(float carrier_hz) {
  const float lambda_m = 299'792'458.0f / carrier_hz;
  const float spacing_m = 0.5f * lambda_m;
  return Probe64(coords::Coords::Zero(), spacing_m, spacing_m);
}

void writeKeyValueCsv(
    const std::filesystem::path &path,
    const std::vector<std::pair<std::string, std::string>> &rows) {
  ensureParentDirectory(path);
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("failed to open metadata csv");
  }

  out << "key,value\n";
  for (const auto &[key, value] : rows) {
    out << key << ',' << value << '\n';
  }
}

void writeTruthHeader(std::ofstream &out) {
  out << "chirp_index,time_s,x_m,y_m,z_m,range_m,radial_velocity_mps,doppler_"
         "hz\n";
  out << std::fixed << std::setprecision(9);
}

void appendTruthRow(std::ofstream &out, std::size_t chirp_index,
                    const RadarSimulator::Metrics &metrics) {
  out << chirp_index << ',' << metrics.time_s << ',' << metrics.position_m.x()
      << ',' << metrics.position_m.y() << ',' << metrics.position_m.z() << ','
      << metrics.range_m << ',' << metrics.radial_velocity_mps << ','
      << metrics.doppler_hz << '\n';
}

void writeComplexBinary(const std::filesystem::path &path,
                        const std::vector<Complex> &samples) {
  ensureParentDirectory(path);
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("failed to open binary for writing");
  }
  out.write(reinterpret_cast<const char *>(samples.data()),
            static_cast<std::streamsize>(sizeof(Complex) * samples.size()));
}

std::string formatFloat(float value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(9) << value;
  return out.str();
}

} // namespace

int main() {
  try {
    const auto output_dir = std::filesystem::path("output");
    const auto metadata_path = output_dir / "metadata.csv";
    const auto truth_path = output_dir / "truth.csv";
    const auto tx_path = output_dir / "tx_chirp.bin";
    const auto rx_path = output_dir / "rx_burst.bin";

    const problem::ProblemDescription description =
        problem::kDefaultProblemDescription;

    problem::Config sim_config = description.simulation;
    sim_config.noise_stddev = 0.0f;
    sim_config.max_range_m = 500.0f;
    sim_config.bandwidth_hz = 10.0e6f;
    const float v_comp = 30.0f * std::sqrt(2.0f);
    problem::TargetModel target = description.target;
    target.initial_position_m = RadarSimulator::Vec3(100.0f, 100.0f, 150.0f);
    target.base_velocity_mps = RadarSimulator::Vec3(v_comp, v_comp, 0.0f);

    const float chirp_duration_s =
        static_cast<float>(RadarSimulator::kBlockSize) / sim_config.sample_rate_hz;
    const std::size_t chirp_count =
        static_cast<std::size_t>(std::llround(0.1f / chirp_duration_s));

    RadarSimulator simulator(sim_config, target, description.floorplane, description.random_seed);
    const Probe64 probe = makeProbe(sim_config.carrier_hz);
    const auto probe_state = simulator.prepareProbeState(probe);
    const std::vector<Complex> tx_chirp =
        makeChirpReference(sim_config.sample_rate_hz, sim_config.bandwidth_hz);

    writeComplexBinary(tx_path, tx_chirp);

    ensureParentDirectory(rx_path);
    std::ofstream rx_out(rx_path, std::ios::binary);
    if (!rx_out) {
      throw std::runtime_error("failed to open rx burst binary");
    }

    ensureParentDirectory(truth_path);
    std::ofstream truth_out(truth_path);
    if (!truth_out) {
      throw std::runtime_error("failed to open truth csv");
    }
    writeTruthHeader(truth_out);

    constexpr std::size_t kBatchSize = 64;
    std::vector<Complex> batch_buffer(kBatchSize * RadarSimulator::kBlockSize * problem::kProbeNumElements);

    // Optimized loop: Batch multiple chirps before writing to disk
    for (std::size_t batch_start = 0; batch_start < chirp_count; batch_start += kBatchSize) {
      std::size_t current_batch_size = std::min(kBatchSize, chirp_count - batch_start);

      for (std::size_t b = 0; b < current_batch_size; ++b) {
        std::size_t chirp_idx = batch_start + b;
        Complex* chirp_ptr = batch_buffer.data() + b * RadarSimulator::kBlockSize * problem::kProbeNumElements;

        // Heavy multi-core computation happens here
        simulator.stepChirp(probe_state, chirp_ptr, tx_chirp.data());
        appendTruthRow(truth_out, chirp_idx, simulator.lastMetrics());
      }

      // Perform one large write per batch (e.g. 8MB per write)
      rx_out.write(reinterpret_cast<const char*>(batch_buffer.data()),
                   static_cast<std::streamsize>(current_batch_size * RadarSimulator::kBlockSize * problem::kProbeNumElements * sizeof(Complex)));
    }

    const float lambda_m = sim_config.speed_of_light_mps / sim_config.carrier_hz;
    const float probe_spacing_m = 0.5f * lambda_m;

    writeKeyValueCsv(
        metadata_path,
        {{"sample_rate_hz", formatFloat(sim_config.sample_rate_hz)},
         {"carrier_hz", formatFloat(sim_config.carrier_hz)},
         {"bandwidth_hz", formatFloat(sim_config.bandwidth_hz)},
         {"speed_of_light_mps", formatFloat(sim_config.speed_of_light_mps)},
         {"block_size", std::to_string(RadarSimulator::kBlockSize)},
         {"chirp_count", std::to_string(chirp_count)},
         {"chirp_duration_s", formatFloat(chirp_duration_s)},
         {"probe_num_x", std::to_string(problem::kProbeNumX)},
         {"probe_num_y", std::to_string(problem::kProbeNumY)},
         {"probe_dx_m", formatFloat(probe_spacing_m)},
         {"probe_dy_m", formatFloat(probe_spacing_m)},
         {"tx_chirp_path", tx_path.filename().string()},
         {"rx_burst_path", rx_path.filename().string()},
         {"truth_path", truth_path.filename().string()}});

    std::cout << "Optimized simulation complete (Batch I/O Enabled).\n";
  } catch (const std::exception &ex) {
    std::cerr << "Radar demo failed: " << ex.what() << '\n';
    return 1;
  }
  return 0;
}
