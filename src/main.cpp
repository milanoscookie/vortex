#include "problem_description.h"
#include "probe.h"
#include "simulation.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using Complex = RadarSimulator::Complex;
using Probe64 = Probe<problem::RadarSettings::kProbeNumX,
                      problem::RadarSettings::kProbeNumY>;
constexpr std::size_t kBatchSize = 64;
constexpr std::size_t kWriteBatchSize = 256;

struct OutputPaths {
  std::filesystem::path metadata_csv;
  std::filesystem::path truth_csv;
  std::filesystem::path tx_chirp_binary;
  std::filesystem::path rx_burst_binary;
};

OutputPaths makeOutputPaths(const std::filesystem::path &output_dir) {
  return {
      .metadata_csv = output_dir / "metadata.csv",
      .truth_csv = output_dir / "truth.csv",
      .tx_chirp_binary = output_dir / "tx_chirp.bin",
      .rx_burst_binary = output_dir / "rx_burst.bin",
  };
}

void ensureParentDirectory(const std::filesystem::path &path) {
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

Complex chirpSample(float t_s, float chirp_slope_hz_per_s, float start_hz) {
  const float phase = 2.0f * problem::Constants::kPi *
                      (start_hz * t_s +
                       0.5f * chirp_slope_hz_per_s * t_s * t_s);
  return Complex(std::cos(phase), std::sin(phase));
}

std::vector<Complex> makeChirpReference(
    const problem::RadarSettings &radar_settings) {
  std::vector<Complex> chirp(RadarSimulator::kBlockSize);
  const float chirp_duration_s =
      static_cast<float>(RadarSimulator::kBlockSize) /
      radar_settings.sample_rate_hz;
  const float chirp_slope_hz_per_s =
      radar_settings.bandwidth_hz / chirp_duration_s;
  const float start_hz = -0.5f * radar_settings.bandwidth_hz;

  for (std::size_t n = 0; n < RadarSimulator::kBlockSize; ++n) {
    const float t_s = static_cast<float>(n) / radar_settings.sample_rate_hz;
    chirp[n] = chirpSample(t_s, chirp_slope_hz_per_s, start_hz);
  }

  return chirp;
}

Probe64 makeProbe(const problem::ProblemDescription &description) {
  const auto &radar_settings = description.radar;
  const auto &probe_settings = description.probe;
  const float lambda_m =
      problem::Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
  const float spacing_x_m = probe_settings.spacing_x_wavelengths * lambda_m;
  const float spacing_y_m = probe_settings.spacing_y_wavelengths * lambda_m;
  return Probe64(probe_settings.center_m, spacing_x_m, spacing_y_m);
}

std::ofstream openOutputFile(const std::filesystem::path &path,
                             std::ios::openmode mode,
                             std::string_view description) {
  ensureParentDirectory(path);
  std::ofstream out(path, mode);
  if (!out) {
    throw std::runtime_error("failed to open " + std::string(description));
  }
  return out;
}

void writeKeyValueCsv(
    const std::filesystem::path &path,
    const std::vector<std::pair<std::string, std::string>> &rows) {
  std::ofstream out = openOutputFile(path, std::ios::out, "metadata csv");

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
                    const RadarSimulator::SimulationMetrics &metrics) {
  out << chirp_index << ',' << metrics.time_s << ',' << metrics.position_m.x()
      << ',' << metrics.position_m.y() << ',' << metrics.position_m.z() << ','
      << metrics.range_m << ',' << metrics.radial_velocity_mps << ','
      << metrics.doppler_hz << '\n';
}

void writeComplexBinary(const std::filesystem::path &path,
                        const std::vector<Complex> &samples) {
  std::ofstream out =
      openOutputFile(path, std::ios::out | std::ios::binary, "binary output");
  out.write(reinterpret_cast<const char *>(samples.data()),
            static_cast<std::streamsize>(sizeof(Complex) * samples.size()));
}

std::string formatFloat(float value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(9) << value;
  return out.str();
}

std::size_t computeChirpCount(const problem::ProblemDescription &description) {
  const float chirp_duration_s =
      static_cast<float>(RadarSimulator::kBlockSize) /
      description.radar.sample_rate_hz;
  return static_cast<std::size_t>(
      std::llround(description.simulator.burst_duration_s / chirp_duration_s));
}

std::vector<std::pair<std::string, std::string>> buildMetadataRows(
    const problem::ProblemDescription &description, std::size_t chirp_count,
    const OutputPaths &output_paths) {
  const problem::RadarSettings &radar_settings = description.radar;
  const float chirp_duration_s =
      static_cast<float>(RadarSimulator::kBlockSize) /
      radar_settings.sample_rate_hz;
  const float lambda_m =
      problem::Constants::kSpeedOfLightMps / radar_settings.carrier_hz;
  const float probe_spacing_x_m =
      description.probe.spacing_x_wavelengths * lambda_m;
  const float probe_spacing_y_m =
      description.probe.spacing_y_wavelengths * lambda_m;

  return {{"sample_rate_hz", formatFloat(radar_settings.sample_rate_hz)},
          {"carrier_hz", formatFloat(radar_settings.carrier_hz)},
          {"bandwidth_hz", formatFloat(radar_settings.bandwidth_hz)},
          {"speed_of_light_mps",
           formatFloat(problem::Constants::kSpeedOfLightMps)},
          {"block_size", std::to_string(RadarSimulator::kBlockSize)},
          {"chirp_count", std::to_string(chirp_count)},
          {"chirp_duration_s", formatFloat(chirp_duration_s)},
          {"burst_duration_s",
           formatFloat(description.simulator.burst_duration_s)},
          {"probe_num_x", std::to_string(problem::RadarSettings::kProbeNumX)},
          {"probe_num_y", std::to_string(problem::RadarSettings::kProbeNumY)},
          {"probe_dx_m", formatFloat(probe_spacing_x_m)},
          {"probe_dy_m", formatFloat(probe_spacing_y_m)},
          {"tx_chirp_path", output_paths.tx_chirp_binary.filename().string()},
          {"rx_burst_path", output_paths.rx_burst_binary.filename().string()},
          {"truth_path", output_paths.truth_csv.filename().string()}};
}

} // namespace

int main() {
  try {
    const auto output_dir = std::filesystem::path("output");
    const OutputPaths output_paths = makeOutputPaths(output_dir);

    const problem::ProblemDescription description =
        problem::kDefaultProblemDescription;

    const problem::RadarSettings &radar_settings = description.radar;
    const std::size_t chirp_count = computeChirpCount(description);

    RadarSimulator simulator(description);
    const Probe64 probe = makeProbe(description);
    const auto probe_state = simulator.prepareProbeState(probe);
    const std::vector<Complex> tx_chirp = makeChirpReference(radar_settings);
    RadarSimulator::ElementVector step_output;

    writeComplexBinary(output_paths.tx_chirp_binary, tx_chirp);

    std::ofstream rx_out = openOutputFile(output_paths.rx_burst_binary,
                                          std::ios::out | std::ios::binary,
                                          "rx burst binary");

    std::ofstream truth_out =
        openOutputFile(output_paths.truth_csv, std::ios::out, "truth csv");
    writeTruthHeader(truth_out);

    std::vector<Complex> batch_buffer(
        kWriteBatchSize * RadarSimulator::kBlockSize *
        problem::RadarSettings::kProbeNumElements);

    for (std::size_t batch_start = 0; batch_start < chirp_count;
         batch_start += kWriteBatchSize) {
      const std::size_t current_batch_size =
          std::min(kWriteBatchSize, chirp_count - batch_start);

      for (std::size_t b = 0; b < current_batch_size; ++b) {
        const std::size_t chirp_idx = batch_start + b;
        Complex *chirp_ptr =
            batch_buffer.data() +
            b * RadarSimulator::kBlockSize *
                problem::RadarSettings::kProbeNumElements;

        for (std::size_t sample_index = 0; sample_index < RadarSimulator::kBlockSize;
             ++sample_index) {
          simulator.step<problem::RadarSettings::kProbeNumX,
                         problem::RadarSettings::kProbeNumY>(
              probe_state, step_output, tx_chirp[sample_index]);
          std::memcpy(
              chirp_ptr + sample_index * problem::RadarSettings::kProbeNumElements,
              step_output.data(),
              sizeof(Complex) * problem::RadarSettings::kProbeNumElements);
        }
        appendTruthRow(truth_out, chirp_idx, simulator.lastMetrics());
      }

      rx_out.write(reinterpret_cast<const char *>(batch_buffer.data()),
                   static_cast<std::streamsize>(
                       current_batch_size * RadarSimulator::kBlockSize *
                       problem::RadarSettings::kProbeNumElements *
                       sizeof(Complex)));
    }

    writeKeyValueCsv(output_paths.metadata_csv,
                     buildMetadataRows(description, chirp_count, output_paths));

    std::cout << "Optimized simulation complete (Batch I/O Enabled).\n";
  } catch (const std::exception &ex) {
    std::cerr << "Radar demo failed: " << ex.what() << '\n';
    return 1;
  }
  return 0;
}
