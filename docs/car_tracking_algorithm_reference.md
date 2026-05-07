# Radar Car-Tracking Algorithm Reference

This document is a detailed technical reference for the simulator + signal-processing stack in this repository.
It is intended to provide enough equations, data formats, and implementation details for someone to build a car-tracking algorithm that is consistent with the current codebase.

## 1) End-to-End Pipeline

The current pipeline is:

1. **Scenario definition** (`lib/problem_description.h`)
2. **Signal generation / simulation** (`lib/simulation.h`, `lib/simulation.cpp`, `lib/dynamics.cpp`, `lib/environment.cpp`)
3. **Optional dataset export** (`src/main.cpp`) to:
   - `output/tx_chirp.bin`
   - `output/rx_burst.bin`
   - `output/truth.csv`
   - `output/metadata.csv`
4. **Streaming tracking / estimation** (`src/fmcw_tracker.cpp`, `src/fmcw_tracker.h`)

The active tracker is the C++ `fmcw_tracker::StreamingTracker`. It consumes chirps directly in memory via `pushChirp(...)`; it does not read exported files.

---

## 2) Coordinate System and State Definitions

### 2.1 Coordinate convention (as implemented)

- Car state is stored as `(x, y, z)` in meters.
- Probe center is at origin by default.
- In the tracker array geometry, receive elements are spread in local **x** and **y** with `z=0` for each element.
- Element positions are centered on the array midpoint:

\[
\mathbf{r}_{i_x,i_y} =
\begin{bmatrix}
\left(i_x - \frac{N_x-1}{2}\right)d_x \\
\left(i_y - \frac{N_y-1}{2}\right)d_y \\
0
\end{bmatrix}
\]

- The LOS (line-of-sight) direction from radar to target is:

\[
\hat{u} = \frac{\mathbf{p}}{\|\mathbf{p}\|}
\]

where \(\mathbf{p} = [x,y,z]^T\).

### 2.2 Car dynamics model

`lib/dynamics.cpp` uses:

\[
\mathbf{p}(t) = \mathbf{p}_0 + \mathbf{v}_0 t + \begin{bmatrix}0\\0\\A\sin(2\pi f_b t + \phi_b)\end{bmatrix}
\]

\[
\mathbf{v}(t) = \mathbf{v}_0 + \begin{bmatrix}0\\0\\2\pi f_b A\cos(2\pi f_b t + \phi_b)\end{bmatrix}
\]

where:

- \(\mathbf{p}_0\): `initial_position_m`
- \(\mathbf{v}_0\): `base_velocity_mps`
- \(A\): `bounce_amplitude_m`
- \(f_b\): `bounce_frequency_hz`
- \(\phi_b\): `bounce_phase_rad`

Default currently has no bounce (`A=0`).

---

## 3) Radar Signal Model Used by Simulator

### 3.1 FMCW transmit chirp

From `src/main.cpp`:

- Chirp duration:

\[
T_c = \frac{N_s}{f_s}
\]

where:
- \(N_s\): `kRadarBlockSize`
- \(f_s\): `sample_rate_hz`

- Chirp slope:

\[
\mu = \frac{B}{T_c}
\]

- Start frequency offset: \(f_0 = -\frac{B}{2}\)

- Complex baseband TX sample:

\[
s_{tx}(t) = e^{j2\pi\left(f_0 t + \frac{1}{2}\mu t^2\right)}
\]

### 3.2 Propagation / reflection model

From `RadarSimulator::step` in `lib/simulation.h`:

- Range:

\[
R = \|\mathbf{p}(t)-\mathbf{p}_{rx}\|
\]

- Two-way delay:

\[
\tau = \frac{2R}{c} + \tau_{elem}
\]

- Path gain:

\[
g(R) = \frac{G}{R^2}
\]

where \(G\) is `field_gain`.

- Wavelength:

\[
\lambda = \frac{c}{f_c}
\]

- Propagation phase:

\[
\phi_{prop} = -\frac{4\pi R}{\lambda}
\]

- Radial velocity:

\[
v_r = \mathbf{v}(t) \cdot \hat{u}
\]

- Doppler phase term:

\[
\phi_D(t) = \frac{4\pi v_r t}{\lambda}
\]

- Received contribution per element:

\[
s_{rx}(t) = w\; s_{tx}(t-\tau)\;\Gamma\; e^{j\phi_{prop}}\; e^{j\phi_D(t)}
\]

where:
- \(w\): element weight
- \(\Gamma\): target reflectivity (`std::complex<float>`)

### 3.3 Noise model

From `lib/simulation.cpp`:

- Complex IID Gaussian noise with per-component variance scaling:

\[
n = n_I + j n_Q, \quad n_I,n_Q \sim \mathcal{N}(0,\sigma^2)
\]

\[
\sigma = \texttt{noise\_stddev} \cdot \frac{1}{\sqrt{2}}
\]

Constants are centralized in `lib/problem_config.h`:

- `kDefaultNoiseStddev`
- `kNoiseDistributionMean`
- `kNoiseDistributionStddev`
- `kComplexNoiseQuadratureScale`

### 3.4 Optional static floorplane clutter

From `lib/environment.cpp`:

- Beat frequency for static reflector at configured range \(R_f\):

\[
f_{b,floor} = \frac{2\mu R_f}{c}
\]

- Amplitude scaling:

\[
A_f = A_{ref}\left(\frac{R_{ref}}{R_f}\right)^{\alpha}
\]

- Base phase:

\[
\phi_{f,0} = -\frac{4\pi R_f}{\lambda} + \phi_{cfg}
\]

- Sampled clutter term:

\[
s_{floor}(t) = A_f e^{j(2\pi f_{b,floor}t + \phi_{f,0})}
\]

This is added to each active element when enabled.

---

## 4) Tracker Interface (`src/fmcw_tracker.h`)

### 4.1 Construction inputs

`StreamingTracker` is constructed from:

- `problem::ProblemDescription`
- `DetectionConfig`

`RadarConfig` is derived from the scenario and contains:

- `sample_rate_hz`
- `carrier_hz`
- `bandwidth_hz`
- `chirp_duration_s`
- `speed_of_light_mps`
- `block_size`
- `chirp_count`
- `probe_num_x`, `probe_num_y`
- `probe_dx_m`, `probe_dy_m`

### 4.2 Runtime input contract

Each chirp is streamed into:

\[
\texttt{pushChirp(chirp\_index, tx\_chirp, rx\_block)}
\]

with:

- `tx_chirp`: shape `[block_size]`, complex baseband TX reference
- `rx_block`: shape `[block_size * num_rx]`, flattened sample-major / element-minor burst for one chirp

The tracker caches the conjugated TX chirp on first use:

\[
s_{tx}^*[n]
\]

### 4.3 Fixed implementation constraints

The current implementation enforces:

- CPI chirps = `64`
- Range FFT length = `4096`
- Doppler FFT length = `128` (2x zero-padded CPI)
- Azimuth grid count = `181`
- Elevation grid count = `5001`
- Single-car support only

### 4.4 DetectionConfig fields that directly change behavior

- `min_range_m`, `max_range_m`
- `coherent_processing_interval_chirps`
- `hop_chirps`
- `zero_doppler_guard_bins`
- `nfft_range_min`
- `static_clutter_suppression_enable`
- `aoa_enable`
- `azimuth_min_deg`, `azimuth_max_deg`, `azimuth_count`
- `elevation_min_deg`, `elevation_max_deg`, `elevation_count`
- `range_gate_bins`, `doppler_gate_bins`
- `range_association_sigma_m`, `doppler_association_sigma_hz`
- `range_interp_gate_m`, `doppler_interp_gate_hz`
- `azimuth_association_sigma_deg`, `elevation_association_sigma_deg`

---

## 5) Current Streaming Tracker Algorithm (`src/fmcw_tracker.cpp`)

### 5.1 Sliding window / CPI scheduling

- RX chirps are stored in a ring buffer.
- Once at least `coherent_processing_interval_chirps` chirps have arrived, a CPI is processed every `hop_chirps` chirps.
- For a CPI starting at chirp \(k_0\) with length \(K\), the reported batch time is:

\[
t_{batch} = \left(k_0 + \frac{K}{2}\right)T_c
\]

### 5.2 Fast-time dechirp and wrap guard

For each chirp, sample, and RX channel:

\[
s_{beat}[k,n,m] = s_{rx}[k,n,m] \cdot s_{tx}^*[n]
\]

Before the range FFT, the tracker blanks an initial fast-time prefix to reduce delayed energy leaking in from the previous chirp. The guard is:

\[
N_{guard} = \min\left(\frac{N_s}{8}, \left\lceil \frac{2R_{max}}{c}f_s \right\rceil + 2\right)
\]

where \(R_{max} = \min(\texttt{description.radar.max\_range\_m}, \texttt{DetectionConfig.max\_range\_m})\).

### 5.3 Range FFT and range-axis construction

For each chirp and RX channel:

1. Apply Hann window in fast-time.
2. Zero-pad / FFT to `nfft_range_min`.
3. Keep only **negative-frequency** bins.
4. Convert those bins to range and keep only bins inside the configured range gate.

The implementation assumes the down-chirp convention produces target beat energy on negative FFT frequencies, so range is mapped as:

\[
R = -\frac{c f_b}{2\mu}
\]

No positive/negative folding is done in the C++ tracker.

### 5.4 Optional static clutter suppression

If enabled, remove the slow-time mean per retained range/RX bin:

\[
X_{dyn}[k,r,m] = X[k,r,m] - \frac{1}{K}\sum_{k'}X[k',r,m]
\]

### 5.5 Doppler FFT

For each retained range bin and RX channel:

1. Apply Hann window in slow-time.
2. Zero-pad to `2 * CPI` chirps.
3. FFT along the chirp axis.
4. FFT-shift to centered Doppler order.
5. Form noncoherent power across RX:

\[
P[d,r] = \frac{1}{M}\sum_m |X[d,r,m]|^2
\]

The Doppler axis is built from FFT frequency bins, shifted, then negated to match the simulator sign convention:

\[
f_D = -f_{FFT,shifted}
\]

Velocity mapping is:

\[
v_r = \frac{\lambda}{2}f_D
\]

### 5.6 Prediction-centered candidate search

Instead of taking the global RD maximum, the tracker scores candidates in a gated neighborhood.

If tracking is initialized, the predicted state is propagated with constant velocity:

\[
\mathbf{p}_{pred}(t) = \mathbf{p}_{k-1} + \mathbf{v}_{k-1}\Delta t
\]

and converted to predicted range, Doppler, and direction.

Candidate scoring uses log-power plus Gaussian-style association penalties:

\[
\text{score}_{RD} = \log(P[d,r] + \epsilon)
- \frac{1}{2}\left(\frac{R_{cand}-R_{pred}}{\sigma_R}\right)^2
- \frac{1}{2}\left(\frac{f_{D,cand}-f_{D,pred}}{\sigma_D}\right)^2
\]

where the penalty terms are applied only after initialization.

The tracker also compensates range-Doppler coupling when evaluating a candidate:

\[
R_{cand} = R_{raw}(r) - \frac{c}{2\mu}f_{D,cand}
\]

Only the best few RD candidates (currently 6) are kept for AoA evaluation.

### 5.7 Zero-Doppler guard and search gates

- Bins near zero Doppler are excluded using `zero_doppler_guard_bins`.
- Search is limited to a rectangular gate centered at the predicted range/Doppler bins when tracking is initialized.
- Without an initialized track, the search center defaults to the middle of the current retained range and Doppler axes.

### 5.8 Angle-of-arrival (AoA)

The direction grid is built over azimuth/elevation samples:

\[
\hat{u}(\text{az},\text{el})=
\begin{bmatrix}
\cos(\text{el})\cos(\text{az})\\
\cos(\text{el})\sin(\text{az})\\
\sin(\text{el})
\end{bmatrix}
\]

The steering phase used by the implementation for element position \(\mathbf{r}_m\) is:

\[
a_m(\hat{u}) = e^{-j\frac{2\pi}{\lambda}\hat{u}^T\mathbf{r}_m}
\]

Beam score for an RD-cell snapshot \(x\) is:

\[
S(\hat{u}) = \log\left(|a(\hat{u})^H x|^2 + \epsilon\right)
\]

If tracking is initialized, angular association penalties are added:

\[
\text{score}_{AoA} = S(\hat{u})
- \frac{1}{2}\left(\frac{\Delta az}{\sigma_{az}}\right)^2
- \frac{1}{2}\left(\frac{\Delta el}{\sigma_{el}}\right)^2
\]

Implementation detail: elevation search is accelerated by coarse sampling first, then refining around the best coarse elevation for each azimuth.

### 5.9 Joint RD + AoA selection

For each top RD candidate, AoA is evaluated on the selected snapshot. The total candidate score is:

\[
\text{score}_{total} = \text{score}_{RD} + \text{score}_{AoA}
\]

The tracker keeps the candidate with maximum total score.

### 5.10 Sub-bin interpolation

After joint selection, 1D quadratic interpolation is applied in log-power on both axes.

For neighboring powers \(p_{-1}, p_0, p_{+1}\):

\[
y_i = \log(p_i + \epsilon)
\]

\[
\delta = \frac{1}{2}\frac{y_{-1}-y_{+1}}{y_{-1}-2y_0+y_{+1}}
\]

Interpolation is accepted only if:

- the parabola is concave (`denom < 0`)
- `|delta| < 0.45`
- the refined value does not jump too far from the predicted state

The final refined range again includes range-Doppler coupling correction:

\[
\hat{R} = R_{interp} - \frac{c}{2\mu}\hat{f}_D
\]

### 5.11 Measurement gating and invalid updates

The final measurement is rejected if range or Doppler is non-finite or deviates too far from prediction.

If a batch is invalid but the tracker has prior state, it still propagates the previous state forward and stores predicted range, Doppler, and direction in the `BatchResult`.

### 5.12 State fusion update

When a valid measurement arrives, the tracker does not run a Kalman filter. Instead, it performs a fixed-weight blend between previous tracked state and the new measurement.

With measurement weight \(w=0.75\):

\[
\hat{u}_k = \text{normalize}\left((1-w)\hat{u}_{k-1} + w\hat{u}_{meas}\right)
\]

\[
\hat{R}_k = (1-w)\hat{R}_{k-1} + w\hat{R}_{meas}
\]

\[
\hat{v}_{r,k} = (1-w)\hat{v}_{r,k-1} + w\hat{v}_{r,meas}
\]

The stored Cartesian state is then formed as radial-only motion along the fused direction:

\[
\hat{\mathbf{p}}_k = \hat{R}_k \hat{u}_k
\]

\[
\hat{\mathbf{v}}_k = \hat{v}_{r,k} \hat{u}_k
\]

This is still not a full Cartesian velocity solve.

### 5.13 Per-batch outputs

Each `BatchResult` stores:

- `time_s`
- `range_m`, `doppler_hz`, `phase_rad`
- `predicted_range_m`, `predicted_doppler_hz`, `predicted_direction`
- `direction`
- `range_bin`, `doppler_bin`, `azimuth_bin`, `elevation_bin`
- `range_bin_offset`, `doppler_bin_offset`
- `doppler_slice_power` for the selected range bin
- `slow_time_phase_rad` for the selected beam / range bin
- `valid`

---

## 6) Summary and Micro-Doppler Estimation

`StreamingTracker::buildSummary()` produces a `TrackSummary` with:

- all batch results
- raw and smoothed Cartesian positions
- range and radial-velocity traces
- estimated Cartesian velocity from finite-difference gradients
- truth metrics from `truthAtTime(...)`
- micro-Doppler estimates and diagnostics

### 6.1 Position smoothing and velocity derivation

- Raw positions are formed as:

\[
\mathbf{p}_{raw}[k] = R[k]\,\hat{u}[k]
\]

- A boxcar smoother of length `min(5, num_batches)` is applied independently to `x`, `y`, and `z`.
- Cartesian velocity is then estimated by numerical gradient of the smoothed coordinates.

### 6.2 Micro-Doppler from batch-to-batch Doppler trace

One micro-Doppler estimate is obtained by:

1. taking valid `doppler_hz` values across batches
2. splitting them into contiguous valid segments
3. high-pass filtering each segment with a moving-average subtraction
4. windowing and FFTing each segment
5. accumulating power in the search band 10-100 Hz

The dominant frequency is refined with the same quadratic peak interpolation used elsewhere.

### 6.3 Micro-Doppler from CPI residual phase

Another estimate is obtained from CPI slow-time phase:

1. beamform the selected range bin per chirp
2. remove coarse Doppler phase within each CPI
3. unwrap phase over continuous chirp segments
4. align phase between overlapping CPI batches
5. high-pass filter the unwrapped phase residual
6. FFT and accumulate power in 10-100 Hz

If the batch-Doppler estimate is unavailable, the tracker falls back to this residual-phase estimate.

### 6.4 Reported micro-Doppler metrics

`TrackSummary` reports:

- `microdoppler_phase_frequency_hz`
- `microdoppler_truth_frequency_hz`
- `microdoppler_frequency_rmse_hz`
- `microdoppler_residual_phase_mean_rad`
- `microdoppler_residual_phase_rms_rad`
- `microdoppler_residual_phase_stddev_rad`
- `microdoppler_peak_power`
- `microdoppler_valid_cpi_count`
- top micro-Doppler candidate frequencies and powers

---

## 7) Resolution and Ambiguity Equations

Used by the current tracker design:

- Unambiguous radial velocity span for chirp spacing \(T_c\):

\[
v_{max} = \frac{\lambda}{4T_c}, \quad v \in [-v_{max}, v_{max}]
\]

- Doppler-bin spacing after 2x zero-padding of a CPI with \(K\) chirps:

\[
\Delta f_D = \frac{1}{2KT_c}
\]

- Velocity-bin spacing:

\[
\Delta v = \frac{\lambda}{2}\Delta f_D = \frac{\lambda}{4KT_c}
\]

- Ideal FMCW range resolution:

\[
\Delta R \approx \frac{c}{2B}
\]

---

## 8) Parameters That Most Affect Tracking Quality

### 8.1 Simulator-side (`lib/problem_description.h`)

- `radar.bandwidth_hz`
  - Higher improves range resolution.
- `radar.sample_rate_hz`
  - With fixed block size, lower sample rate increases chirp duration and improves Doppler resolution.
  - But lower sample rate reduces beat-frequency Nyquist headroom.
- `radar.carrier_hz`
  - Higher carrier reduces wavelength and increases Doppler sensitivity.
- `simulator.burst_duration_s`
  - Sets how many chirps are available in total.
- `simulator.noise_stddev`
  - Raises or lowers SNR.
- `floorplane.enable_static_floorplane`
  - Strong static clutter can dominate detection if not suppressed.

### 8.2 Tracker-side (`DetectionConfig`)

- `coherent_processing_interval_chirps`
  - Larger CPI improves Doppler resolution and coherent gain.
- `hop_chirps`
  - Smaller hop increases update rate and overlap between CPIs.
- `zero_doppler_guard_bins`
  - Helps reject static leakage; too large can suppress slow targets.
- `min_range_m` / `max_range_m`
  - Define which negative-frequency range bins are even retained.
- `static_clutter_suppression_enable`
  - Helps in static-heavy scenes; hurts if the target is near-zero Doppler and the scene is clean.
- `range_gate_bins` / `doppler_gate_bins`
  - Determine how tightly the tracker searches around its prediction.
- Association sigmas and interpolation gates
  - Control how strongly prediction biases selection and whether sub-bin refinement is trusted.
- AoA grid span and density
  - Directly trade angular granularity against compute cost.

---

## 9) Implementation Notes and Caveats

1. **Single-target / single-car only**
   - The tracker throws unless the scenario contains exactly one car.

2. **Hard-coded operating point**
   - CPI, range FFT, and AoA grid sizes are fixed by compile-time assumptions.

3. **Negative-frequency-only range processing**
   - Unlike the older Python tracker, there is no positive/negative folding.

4. **Prediction-biased detection**
   - Once initialized, detection is no longer a pure global-max search; gating and association penalties strongly influence selection.

5. **Range-Doppler coupling correction is explicit**
   - Range estimates are corrected by a term proportional to Doppler frequency.

6. **Velocity state remains radial-only**
   - Cartesian velocity is modeled as radial speed along the estimated direction, not independently solved from measurements.

7. **Invalid measurements do not fully break the track**
   - The tracker propagates prior state through missed detections.

8. **Micro-Doppler search band is narrow**
   - The current implementation only searches roughly 10-100 Hz residual content.

---

## 10) Practical Roadmap for a Better Tracker

If building a new algorithm on top of this simulator/tracker stack, a practical roadmap is:

1. Keep the same dechirp, chirp timing, and sign conventions as `src/fmcw_tracker.cpp`.
2. Replace heuristic RD candidate selection with a detector such as CFAR + clustering.
3. Replace fixed-weight fusion with an EKF/UKF or particle filter over Cartesian state.
4. Estimate full velocity instead of projecting radial speed onto direction.
5. Replace grid AoA with covariance-based methods if multiple snapshots or richer apertures are available.
6. Generalize the implementation beyond one target and fixed compile-time dimensions.
7. Validate against `truthAtTime(...)` / `truth_metrics` using range, Doppler, angle, and Cartesian error.

---

## 11) File Map (Primary References)

- Scenario/config: `lib/problem_description.h`
- Noise constants: `lib/problem_config.h`
- Dynamics: `lib/dynamics.cpp`
- Environment/clutter: `lib/environment.cpp`
- Simulator core: `lib/simulation.h`, `lib/simulation.cpp`
- Streaming tracker API: `src/fmcw_tracker.h`
- Streaming tracker implementation: `src/fmcw_tracker.cpp`
- Data generation executable: `src/main.cpp`

---

## 12) Quick Formula Sheet

\[
\lambda = \frac{c}{f_c}
\]

\[
T_c = \frac{N_s}{f_s},\quad \mu = \frac{B}{T_c}
\]

\[
R = -\frac{c f_b}{2\mu}
\]

\[
f_D = \frac{2v_r}{\lambda},\quad v_r = \frac{\lambda f_D}{2}
\]

\[
R_{corr} = R_{raw} - \frac{c}{2\mu}f_D
\]

\[
\hat{\mathbf{p}} = \hat{R}\hat{u},\quad \hat{\mathbf{v}} = \hat{v}_r\hat{u}
\]

\[
a_m(\hat{u}) = e^{-j\frac{2\pi}{\lambda}\hat{u}^T\mathbf{r}_m}
\]
