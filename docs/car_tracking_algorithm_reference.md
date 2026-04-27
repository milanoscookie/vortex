# Radar Car-Tracking Algorithm Reference

This document is a detailed technical reference for the simulator + signal-processing stack in this repository.
It is intended to provide enough equations, data formats, and implementation details for someone to build a car-tracking algorithm that is consistent with the current codebase.

## 1) End-to-End Pipeline

The current pipeline is:

1. **Scenario definition** (`lib/problem_description.h`)
2. **Signal generation / simulation** (`lib/simulation.h`, `lib/simulation.cpp`, `lib/dynamics.cpp`, `lib/environment.cpp`)
3. **Dataset export** (`src/main.cpp`) to:
   - `output/tx_chirp.bin`
   - `output/rx_burst.bin`
   - `output/truth.csv`
   - `output/metadata.csv`
4. **Tracking / estimation** (`tools/fmcw_car_tracker.py`)

The Python tracker reads only exported files, not C++ internals.

---

## 2) Coordinate System and State Definitions

### 2.1 Coordinate convention (as implemented)

- Car state is stored as `(x, y, z)` in meters.
- Probe center is at origin by default.
- In the probe geometry, array elements are spread in local **y** and **z** (with `x=0` for each element in `tools/fmcw_car_tracker.py::element_positions`).
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

## 4) Exported Data Interface (Contract)

### 4.1 `output/tx_chirp.bin`

- Type: contiguous `complex64`
- Shape: `[block_size]`

### 4.2 `output/rx_burst.bin`

- Type: contiguous `complex64`
- Logical shape in tracker:

\[
[\text{chirp\_count},\;\text{block\_size},\;\text{num\_rx}]
\]

- Layout string exported in metadata:

`chirp_major_sample_major_element_major_complex64`

### 4.3 `output/truth.csv`

Columns from `src/main.cpp`:

- `chirp_index`
- `time_s`
- `x_m`, `y_m`, `z_m`
- `range_m`
- `radial_velocity_mps`
- `doppler_hz`

### 4.4 `output/metadata.csv`

Contains waveform, array, and file-path fields used by Python.
Important keys consumed by tracker:

- `sample_rate_hz`
- `carrier_hz`
- `bandwidth_hz`
- `speed_of_light_mps`
- `block_size`
- `chirp_count`
- `chirp_duration_s`
- `probe_num_x`, `probe_num_y`
- `probe_dx_m`, `probe_dy_m`
- `tx_chirp_path`, `rx_burst_path`, `truth_path`

---

## 5) Current Tracker Algorithm (`tools/fmcw_car_tracker.py`)

### 5.1 Dechirp

\[
s_{beat}[k,n,m] = s_{rx}[k,n,m] \cdot s_{tx}^*[n]
\]

Indices:
- \(k\): chirp index
- \(n\): fast-time sample index
- \(m\): receive channel index

### 5.2 Range FFT + positive/negative folding

For each chirp and RX channel:

1. Apply Hann window in fast-time.
2. Compute FFT with `nfft = max(nfft_range_min, next_pow2(num_samples))`.
3. FFT-shift.
4. Fold positive/negative beat-frequency bins by selecting stronger side per chirp.

Range mapping:

\[
R = \frac{c\,f_b}{2\mu}
\]

### 5.3 Optional static clutter suppression

If enabled, remove slow-time mean per range/RX bin:

\[
X_{dyn}[k,r,m] = X[k,r,m] - \frac{1}{K}\sum_{k'}X[k',r,m]
\]

### 5.4 Range-Doppler processing

For each CPI:

1. Apply Hann window in slow-time.
2. FFT along chirp axis.
3. FFT-shift to centered Doppler axis.
4. Noncoherent combine across RX:

\[
P[d,r] = \frac{1}{M}\sum_m |X[d,r,m]|^2
\]

5. Apply masks:
   - Range gate: `min_range_m <= R <= max_range_m`
   - Doppler zero guard (`zero_doppler_guard_bins`)
6. Detect max-power bin.
7. Refine with 1D parabolic interpolation in both dimensions.

Velocity and Doppler:

\[
v_r = \frac{\lambda}{2}f_D
\]

### 5.5 Angle-of-arrival (AoA)

Steering directions are built over azimuth/elevation grids:

\[
\hat{u}(\text{az},\text{el})=
\begin{bmatrix}
\cos(\text{el})\cos(\text{az})\\
\cos(\text{el})\sin(\text{az})\\
\sin(\text{el})
\end{bmatrix}
\]

Monostatic steering phase per array element position \(\mathbf{r}_m\):

\[
a_m = e^{j\frac{4\pi}{\lambda}\hat{u}^T\mathbf{r}_m}
\]

Beam score:

\[
S(\hat{u}) = |a(\hat{u})^H x|^2
\]

Direction estimate is argmax over grid.

### 5.6 Position and y-velocity derivation

- Estimated 3D point from range and direction:

\[
\hat{\mathbf{p}} = \hat{R}\,\hat{u}
\]

- Estimated y-velocity (LOS decomposition):

\[
\hat{v}_y = \hat{v}_r\,\hat{u}_y
\]

This is not a full Cartesian velocity solve; it is radial velocity projected onto estimated direction.

---

## 6) Resolution and Ambiguity Equations

Used in tracker diagnostics:

- Unambiguous radial velocity span:

\[
v_{max} = \frac{\lambda}{4T_c}, \quad v \in [-v_{max}, v_{max}]
\]

- Velocity resolution for CPI length \(M\):

\[
\Delta v = \frac{\lambda}{2MT_c}
\]

- Range resolution (ideal FMCW):

\[
\Delta R \approx \frac{c}{2B}
\]

---

## 7) Parameters That Most Affect Tracking Quality

### 7.1 Simulator-side (`lib/problem_description.h`)

- `simulation.bandwidth_hz`
  - Higher improves range resolution.
- `simulation.sample_rate_hz`
  - With fixed block size, lower sample rate increases chirp duration (`T_c`) and improves Doppler resolution.
  - But lower sample rate reduces beat-frequency Nyquist headroom.
- `simulation.carrier_hz`
  - Higher carrier reduces wavelength and improves Doppler sensitivity for fixed CPI.
- `simulation.noise_stddev`
  - Raises/lower SNR.
- `floorplane.enable_static_floorplane`
  - Strong static clutter can dominate detection if not handled.

### 7.2 Tracker-side (`DetectionConfig`)

- `coherent_processing_interval_chirps`
  - Larger CPI improves Doppler resolution and SNR (coherent gain), but reduces temporal update rate.
- `zero_doppler_guard_bins`
  - Helps avoid static leakage; too large can remove slow-moving target content.
- `min_range_m` / `max_range_m`
  - Strongly constrains search and can avoid wrong peaks.
- `static_clutter_suppression_enable`
  - Useful in static-heavy scenes; harmful if target Doppler is near zero and scene is already clean.
- AoA grid spans and counts
  - Finer grids improve angular granularity at higher compute cost.

---

## 8) Implementation Notes and Caveats

1. **Single-target detector by global max**
   - Current `detect_moving_car_in_cpi` chooses one strongest `(doppler, range)` cell after masking.
   - Multi-target scenes need CFAR / peak clustering / track association.

2. **Range folding logic**
   - Range FFT folds +/- beat bins by power; this is robust to sign conventions but can hide sign-related structure.

3. **AoA from one snapshot per CPI**
   - AoA uses the selected RD cell snapshot; outlier cell selection directly causes angular outliers.

4. **y-velocity interpretation**
   - `est_y_velocity_mps` is projection from radial velocity, not independent Cartesian estimation.

5. **Truth sampling for plots**
   - Truth is interpolated to estimate times using linear interpolation.

6. **Printed simulator message**
   - `main.cpp` prints a floorplane message string regardless of enabled flag; rely on metadata key `static_floorplane_enabled` for actual status.

---

## 9) Minimal Steps to Build a Better Tracker

If building a new algorithm for this simulator, a practical roadmap is:

1. Parse `metadata.csv` and load binaries exactly as defined above.
2. Perform dechirp, range FFT, and Doppler FFT with consistent windowing and axis definitions.
3. Add a robust detector (CFAR in RD map) instead of single global max.
4. Estimate AoA per detection (or with covariance-based methods like MUSIC/Capon).
5. Convert detections to measurement vector \([R, v_r, az, el]\).
6. Track in time with a dynamic model (e.g., EKF/UKF in Cartesian state), fusing measurement uncertainty.
7. Validate against `truth.csv` metrics:
   - RMSE(range)
   - RMSE(radial velocity)
   - angular error
   - Cartesian position/velocity error

---

## 10) File Map (Primary References)

- Scenario/config: `lib/problem_description.h`
- Noise constants: `lib/problem_config.h`
- Dynamics: `lib/dynamics.cpp`
- Environment/clutter: `lib/environment.cpp`
- Simulator core: `lib/simulation.h`, `lib/simulation.cpp`
- Data generation executable: `src/main.cpp`
- Tracking algorithm: `tools/fmcw_car_tracker.py`

---

## 11) Quick Formula Sheet

\[
\lambda = \frac{c}{f_c}
\]

\[
T_c = \frac{N_s}{f_s},\quad \mu = \frac{B}{T_c}
\]

\[
R = \frac{c f_b}{2\mu}
\]

\[
f_D = \frac{2v_r}{\lambda},\quad v_r = \frac{\lambda f_D}{2}
\]

\[
v_{max} = \frac{\lambda}{4T_c},\quad \Delta v = \frac{\lambda}{2MT_c}
\]

\[
\hat{\mathbf{p}} = \hat{R}\hat{u},\quad \hat{v}_y = \hat{v}_r\hat{u}_y
\]
