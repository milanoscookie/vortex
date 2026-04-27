# Simulator Implementation Reference (`lib/*` + `src/main.cpp`)

This document explains how the current C++ simulator is implemented, file by file, and includes the equations used to generate radar data.

It focuses on:

- `lib/problem_config.h`
- `lib/problem_description.h`
- `lib/probe_element.h`
- `lib/probe.h`
- `lib/dynamics.h` / `lib/dynamics.cpp`
- `lib/environment.h` / `lib/environment.cpp`
- `lib/simulation.h` / `lib/simulation.cpp`
- `src/main.cpp`

`lib/utils/*` is summarized at the end.

---

## 1) High-Level Architecture

The simulator is split into three conceptual layers:

1. **Scenario configuration** (`problem_*` headers)
2. **Physics/signal generation** (`dynamics`, `environment`, `simulation`)
3. **Dataset export executable** (`src/main.cpp`)

At runtime, `main.cpp` constructs a `RadarSimulator`, steps sample-by-sample for every chirp, and writes:

- `output/tx_chirp.bin`
- `output/rx_burst.bin`
- `output/truth.csv`
- `output/metadata.csv`

---

## 2) Constants and Scenario Config

## 2.1 `lib/problem_config.h`

Defines centralized noise constants:

- `kDefaultNoiseStddev`
- `kNoiseDistributionMean`
- `kNoiseDistributionStddev`
- `kComplexNoiseQuadratureScale = 1/sqrt(2)`

This keeps noise behavior configurable without hardcoding in `simulation.cpp`.

## 2.2 `lib/problem_description.h`

This is the main scenario definition.

Key compile-time constants:

- `kRadarBlockSize`
- `kRadarChirpCount`
- `kProbeNumX`, `kProbeNumY`

Key structs:

- `CarDynamicsModel`
  - initial position, base velocity, optional vertical bounce, reflectivity
- `Config`
  - waveform + propagation parameters (`sample_rate_hz`, `carrier_hz`, `bandwidth_hz`, `speed_of_light_mps`, `noise_stddev`, etc.)
- `FloorplaneClutterConfig`
  - enables/configures static floor clutter model
- `ProblemDescription`
  - bundles all active defaults as `kDefaultProblemDescription`

Important derived quantities used across the simulator:

\[
\lambda = \frac{c}{f_c},\quad T_c = \frac{N_s}{f_s},\quad \mu = \frac{B}{T_c}
\]

where:

- \(c\): speed of light
- \(f_c\): carrier frequency
- \(f_s\): sample rate
- \(N_s\): block size (samples/chirp)
- \(B\): chirp bandwidth
- \(T_c\): chirp duration
- \(\mu\): chirp slope

---

## 3) Probe / Array Geometry

## 3.1 `lib/probe_element.h`

Defines a single RX element state:

- `enabled` (active/dead)
- `weight` (apodization)
- `delay` (integer sample delay)

## 3.2 `lib/probe.h`

Templated 2D grid probe: `Probe<NumX, NumY>`.

Element position equation (for index `(ix, iy)`):

\[
x_{off} = (ix - 0.5(NumX-1))\,d_x
\]
\[
y_{off} = (iy - 0.5(NumY-1))\,d_y
\]

Position is center + offset in XY plane:

\[
\mathbf{r}_{ix,iy} = \mathbf{r}_{center} + [x_{off}, y_{off}, 0]^T
\]

In `main.cpp`, spacing is set to half wavelength:

\[
d_x = d_y = \frac{\lambda}{2}
\]

---

## 4) Target Dynamics

## 4.1 `lib/dynamics.h` / `lib/dynamics.cpp`

`CarDynamics` is deterministic kinematics + optional bounce.

Position:

\[
\mathbf{p}(t) = \mathbf{p}_0 + \mathbf{v}_0 t + [0,0,A\sin(2\pi f_b t + \phi_b)]^T
\]

Velocity:

\[
\mathbf{v}(t) = \mathbf{v}_0 + [0,0,2\pi f_b A\cos(2\pi f_b t + \phi_b)]^T
\]

State output includes center, velocity, yaw, dimensions, reflectivity.

---

## 5) Static Environment / Clutter

## 5.1 `lib/environment.h` / `lib/environment.cpp`

`Environment` currently models an optional static floorplane return.

When enabled, it computes a fixed beat-tone at configured range `range_m`:

1. Chirp duration and slope:

\[
T_c = \frac{N_s}{f_s},\quad \mu = \frac{B}{T_c}
\]

2. Beat frequency for static reflector:

\[
f_{b, floor} = \frac{2\mu R_f}{c}
\]

3. Amplitude law:

\[
A_f = A_{ref}\left(\frac{R_{ref}}{R_f}\right)^\alpha
\]

4. Base phase:

\[
\phi_{0,f} = -\frac{4\pi R_f}{\lambda} + \phi_{cfg}
\]

5. Time-domain clutter sample:

\[
s_f(t) = A_f\,e^{j(2\pi f_{b,floor} t + \phi_{0,f})}
\]

This clutter term is added to each active RX channel during simulation step.

---

## 6) Core Radar Simulator

## 6.1 `lib/simulation.h`

`RadarSimulator` owns:

- scenario config (`config_`)
- target dynamics (`dynamics_`)
- environment clutter model (`environment_`)
- TX history buffer (`tx_history_`) for delayed sampling
- RNG/distribution for additive noise
- sample clock/index and last truth metrics

Public APIs:

- `step(output, tx_sample)` (default probe)
- templated `step(probe, output, tx_sample)`
- `lastMetrics()` for truth export

The templated `step` in the header performs most per-sample physics.

## 6.2 `lib/simulation.cpp`

Implements constructors and helpers (`metricsAt`, delayed TX interpolation, noise sampling).

### TX history sizing

History is sized to cover max round-trip delay:

\[
N_{hist} = \lceil (2R_{max}/c) f_s \rceil + 4
\]

### Delayed TX interpolation

Given delay \(\tau\), delayed sample index is:

\[
i_d = i_{now} - \tau f_s
\]

Linear interpolation between floor and ceil indices:

\[
s(i_d) = (1-\alpha)s[i_0] + \alpha s[i_1]
\]

where \(\alpha = i_d - \lfloor i_d \rfloor\).

### Noise

If `noise_stddev <= 0`, no noise is added.

Else:

\[
\sigma = \texttt{noise\_stddev}\cdot\frac{1}{\sqrt{2}}
\]

\[
n = (\mathcal{N}(0,1)\sigma) + j(\mathcal{N}(0,1)\sigma)
\]

---

## 7) Per-Sample Signal Equation in `RadarSimulator::step`

For each active RX element:

1. Optional clutter + additive noise.
2. Compute displacement to target:

\[
\Delta\mathbf{r} = \mathbf{p}_{target}(t) - \mathbf{r}_{rx}
\]

3. Range:

\[
R = \|\Delta\mathbf{r}\|
\]

4. LOS unit vector:

\[
\hat{u} = \frac{\Delta\mathbf{r}}{\max(R, R_{min})}
\]

5. Delay:

\[
\tau = \frac{2R}{c} + \tau_{elem}
\]

6. Path gain and phase:

\[
g(R) = \frac{G}{R^2},\quad \phi_{prop} = -\frac{4\pi R}{\lambda}
\]

7. Radial velocity and Doppler phase:

\[
v_r = \mathbf{v}\cdot\hat{u},\quad \phi_D(t)=\frac{4\pi v_r t}{\lambda}
\]

8. Final per-element received signal contribution:

\[
s_{rx}(t)=w\cdot s_{tx}(t-\tau)\cdot \Gamma\cdot g(R)e^{j\phi_{prop}}\cdot e^{j\phi_D(t)}
\]

where \(\Gamma\) is complex reflectivity.

All element contributions are written to output vector for this time sample.

---

## 8) Truth Metrics (`metricsAt`)

At time `t`, simulator logs:

- `position_m = p(t)`
- `velocity_mps = v(t)`
- `range_m = ||p(t)||`
- `line_of_sight = p(t)/max(range, min_range_m)`
- `delay_s = 2*range/c`
- `radial_velocity_mps = v(t) dot line_of_sight`
- `doppler_hz = 2*radial_velocity/lambda`

These are exported into `truth.csv` once per chirp.

---

## 9) Dataset Generation in `src/main.cpp`

`main.cpp` orchestrates the whole simulation run.

### 9.1 Chirp synthesis

Complex reference chirp uses:

\[
s_{tx}(t)=e^{j2\pi(f_0 t + \frac{1}{2}\mu t^2)},\quad f_0=-B/2
\]

generated for `kBlockSize` samples.

### 9.2 Probe construction

Probe uses half-wavelength spacing at current carrier:

\[
d = \frac{\lambda}{2}
\]

### 9.3 Main simulation loops

Outer loop over chirps, inner loop over fast-time samples:

- call `simulator.step(probe, rx_sample, tx_chirp[sample_index])`
- append all element complex samples to `rx_burst.bin`

After each chirp:

- append one truth row from `simulator.lastMetrics()`

### 9.4 Metadata export

Writes key-value metadata used by Python tools (waveform, array geometry, file paths, scenario settings).

---

## 10) Build Integration (`lib/CMakeLists.txt`)

The `Vortex` executable directly builds in these simulator sources:

- `dynamics.cpp`
- `environment.cpp`
- `simulation.cpp`

with include dirs exposing `lib/` and Eigen path.

---

## 11) `lib/utils/*` Summary

`lib/utils` includes reusable DSP and utility primitives (`RingBuffer`, filters, linear systems, queues, coordinate helpers).

For the current simulator path used by `Vortex`, the most relevant dependency is coordinate type support via `coords.h` used in probe geometry.
Other utilities are generally infrastructure/helpers for broader project work and tests.

---

## 12) Practical Notes for Algorithm Developers

1. The simulator is currently single-point-target dominant (one reflected point model with optional static clutter).
2. Truth is generated from the same physics equations listed above, so it is internally consistent.
3. Signal realism is mostly controlled by:
   - `noise_stddev`
   - floorplane clutter settings
   - waveform (`sample_rate_hz`, `bandwidth_hz`, `carrier_hz`, block/chirp count)
4. `rx_burst.bin` layout is chirp-major then sample-major then element-major (`complex64`).
5. If you modify waveform parameters, always ensure tracker assumptions (chirp duration, slope, FFT sizing, CPI size) are updated consistently.
