# Multi-Car Radar Tracking Algorithm Reference

This document is a design reference for extending the current FMCW radar pipeline to track multiple cars simultaneously.
It is written to stay consistent with the signal conventions and implementation realities already documented in `docs/car_tracking_algorithm_reference.md`, while describing a detector-cluster-associate-smooth-manage architecture suitable for multiple targets without relying on an Extended Kalman Filter (EKF).

## 1) Scope and Design Goal

The goal is to estimate a time-varying set of car trajectories

\[
\mathcal{T}_k = \{\tau_k^{(1)}, \tau_k^{(2)}, \dots, \tau_k^{(N_k)}\}
\]

at each CPI index \(k\), where \(N_k\) is unknown and may change with time.

Each trajectory should maintain:

- a persistent track ID
- a position and velocity estimate
- a lifecycle state such as tentative, confirmed, or deleted

The design here assumes a classic detect-then-track pipeline:

1. dechirp and build the range-Doppler data cube
2. detect significant target cells
3. cluster cells into measurement candidates
4. estimate angle for each candidate
5. associate candidates to existing tracks
6. update per-track position and velocity estimates
7. create new tracks and delete stale tracks

---

## 2) Why the Current Tracker Does Not Generalize Directly

The current C++ tracker is built around a single best hypothesis per CPI. In particular:

- `src/fmcw_tracker.cpp` selects a small set of top range-Doppler candidates, then keeps a single joint RD+AoA winner.
- `src/fmcw_tracker.cpp:866` explicitly throws if more than one car is present.
- `src/fmcw_tracker.h` stores only one `TrackingState` and one stream of `BatchResult` entries.

That structure is appropriate for one target, but multi-car tracking needs a set-valued measurement model and a set of live track states.

---

## 3) Multi-Target Signal Model

Let there be \(N_t\) cars visible during a CPI. For receive element \(m\), the received signal is the superposition

\[
s_{rx,m}(t) = \sum_{q=1}^{N_t} w_m\,s_{tx}(t-\tau_{q,m})\,\Gamma_q\,e^{j\phi_{prop,q,m}}\,e^{j\phi_{D,q}(t)} + n_m(t).
\]

For target \(q\):

- position: \(\mathbf{p}_q(t) = [x_q,y_q,z_q]^T\)
- velocity: \(\mathbf{v}_q(t) = [v_{x,q},v_{y,q},v_{z,q}]^T\)
- LOS unit vector:

\[
\hat{u}_q = \frac{\mathbf{p}_q}{\|\mathbf{p}_q\|}
\]

- radial velocity:

\[
v_{r,q} = \mathbf{v}_q^T \hat{u}_q
\]

- range to element \(m\):

\[
R_{q,m} = \|\mathbf{p}_q - \mathbf{r}_m\|
\]

- two-way delay:

\[
\tau_{q,m} = \frac{2R_{q,m}}{c}
\]

- propagation phase:

\[
\phi_{prop,q,m} = -\frac{4\pi R_{q,m}}{\lambda}
\]

- Doppler phase:

\[
\phi_{D,q}(t) = \frac{4\pi v_{r,q} t}{\lambda}.
\]

After dechirp, FFT, and beamforming, the measured RD map is no longer a single isolated peak. It is a sum of target responses, sidelobes, leakage, noise, and clutter. The algorithm must therefore reason over sets of detections rather than a single global maximum.

---

## 4) Processing Stages That Should Stay Compatible With the Current Repo

The following front-end conventions should remain unchanged so the algorithm stays aligned with the existing simulator and tracker:

- dechirp with the conjugated transmit chirp
- blank the fast-time wrap guard prefix
- perform range FFT with the same down-chirp sign convention
- keep the negative-frequency range bins used by the current tracker
- use the same range mapping

\[
R = -\frac{c f_b}{2\mu}
\]

- use the same Doppler FFT sign convention and velocity mapping

\[
f_D = -f_{FFT,shifted}, \qquad v_r = \frac{\lambda f_D}{2}
\]

- keep explicit range-Doppler coupling correction

\[
R_{corr} = R_{raw} - \frac{c}{2\mu} f_D.
\]

Those choices matter because downstream association and filtering will fail if range or Doppler signs are inconsistent between prediction and measurement.

---

## 5) Range-Doppler Detection Model

Let the coherently processed and optionally clutter-suppressed data cube be

\[
X[d,r,m]
\]

after Doppler FFT for Doppler bin \(d\), range bin \(r\), and receive channel \(m\).

Define noncoherent power

\[
P[d,r] = \frac{1}{M}\sum_{m=1}^{M} |X[d,r,m]|^2.
\]

The single-target tracker effectively chooses one maximizer of a prediction-biased score. A multi-target tracker instead forms a detection set

\[
\mathcal{D}_k = \{(d_i,r_i,p_i)\}_{i=1}^{M_k}
\]

where each element passes a threshold test such as

\[
P[d_i,r_i] > T[d_i,r_i].
\]

The threshold can be global in a simple implementation, but a local adaptive threshold is usually better because noise and clutter are not spatially uniform across the RD plane.

### 5.1 Practical detector recommendation

For this repo, a practical baseline is:

1. apply zero-Doppler exclusion bins as already done
2. compute the RD power map
3. estimate a local noise floor around each cell under test
4. threshold the cell against a desired false-alarm ratio
5. keep only local peaks or clustered above-threshold regions

Even if the first implementation uses a simple threshold, the doc should treat the detector output as a set of candidate cells, not one winner.

---

## 6) Clustering Detections Into Object Candidates

One physical car generally occupies multiple adjacent RD bins because of finite resolution, window sidelobes, range migration, and imperfect bin alignment. Therefore the raw detection set

\[
\mathcal{D}_k
\]

must be grouped into candidate objects

\[
\mathcal{C}_k = \{C_k^{(1)}, C_k^{(2)}, \dots, C_k^{(J_k)}\}.
\]

This document does not require a specific named clustering algorithm. A radar-friendly baseline is neighborhood grouping in the discrete RD grid.

Define a neighborhood relation between two detections \(i\) and \(j\):

\[
|r_i-r_j| \le g_r, \qquad |d_i-d_j| \le g_d
\]

for chosen range and Doppler grouping widths \(g_r\) and \(g_d\). Two detections belong to the same cluster if they are connected through that relation.

For each cluster \(C\), define either:

- the peak cell estimator

\[
(\hat r, \hat d) = \arg\max_{(r,d) \in C} P[d,r]
\]

- or the power-weighted centroid estimator

\[
\hat r = \frac{\sum_{(r,d)\in C} r\,P[d,r]}{\sum_{(r,d)\in C} P[d,r]},
\qquad
\hat d = \frac{\sum_{(r,d)\in C} d\,P[d,r]}{\sum_{(r,d)\in C} P[d,r]}.
\]

The peak estimator is simpler and more stable in low SNR. The centroid estimator better captures extended blobs but is more sensitive to sidelobes and merged objects.

### 6.1 Why clustering matters

This stage is what turns "many blips" into "one car candidate." A single vehicle may light up several adjacent bins in range and Doppler, so the tracker should associate and smooth one cluster-level measurement, not every threshold crossing independently.

---

## 7) Angle Estimation Per Candidate

After RD clustering, each object candidate should get its own AoA estimate. This is critical: angle estimation should be tied to a specific detection hypothesis, not run once globally for the scene.

For cluster \(C^{(j)}\), choose a representative RD cell or a small coherent neighborhood and form the array snapshot

\[
\mathbf{x}^{(j)} \in \mathbb{C}^{M}.
\]

For hypothesized direction \(\hat{u}(\mathrm{az},\mathrm{el})\), the steering vector is

\[
a_m(\hat{u}) = e^{-j\frac{2\pi}{\lambda}\hat{u}^T \mathbf{r}_m}.
\]

Stacking over all elements gives \(\mathbf{a}(\hat{u})\), and the beamformer score is

\[
S(\hat{u}) = |\mathbf{a}(\hat{u})^H \mathbf{x}^{(j)}|^2.
\]

Then

\[
\hat{u}^{(j)} = \arg\max_{\hat{u}} S(\hat{u}).
\]

Equivalent angular outputs may be written as azimuth and elevation

\[
\hat z_{ang}^{(j)} = [\hat \theta^{(j)}, \hat \phi^{(j)}]^T.
\]

### 7.1 Implementation implications

- AoA cost scales with the number of detection clusters, not just with one selected RD bin.
- A coarse-to-fine angular search like the current implementation remains useful.
- If two cars overlap in the same RD cell, simple beamforming can merge them; the document should acknowledge this as a failure mode.

---

## 8) Measurement Vector for Each Candidate

After RD and AoA estimation, each cluster yields one measurement vector

\[
z_k^{(j)} =
\begin{bmatrix}
R_k^{(j)} \\
f_{D,k}^{(j)} \\
\theta_k^{(j)} \\
\phi_k^{(j)}
\end{bmatrix}
\]

or equivalently radial velocity instead of Doppler frequency:

\[
z_k^{(j)} =
\begin{bmatrix}
R_k^{(j)} \\
v_{r,k}^{(j)} \\
\theta_k^{(j)} \\
\phi_k^{(j)}
\end{bmatrix}.
\]

Associated metadata should include:

- peak or integrated power
- cluster size in RD bins
- interpolation offsets
- optional spread metrics derived from cluster shape

The full measurement set for CPI \(k\) is then

\[
\mathcal{Z}_k = \{z_k^{(1)}, z_k^{(2)}, \dots, z_k^{(M_k)}\}.
\]

---

## 9) Track State Model

Each live car track should maintain a Cartesian kinematic state such as

\[
\mathbf{x}_k =
\begin{bmatrix}
p_x & p_y & p_z & v_x & v_y & v_z
\end{bmatrix}^T.
\]

For a constant-velocity model with CPI spacing \(\Delta t\), prediction is

\[
\mathbf{x}_{k|k-1} = F(\Delta t)\,\mathbf{x}_{k-1|k-1}
\]

with

\[
F(\Delta t) =
\begin{bmatrix}
1&0&0&\Delta t&0&0 \\
0&1&0&0&\Delta t&0 \\
0&0&1&0&0&\Delta t \\
0&0&0&1&0&0 \\
0&0&0&0&1&0 \\
0&0&0&0&0&1
\end{bmatrix}.
\]

Expanded component-wise, the prediction step is simply

\[
\hat{\mathbf{p}}_{k|k-1} = \hat{\mathbf{p}}_{k-1|k-1} + \Delta t\,\hat{\mathbf{v}}_{k-1|k-1},
\qquad
\hat{\mathbf{v}}_{k|k-1} = \hat{\mathbf{v}}_{k-1|k-1}.
\]

This is intentionally lightweight: it says "move the track forward using its last known velocity" and handles uncertainty with fixed gates, fixed smoothing gains, and lifecycle logic rather than with a propagated covariance matrix.

### 9.1 Why Cartesian state is preferable

The current single-car tracker stores direction and radial speed, then projects the result back to Cartesian coordinates. That is insufficient for multi-car tracking because:

- two cars can have similar range and radial speed but different tangential motion
- association quality depends on predicted spatial separation
- track continuity across crossings is much better with full Cartesian kinematics

---

## 10) Measurement Mapping and Predicted Observables

Given a predicted Cartesian state

\[
\mathbf{x} = [p_x,p_y,p_z,v_x,v_y,v_z]^T,
\]

the predicted observables are

\[
h(\mathbf{x}) =
\begin{bmatrix}
R(\mathbf{x}) \\
f_D(\mathbf{x}) \\
\theta(\mathbf{x}) \\
\phi(\mathbf{x})
\end{bmatrix}
\]

with

\[
R(\mathbf{x}) = \sqrt{p_x^2+p_y^2+p_z^2},
\]

\[
\hat{u}(\mathbf{x}) = \frac{1}{R(\mathbf{x})}
\begin{bmatrix}
p_x \\
p_y \\
p_z
\end{bmatrix},
\]

\[
v_r(\mathbf{x}) =
\begin{bmatrix}
v_x & v_y & v_z
\end{bmatrix}
\hat{u}(\mathbf{x}),
\qquad
f_D(\mathbf{x}) = \frac{2}{\lambda} v_r(\mathbf{x}),
\]

\[
\theta(\mathbf{x}) = \operatorname{atan2}(p_y,p_x),
\qquad
\phi(\mathbf{x}) = \operatorname{atan2}(p_z,\sqrt{p_x^2+p_y^2}).
\]

This map is still useful even without an EKF. The tracker uses it to predict where a track should appear in range, Doppler, and angle space before comparing it to new cluster measurements.

---

## 11) Association Between Measurements and Tracks

Let the predicted track set at CPI \(k\) be

\[
\widehat{\mathcal{T}}_{k|k-1} = \{\hat{\mathbf{x}}_{k|k-1}^{(i)}\}_{i=1}^{N_{trk}}.
\]

For track \(i\), first map its predicted Cartesian state into the measurement domain:

\[
\hat z_{k|k-1}^{(i)} = h(\hat{\mathbf{x}}_{k|k-1}^{(i)}).
\]

For measurement \(j\), define residual components such as

\[
\Delta R_{ij} = R_k^{(j)} - \hat R_{k|k-1}^{(i)},
\qquad
\Delta v_{r,ij} = v_{r,k}^{(j)} - \hat v_{r,k|k-1}^{(i)}
\]

along with wrapped angle residuals \(\Delta \theta_{ij}\) and \(\Delta \phi_{ij}\).

Without an EKF, the default gating statistic should be geometric rather than covariance-based. A practical weighted distance is

\[
d_{ij}^2 =
w_R\,(\Delta R_{ij})^2
+ w_v\,(\Delta v_{r,ij})^2
+ w_\theta\,(\operatorname{wrap}(\Delta \theta_{ij}))^2
+ w_\phi\,(\operatorname{wrap}(\Delta \phi_{ij}))^2.
\]

In many road scenes, association can be simplified further by converting the measurement into Cartesian position and computing a position-based distance to the predicted track.

Accept the pair as feasible only if it falls inside fixed gates such as:

- range error less than \(G_R\)
- radial-velocity error less than \(G_v\)
- angular error less than \(G_\theta\) and \(G_\phi\)
- or combined weighted distance less than \(G\)

### 11.1 Assignment choices

For sparse scenes, nearest-neighbor assignment is often enough:

\[
j^*(i) = \arg\min_j d_{ij}.
\]

For multiple nearby cars, a global one-to-one assignment is better. Define a cost matrix

\[
C_{ij} = d_{ij}
\]

for gated pairs and a large penalty for infeasible pairs, then solve the minimum-cost bipartite assignment.

This is the right default for a first multi-car implementation in this repo: it is much more stable than greedy matching but far simpler than JPDA or MHT.

### 11.2 Why this works without an EKF

Classical radar trackers often do not need dynamic covariance propagation to work well. If the CPI rate is high enough and the scene is mostly cars moving with moderate acceleration, fixed gates plus nearest-neighbor or Hungarian assignment are usually enough to connect detections reliably.

### 11.3 Angular wrapping and sign consistency

Association logic must normalize angle residuals into consistent wrapped intervals, and it must use the same range and Doppler sign conventions as the FFT front end. Small sign mismatches can silently destroy track continuity.

---

## 12) Track Update with an Alpha-Beta Filter

Once a measurement \(z_k^{(j)}\) is assigned to track \(i\), the tracker should update position and velocity with fixed gains rather than a Kalman gain.

For a Cartesian position measurement \(\mathbf{p}_{meas}^{(j)}\) and predicted state

\[
\hat{\mathbf{p}}^- = \hat{\mathbf{p}}_{k|k-1}^{(i)},
\qquad
\hat{\mathbf{v}}^- = \hat{\mathbf{v}}_{k|k-1}^{(i)},
\]

define residual

\[
\mathbf{r}_k = \mathbf{p}_{meas}^{(j)} - \hat{\mathbf{p}}^-.
\]

Then the alpha-beta update is

\[
\hat{\mathbf{p}}^+ = \hat{\mathbf{p}}^- + \alpha\,\mathbf{r}_k
\]

\[
\hat{\mathbf{v}}^+ = \hat{\mathbf{v}}^- + \frac{\beta}{\Delta t}\,\mathbf{r}_k.
\]

Here \(\alpha\) controls how strongly the track snaps toward the new measurement, while \(\beta\) controls how aggressively velocity is corrected.

Typical interpretation:

- high \(\alpha\): fast response, less smoothing
- low \(\alpha\): more stable tracks, more lag
- high \(\beta\): velocity adapts quickly
- low \(\beta\): velocity stays conservative

If no measurement is assigned, the track should coast:

\[
\hat{\mathbf{p}}_{k|k}^{(i)} = \hat{\mathbf{p}}_{k|k-1}^{(i)},
\qquad
\hat{\mathbf{v}}_{k|k}^{(i)} = \hat{\mathbf{v}}_{k|k-1}^{(i)}
\]

with a missed-detection counter incremented.

---

## 13) Track Birth, Confirmation, and Deletion

Multi-target tracking is not only state estimation; it is also hypothesis management.

### 13.1 Birth

Any measurement not assigned to an existing track may initialize a tentative track. Its initial state can be formed from range and angles:

\[
\hat{\mathbf{p}}_0 = R
\begin{bmatrix}
\cos\phi\cos\theta \\
\cos\phi\sin\theta \\
\sin\phi
\end{bmatrix}
\]

and from radial velocity projected along LOS:

\[
\hat{\mathbf{v}}_0 = v_r
\begin{bmatrix}
\cos\phi\cos\theta \\
\cos\phi\sin\theta \\
\sin\phi
\end{bmatrix}.
\]

This is only a partial velocity initialization, but it is acceptable for a lightweight tracker. The tangential velocity components can start from zero or LOS-projected speed and converge over subsequent alpha-beta updates.

### 13.2 Confirmation

A tentative track becomes confirmed after repeated support, for example if it receives at least \(m\) assignments within \(n\) consecutive CPIs.

### 13.3 Deletion

A track is deleted after too many consecutive misses.

These rules are simple, but they are essential to prevent false tracks from persisting and to let real targets survive short dropouts.

---

## 14) Important Failure Modes

### 14.1 Two cars in one RD cluster

If two cars are close in range and Doppler, the detector may return one merged blob. Then the filter receives only one measurement even though two tracks exist. This can cause coalescence or track loss.

### 14.2 Crossing targets

When two cars cross in angle or range-Doppler space, greedy association can swap IDs. Global assignment plus a full Cartesian motion model reduces this risk.

### 14.3 Near-zero Doppler targets

Zero-Doppler suppression helps reject clutter but can remove slow vehicles. The guard width must be chosen carefully.

### 14.4 Angular ambiguity

With a limited array aperture, the AoA surface may be broad or ambiguous. Then the tracker should use looser angular gates and avoid over-weighting angle in the association score.

### 14.5 Sidelobe births

Strong targets can create sidelobe detections that look like weak new cars. Confirmation logic and cluster-shape heuristics are useful defenses.

---

## 15) Implementation Blueprint for This Repo

To evolve the current codebase toward this design, the main structural changes are:

1. Replace the single best-candidate output of `processCurrentWindow(...)` with a vector of per-CPI measurement candidates.
2. Replace the single `TrackingState tracking_state_` in `src/fmcw_tracker.h` with a container of active track objects.
3. Replace one `BatchResult` stream with either:
   - one per-track history, or
   - one scene-level batch object containing all detections and all track states for that CPI.
4. Keep the current FFT, windowing, steering-vector, and sign-convention code paths as shared front-end machinery.
5. Insert explicit stages for detection, cluster formation, per-cluster AoA, association, and track management.
6. Keep the tracking back end lightweight: constant-velocity prediction, fixed gates, assignment, alpha-beta update, and lifecycle counters.

### 15.1 Data-structure hints

Useful additions would be:

- `DetectionCell` for thresholded RD cells
- `MeasurementCandidate` for cluster-level `range/doppler/angle/power/spread`
- `Track` for `id/state/status/hit_count/miss_count/history`
- `SceneBatchResult` for all detections and tracks produced from one CPI

The existing `BatchResult` type is too single-target-specific to serve as the main multi-car output without substantial reshaping.

### 15.2 Compute scaling

If \(J_k\) candidates survive clustering and the angle grid has \(N_{az}N_{el}\) points, naive AoA cost scales roughly as

\[
\mathcal{O}(J_k M N_{az} N_{el}).
\]

That makes early pruning, coarse-to-fine angle search, and reasonable detector thresholds important implementation choices.

---

## 16) Validation Metrics

A multi-car tracker should not be judged only by instantaneous range error. Useful metrics include:

- per-track Cartesian RMSE
- range, Doppler, azimuth, and elevation RMSE
- track confirmation latency
- missed-track rate
- false-track rate
- ID-switch count
- track fragmentation count

For a scene-level metric, compare the estimated track set against truth as a set rather than only one trajectory at a time.

---

## 17) Recommended Baseline Algorithm

A practical baseline consistent with this document is:

1. Keep the current dechirp, range FFT, Doppler FFT, clutter suppression, and sign conventions.
2. Form an RD power map for each CPI.
3. Detect significant cells with CFAR or another local thresholding method.
4. Group neighboring detections into RD clusters.
5. Extract one range-Doppler hypothesis per cluster, preferably by centroid or peak selection.
6. Estimate AoA separately for each cluster.
7. Convert each cluster into one measurement candidate.
8. Predict all live tracks forward with a Cartesian constant-velocity model.
9. Gate and assign measurements to tracks with Euclidean or weighted geometric distance and a one-to-one assignment method.
10. Update matched tracks with an alpha-beta filter.
11. Coast unmatched tracks, spawn tentative tracks from unmatched measurements, and confirm or delete tracks based on hit and miss counts.

This baseline is intentionally mechanical: detect, cluster, connect the dots, smooth, and manage track life. It avoids the matrix-heavy EKF while still matching how many practical multi-target radar trackers are built.

---

## 18) Quick Formula Sheet

\[
R = -\frac{c f_b}{2\mu}, \qquad v_r = \frac{\lambda f_D}{2}, \qquad R_{corr} = R_{raw} - \frac{c}{2\mu} f_D
\]

\[
P[d,r] = \frac{1}{M}\sum_{m=1}^{M}|X[d,r,m]|^2
\]

\[
d_{ij}^2 =
w_R\,(\Delta R_{ij})^2
+ w_v\,(\Delta v_{r,ij})^2
+ w_\theta\,(\operatorname{wrap}(\Delta \theta_{ij}))^2
+ w_\phi\,(\operatorname{wrap}(\Delta \phi_{ij}))^2
\]

\[
\mathbf{x}_k = [p_x,p_y,p_z,v_x,v_y,v_z]^T
\]

\[
\hat{\mathbf{p}}_{k|k-1} = \hat{\mathbf{p}}_{k-1|k-1} + \Delta t\,\hat{\mathbf{v}}_{k-1|k-1}, \qquad \hat{\mathbf{v}}_{k|k-1} = \hat{\mathbf{v}}_{k-1|k-1}
\]

\[
\hat{\mathbf{p}}^+ = \hat{\mathbf{p}}^- + \alpha\,(\mathbf{p}_{meas}-\hat{\mathbf{p}}^-)
\]

\[
\hat{\mathbf{v}}^+ = \hat{\mathbf{v}}^- + \frac{\beta}{\Delta t}(\mathbf{p}_{meas}-\hat{\mathbf{p}}^-)
\]

\[
\hat{\mathbf{p}} = R
\begin{bmatrix}
\cos\phi\cos\theta \\
\cos\phi\sin\theta \\
\sin\phi
\end{bmatrix}
\]
