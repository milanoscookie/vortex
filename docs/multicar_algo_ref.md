# Multi-Car Radar Tracking Algorithm Reference

This document is a design reference for extending the current FMCW radar pipeline to track multiple cars simultaneously.
It is written to stay consistent with the signal conventions and implementation realities already documented in `docs/car_tracking_algorithm_reference.md`, while describing a detector-cluster-associate-smooth-manage architecture suitable for multiple targets without relying on full Extended Kalman Filter (EKF) covariance machinery.

## 1) Scope and Design Goal

The goal is to estimate a time-varying set of car trajectories

\[
\mathcal{T}_k = \{\chi_k^{(1)}, \chi_k^{(2)}, \dots, \chi_k^{(N_k)}\}
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
v_{r,q} = -\mathbf{v}_q^T \hat{u}_q
\]

with the repo sign convention that positive radial velocity means approaching the radar.

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
\phi_{D,q}(t) = \frac{4\pi v_{r,q} t}{\lambda}
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

where \(f_D\) is the repo-corrected Doppler frequency used throughout downstream detection, association, and tracking. The current code path already bakes this sign choice into the Doppler axis by negating the shifted FFT frequency.

- keep explicit range-Doppler coupling correction

\[
R_{corr} = R_{raw} + s_{RD}\,\frac{c}{2\mu} f_D
\]

where `s_RD` is another repo-defined sign constant validated against simulator truth. The current tracker uses the existing sign convention from `docs/car_tracking_algorithm_reference.md`; future multi-car code should preserve that exact convention rather than hardcoding a minus sign independently.

Those choices matter because downstream association and filtering will fail if range or Doppler signs are inconsistent between prediction and measurement.

---

## 5) Range-Doppler Detection Model

Let the coherently processed and optionally clutter-suppressed data cube be

\[
X[d,r,m]
\]

after Doppler FFT for Doppler bin \(d\), range bin \(r\), and receive channel \(m\).
Let \(N_{rx}\) denote the number of receive channels.

Define noncoherent power

\[
P[d,r] = \frac{1}{N_{rx}}\sum_{m=1}^{N_{rx}} |X[d,r,m]|^2.
\]

The single-target tracker effectively chooses one maximizer of a prediction-biased score. A multi-target tracker instead forms a detection set

\[
\mathcal{D}_k = \{(d_i,r_i,p_i)\}_{i=1}^{N_{det,k}}
\]

where each element passes a threshold test such as

\[
P[d_i,r_i] > T[d_i,r_i].
\]

The threshold can be global in a simple implementation, but a local adaptive threshold is usually better because noise and clutter are not spatially uniform across the RD plane.

### 5.1 Practical detector recommendation

For this repo, a practical baseline is:

1. form the noncoherent RD power map
2. apply a 2D CA-CFAR detector
3. use guard cells around the cell under test
4. use training cells outside the guard region to estimate local noise
5. exclude the zero-Doppler clutter band from both detection and noise estimation
6. apply a minimum SNR threshold
7. keep only local maxima before clustering

This gives a concrete codeable baseline. If strong targets corrupt the CFAR training window, the detector can later be upgraded to OS-CFAR or another robust variant.

---

## 6) Clustering Detections Into Object Candidates

One physical car can occupy multiple RD bins because of finite resolution, window sidelobes, range migration, imperfect bin alignment, and multiple scatterers on the vehicle body. In simple scenes this may look like one connected blob, but in general one car can produce several separated detections and two nearby cars can merge into one blob. Therefore the raw detection set

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

The peak estimator is the recommended baseline. If sub-bin refinement is needed, use local interpolation around the peak cell before using a full blob centroid. A centroid over all thresholded cells can be biased by sidelobes or merged targets. If centroiding is used at all, it should preferably operate on thresholded power with the local noise floor removed, or on another compressed weighting, rather than on raw unbounded linear power from the full blob.

If the centroid form is used, the document must explicitly map fractional bin coordinates into physical observables:

\[
\hat f_b = f_b(\hat r), \qquad \hat f_D = f_D(\hat d)
\]

\[
\hat R = -\frac{c\hat f_b}{2\mu}, \qquad \hat v_r = \frac{\lambda \hat f_D}{2}
\]

### 6.1 Why clustering matters

This stage is what turns "many blips" into measurement candidates. For a first implementation, treating one dominant cluster as one car candidate is acceptable, but it is only a simplifying assumption and should be documented as such.

---

## 7) Angle Estimation Per Candidate

After RD clustering, each object candidate should get its own AoA estimate. This is critical: angle estimation should be tied to a specific detection hypothesis, not run once globally for the scene.

For cluster \(C^{(j)}\), the safest baseline is to use the peak RD cell and form the array snapshot

\[
\mathbf{x}^{(j)} = X[\hat d,\hat r,:] \in \mathbb{C}^{N_{rx}}.
\]

For hypothesized direction \(\hat{u}(\mathrm{az},\mathrm{el})\), the steering vector is

\[
a_m(\hat{u}) = e^{-j\psi_m(\hat{u})}.
\]

The phase term \(\psi_m\) must match the phase convention already present in the RD snapshot `X[d,r,m]`. The correct element-dependent phase depends on the simulated TX/RX geometry.

If the simulator uses one common transmitter and multiple receive elements, the element-dependent phase is one-way:

\[
\Delta\phi_m \approx -\frac{2\pi}{\lambda}\hat u^T \mathbf{r}_m.
\]

If each array element is modeled as a colocated monostatic TX/RX element, the element-dependent phase is two-way:

\[
\Delta\phi_m \approx -\frac{4\pi}{\lambda}\hat u^T \mathbf{r}_m.
\]

For this repo, the current single-target implementation precomputes `steering_conj_` with

\[
\psi_m(\hat{u}) = \frac{2\pi}{\lambda}\hat{u}^T \mathbf{r}_m
\]

and scores directions with \(\mathbf{a}(\hat{u})^H\mathbf{x}\). Multi-car code should reuse that exact convention unless the front-end snapshot phasing is intentionally changed and revalidated. The important requirement is consistency, because a sign or factor mismatch can produce mirrored or biased AoA estimates.

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
- Do not coherently sum neighboring RD cells unless their phase is explicitly aligned first.
- A safer multi-cell extension is to sum per-cell beamformer powers noncoherently rather than summing raw snapshots coherently.
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

Here \(R_k^{(j)}\) should denote the range after the repo-consistent range-Doppler coupling correction, not the raw peak-bin range. In other words, downstream association should use \(R_{corr}\), not \(R_{raw}\).

Associated metadata should include:

- peak or integrated power
- cluster size in RD bins
- interpolation offsets
- optional spread metrics derived from cluster shape

The full measurement set for CPI \(k\) is then

\[
\mathcal{Z}_k = \{z_k^{(1)}, z_k^{(2)}, \dots, z_k^{(N_{meas,k})}\}.
\]

---

## 9) Track State Model

Each live car track should maintain a Cartesian kinematic state. The most general form is

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

### 9.1 Recommended state dimension for cars

For road vehicles, a fully unconstrained 6D state is often noisier than necessary. Prefer one of these, in order of practicality:

- known road plane: track \([x,y,v_x,v_y]^T\) and derive or fix \(z\)
- mostly fixed height: track \([x,y,z,v_x,v_y]^T\) with \(v_z = 0\)
- full free-space motion: use \([x,y,z,v_x,v_y,v_z]^T\) only if the simulator truly needs it

The rest of this document keeps the 6D notation for generality, but an automotive multi-car baseline should strongly consider a ground-plane-constrained state.

### 9.2 Why Cartesian state is preferable

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
v_r(\mathbf{x}) = -\mathbf{v}^T\hat{u}(\mathbf{x}),
\qquad
f_D(\mathbf{x}) = \frac{2}{\lambda} v_r(\mathbf{x}),
\]

\[
\theta(\mathbf{x}) = \operatorname{atan2}(p_y,p_x),
\qquad
\phi(\mathbf{x}) = \operatorname{atan2}(p_z,\sqrt{p_x^2+p_y^2}).
\]

This nonlinear map \(h(\mathbf{x})\) is the same kind of measurement function an EKF would use. The difference in this design is not the prediction model; it is the update model. The tracker still predicts into radar measurement space for gating and association, but it uses fixed-gain smoothing instead of covariance-derived Kalman gains.

---

## 11) Association Between Measurements and Tracks

Let the predicted track set at CPI \(k\) be

\[
\widehat{\mathcal{T}}_{k|k-1} = \{\hat{\mathbf{x}}_{k|k-1}^{(i)}\}_{i=1}^{N_{trk,k}}.
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

Without an EKF, the default gating statistic should still be normalized by expected measurement error. A practical choice is

\[
D_{ij}^2 =
\left(\frac{\Delta R_{ij}}{\sigma_R}\right)^2
+ \left(\frac{\Delta v_{r,ij}}{\sigma_v}\right)^2
+ \left(\frac{\operatorname{wrap}(\Delta \theta_{ij})}{\sigma_\theta}\right)^2
+ \left(\frac{\operatorname{wrap}(\Delta \phi_{ij})}{\sigma_\phi}\right)^2.
\]

This makes the gate interpretable: \(D_{ij}^2 < G^2\) behaves like a rough multi-dimensional sigma gate.

In many road scenes, association can be simplified further by converting the measurement into Cartesian position and computing a position-based distance to the predicted track, but this should still respect the much weaker angular observability relative to range and radial velocity.

Accept the pair as feasible only if it falls inside fixed gates such as:

- range error less than \(G_R\)
- radial-velocity error less than \(G_v\)
- angular error less than \(G_\theta\) and \(G_\phi\)
- or combined weighted distance less than \(G\)

### 11.1 Assignment choices

For sparse scenes, nearest-neighbor assignment is often enough:

\[
j^*(i) = \arg\min_j D_{ij}.
\]

For multiple nearby cars, a global one-to-one assignment is better. Define a cost matrix

\[
C_{ij} = D_{ij}
\]

for gated pairs and mark ungated pairs as infeasible. Then solve a one-to-one assignment only over feasible pairs and explicitly handle:

- matched track-measurement pairs
- unmatched tracks that coast
- unmatched measurements that may spawn tentative tracks
- any assignment whose cost exceeds the gate threshold, which must be rejected rather than accepted via a large finite dummy cost

The cost matrix may be rectangular because the number of tracks and measurements usually differs. If the selected assignment solver requires a square matrix, pad with dummy rows or columns, but dummy assignments are bookkeeping only and must never be interpreted as physical matches.

This is the right default for a first multi-car implementation in this repo: it is much more stable than greedy matching but far simpler than JPDA or MHT.

### 11.2 Why this works without an EKF

For controlled simulations and sparse scenes, fixed gates plus nearest-neighbor or Hungarian assignment can work well enough as a baseline. Covariance-based filters become more important when uncertainty changes strongly with range, angle, SNR, clutter level, or target geometry.

### 11.3 Angular wrapping and sign consistency

Association logic must normalize angle residuals into consistent wrapped intervals, and it must use the same range and Doppler sign conventions as the FFT front end. Small sign mismatches can silently destroy track continuity.

For azimuth and elevation residuals, a safe default is

\[
\operatorname{wrap}(\alpha) = \operatorname{atan2}(\sin\alpha, \cos\alpha)
\]

so residuals land in \([ -\pi, \pi ]\) without `fmod` edge-case surprises.

---

## 12) Track Update with an Alpha-Beta Filter

Once a measurement \(z_k^{(j)}\) is assigned to track \(i\), the tracker should update position and velocity with fixed gains rather than a Kalman gain.

Direct Cartesian conversion is convenient but angle noise can create large cross-range errors at long range. Therefore the update should use scalar range residual for the LOS correction and use AoA primarily for cross-range correction.

For measured range \(R_{meas}^{(j)}\), Cartesian position proxy \(\mathbf{p}_{meas}^{(j)}\), measured LOS unit vector \(\hat u_{meas}^{(j)}\), measured radial velocity \(v_{r,meas}^{(j)}\), and predicted state

\[
\hat{\mathbf{p}}^- = \hat{\mathbf{p}}_{k|k-1}^{(i)},
\qquad
\hat{\mathbf{v}}^- = \hat{\mathbf{v}}_{k|k-1}^{(i)},
\]

first compute the predicted range and LOS direction

\[
R_{pred} = \|\hat{\mathbf{p}}^-\|,
\qquad
\hat u_{pred} = \frac{\hat{\mathbf{p}}^-}{\|\hat{\mathbf{p}}^-\|}.
\]

Then form the LOS residual from range alone:

\[
e_R = R_{meas}^{(j)} - R_{pred},
\qquad
\mathbf{r}_{LOS} = e_R\,\hat u_{pred}.
\]

Use AoA only to construct the cross-range correction:

\[
\mathbf{r}_{cart} = \mathbf{p}_{meas}^{(j)} - \hat{\mathbf{p}}^-,
\qquad
\mathbf{r}_{cross} = \mathbf{r}_{cart} - (\mathbf{r}_{cart}^T \hat u_{pred})\hat u_{pred}.
\]

Then use anisotropic fixed gains

\[
\hat{\mathbf{p}}^+ = \hat{\mathbf{p}}^- + \alpha_R\,\mathbf{r}_{LOS} + \alpha_A\,\mathbf{r}_{cross}
\]

\[
v_{r,pred} = -(\hat{\mathbf{v}}^-)^T\hat u_{meas}^{(j)}
\]

\[
e_{v_r} = v_{r,meas}^{(j)} - v_{r,pred}
\]

\[
\hat{\mathbf{v}}^+ = \hat{\mathbf{v}}^- - \gamma_v\,e_{v_r}\,\hat u_{meas}^{(j)} + \frac{\beta_A}{\Delta t}\,\mathbf{r}_{cross}.
\]

Here \(\alpha_R\) and \(\gamma_v\) should usually be stronger than \(\alpha_A\) and \(\beta_A\), because radar observes range and radial velocity much more reliably than cross-range motion from AoA.

Typical interpretation:

- high \(\alpha_R\): fast range-driven LOS position response
- low \(\alpha_A\): suppress cross-range jitter from noisy angle
- high \(\gamma_v\): trust measured radial velocity strongly
- low \(\beta_A\): keep tangential velocity conservative unless repeated geometry supports it

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

Any measurement not assigned to an existing track may initialize a tentative track, but only if it is not inside a birth-suppression gate around an existing confirmed track. This helps avoid duplicate births from sidelobes or split clusters.

Its initial state can be formed from range and angles:

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
\hat{\mathbf{v}}_0 = -v_r
\begin{bmatrix}
\cos\phi\cos\theta \\
\cos\phi\sin\theta \\
\sin\phi
\end{bmatrix}.
\]

This initializes only the LOS velocity component. Tangential velocity is weakly observed from a single CPI and should start conservatively, then be inferred gradually from repeated geometry, or be constrained by a road-plane or motion prior.

### 13.2 Confirmation

A tentative track becomes confirmed after repeated support, for example if it receives at least \(m\) assignments within \(n\) consecutive CPIs. Rules such as 2-of-3 or 3-of-5 are useful practical baselines.

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

Strong targets can create sidelobe detections that look like weak new cars. Confirmation logic, birth suppression near existing tracks, and cluster-shape or power heuristics are useful defenses.

### 14.6 Multiple scatterers on one car

A vehicle can produce several scattering centers with different RD signatures or even different AoA peaks. A first implementation may still collapse this into one dominant cluster per car, but that simplification should be treated as a modeling limit rather than a universal truth.

---

## 15) Implementation Blueprint for This Repo

To evolve the current codebase toward this design, the main structural changes are:

1. Replace the single best-candidate output of `processCurrentWindow(...)` with a vector of per-CPI measurement candidates.
2. Replace the single `TrackingState tracking_state_` in `src/fmcw_tracker.h` with a container of active track objects.
3. Replace one `BatchResult` stream with either:
   - one per-track history, or
   - one scene-level batch object containing all detections and all track states for that CPI.
4. Keep the current FFT, windowing, steering-vector, and sign-convention code paths as shared front-end machinery, and expose the Doppler and range-Doppler sign choices as explicit constants or helpers rather than duplicating sign assumptions.
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
\mathcal{O}(J_k N_{rx} N_{az} N_{el}).
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
5. Extract one range-Doppler hypothesis per cluster, preferably from the peak cell with optional local interpolation.
6. Estimate AoA separately for each cluster using a snapshot convention consistent with the current repo steering vector.
7. Convert each cluster into one measurement candidate.
8. Predict all live tracks forward with a Cartesian constant-velocity model.
9. Gate and assign measurements to tracks with normalized measurement-space distance and a one-to-one assignment method that explicitly supports unmatched tracks and unmatched measurements.
10. Update matched tracks with anisotropic fixed-gain smoothing that uses measured radial velocity directly.
11. Coast unmatched tracks, spawn tentative tracks only outside birth-suppression gates, and confirm or delete tracks based on hit and miss counts.

This baseline is intentionally mechanical: detect, cluster, connect the dots, smooth, and manage track life. It avoids the matrix-heavy EKF while still matching how many practical multi-target radar trackers are built.

---

## 18) Quick Formula Sheet

\[
f_D = -f_{FFT,shifted}, \qquad R = -\frac{c f_b}{2\mu}, \qquad v_r = \frac{\lambda f_D}{2}, \qquad f_D(\mathbf{x}) = \frac{2v_r(\mathbf{x})}{\lambda}, \qquad R_{corr} = R_{raw} + s_{RD}\frac{c}{2\mu} f_D
\]

\[
P[d,r] = \frac{1}{N_{rx}}\sum_{m=1}^{N_{rx}}|X[d,r,m]|^2
\]

\[
D_{ij}^2 =
\left(\frac{\Delta R_{ij}}{\sigma_R}\right)^2
+ \left(\frac{\Delta v_{r,ij}}{\sigma_v}\right)^2
+ \left(\frac{\operatorname{wrap}(\Delta \theta_{ij})}{\sigma_\theta}\right)^2
+ \left(\frac{\operatorname{wrap}(\Delta \phi_{ij})}{\sigma_\phi}\right)^2
\]

\[
\mathbf{x}_k = [p_x,p_y,p_z,v_x,v_y,v_z]^T
\]

\[
\hat{\mathbf{p}}_{k|k-1} = \hat{\mathbf{p}}_{k-1|k-1} + \Delta t\,\hat{\mathbf{v}}_{k-1|k-1}, \qquad \hat{\mathbf{v}}_{k|k-1} = \hat{\mathbf{v}}_{k-1|k-1}
\]

\[
\hat{\mathbf{p}}^+ = \hat{\mathbf{p}}^- + \alpha_R\,\mathbf{r}_{LOS} + \alpha_A\,\mathbf{r}_{cross}
\]

\[
v_{r,pred} = -(\hat{\mathbf{v}}^-)^T\hat u_{meas}
\]

\[
e_{v_r} = v_{r,meas} - v_{r,pred}
\]

\[
\hat{\mathbf{v}}^+ = \hat{\mathbf{v}}^- - \gamma_v\,e_{v_r}\,\hat u_{meas} + \frac{\beta_A}{\Delta t}\mathbf{r}_{cross}
\]

\[
\hat{\mathbf{p}} = R
\begin{bmatrix}
\cos\phi\cos\theta \\
\cos\phi\sin\theta \\
\sin\phi
\end{bmatrix}
\]
