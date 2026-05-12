import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Radar parameters
CARRIER_HZ = 77.0e9
C = 299792458.0
LAMBDA = C / CARRIER_HZ
VERTICAL_BOUNCE_AMPLITUDE_M = 0.15
LOS_Z_EPS = 0.1


def fit_projected_vertical_motion(
    time, radial_displacement_m, los_z, min_freq_hz=40.0, max_freq_hz=250.0
):
    valid = np.isfinite(time) & np.isfinite(radial_displacement_m) & np.isfinite(los_z)
    if np.count_nonzero(valid) < 16:
        return None

    t = time[valid] - time[valid][0]
    y = radial_displacement_m[valid]
    z = los_z[valid]
    if t[-1] <= 0.0:
        return None

    freq_grid = np.linspace(min_freq_hz, max_freq_hz, 2000)
    best = None
    best_mse = np.inf
    for freq_hz in freq_grid:
        omega_t = 2.0 * np.pi * freq_hz * t
        design = np.column_stack((z * np.sin(omega_t), z * np.cos(omega_t)))
        coeffs, _, _, _ = np.linalg.lstsq(design, y, rcond=None)
        fit = design @ coeffs
        mse = np.mean((y - fit) ** 2)
        if mse < best_mse:
            best_mse = mse
            best = {
                "freq_hz": float(freq_hz),
                "amplitude_m": float(np.hypot(coeffs[0], coeffs[1])),
                "phase_rad": float(np.arctan2(coeffs[1], coeffs[0])),
            }

    if best is None:
        return None

    omega_t = 2.0 * np.pi * best["freq_hz"] * (time - time[0])
    vertical_fit_m = best["amplitude_m"] * np.sin(omega_t + best["phase_rad"])
    radial_fit_m = vertical_fit_m * los_z
    best["vertical_fit_m"] = vertical_fit_m
    best["radial_fit_m"] = radial_fit_m
    return best


def plot_microdoppler_analysis(csv_path):
    if not os.path.exists(csv_path):
        print(f"File {csv_path} not found.")
        return

    df = pd.read_csv(csv_path)
    required_cols = [
        "time_s",
        "raw_unwrapped_phase",
        "expected_bulk_phase",
        "centered_phase",
        "detrended_phase",
        "smoothed_phase_for_zcr",
        "true_range_m",
    ]
    if not all(col in df.columns for col in required_cols):
        print(f"CSV missing columns. Found: {df.columns}")
        return

    time = df["time_s"].to_numpy(dtype=float)
    raw_phase = df["raw_unwrapped_phase"].to_numpy(dtype=float)
    expected_phase = df["expected_bulk_phase"].to_numpy(dtype=float)
    centered_phase = df["centered_phase"].to_numpy(dtype=float)
    detrended_phase = df["detrended_phase"].to_numpy(dtype=float)
    smoothed_phase = df["smoothed_phase_for_zcr"].to_numpy(dtype=float)
    true_range = df["true_range_m"].to_numpy(dtype=float)
    true_los_z = (
        df["true_los_z"].to_numpy(dtype=float)
        if "true_los_z" in df.columns
        else np.ones_like(time)
    )
    estimated_microdoppler_hz = (
        df["estimated_microdoppler_hz"].to_numpy(dtype=float)
        if "estimated_microdoppler_hz" in df.columns
        else np.full_like(time, np.nan)
    )

    residual_phase = detrended_phase
    observed_radial_displacement_m = -residual_phase * LAMBDA / (4.0 * np.pi)
    valid_los_mask = np.abs(true_los_z) >= LOS_Z_EPS
    vertical_displacement_est_m = np.full_like(observed_radial_displacement_m, np.nan)
    vertical_displacement_est_m[valid_los_mask] = (
        observed_radial_displacement_m[valid_los_mask] / true_los_z[valid_los_mask]
    )

    dt = np.mean(np.diff(time))
    fs = 1.0 / dt

    # A. FFT Analysis
    nfft = 1 << (len(detrended_phase) - 1).bit_length()
    nfft *= 8
    window = np.hanning(len(detrended_phase))
    fft_result = np.fft.rfft(detrended_phase * window, n=nfft)
    freqs = np.fft.rfftfreq(nfft, d=dt)
    magnitude = np.abs(fft_result)

    # Peak search in [40, 250] Hz
    mask = (freqs >= 40) & (freqs <= 250)
    if np.any(mask):
        fft_peak_freq = freqs[mask][np.argmax(magnitude[mask])]
    else:
        fft_peak_freq = freqs[np.argmax(magnitude)]

    model_freq_hz = (
        estimated_microdoppler_hz[0]
        if estimated_microdoppler_hz.size > 0
        and np.isfinite(estimated_microdoppler_hz[0])
        else fft_peak_freq
    )
    truth_vertical_model_m = VERTICAL_BOUNCE_AMPLITUDE_M * np.sin(
        2.0 * np.pi * model_freq_hz * (time - time[0])
    )
    truth_radial_model_m = truth_vertical_model_m * true_los_z
    truth_phase_model_rad = -(4.0 * np.pi / LAMBDA) * truth_radial_model_m
    projected_fit = fit_projected_vertical_motion(
        time,
        observed_radial_displacement_m,
        true_los_z,
        min_freq_hz=40.0,
        max_freq_hz=250.0,
    )

    # B. Zero-Crossing Rate (ZCR) on the exported pre-ZCR signal
    crossings = np.where(np.diff(np.sign(smoothed_phase)))[0]
    duration = time[-1] - time[0]
    zcr_freq = len(crossings) / (2.0 * duration)

    # --- Results ---
    print(f"\nAnalysis for {os.path.basename(csv_path)}:")
    print(f"  FFT Peak Freq:      {fft_peak_freq:6.2f} Hz")
    print(f"  Zero-Crossing Freq: {zcr_freq:6.2f} Hz")
    print(f"  Mean smoothed bias: {np.mean(smoothed_phase):6.4f} rad")
    if projected_fit is not None:
        print(
            f"  LOS-fit Freq:       {projected_fit['freq_hz']:6.2f} Hz"
            f"  amp={projected_fit['amplitude_m']:.4f} m"
        )
    print(
        f"  Predicted phase amp: {np.nanmax(np.abs(truth_phase_model_rad)):6.2f} rad"
        f" for {VERTICAL_BOUNCE_AMPLITUDE_M:.2f} m vertical motion"
    )

    # --- Plotting ---
    fig, axes = plt.subplots(6, 1, figsize=(12, 24))

    # 1. Phase decomposition
    axes[0].plot(time, raw_phase, label="Raw unwrapped phase", alpha=0.8)
    axes[0].plot(time, expected_phase, label="Expected bulk phase", alpha=0.8)
    axes[0].plot(time, centered_phase, label="Centered phase", alpha=0.8)
    axes[0].set_title(f"Phase decomposition - {os.path.basename(csv_path)}")
    axes[0].set_ylabel("Phase (rad)")
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_xlim(time[0], time[min(len(time) - 1, max(1, int(0.1 / dt)))])

    # 2. Detrended vs pre-ZCR signal
    axes[1].plot(time, detrended_phase, "k", alpha=0.8, label="Detrended phase")
    axes[1].plot(time, smoothed_phase, "b", alpha=0.8, label="Smoothed phase for ZCR")
    axes[1].plot(
        time,
        truth_phase_model_rad,
        color="tab:green",
        alpha=0.7,
        label="Projected z-bounce model",
    )
    if projected_fit is not None:
        axes[1].plot(
            time,
            -(4.0 * np.pi / LAMBDA) * projected_fit["radial_fit_m"],
            color="tab:orange",
            alpha=0.8,
            label=f"Best LOS-fit phase ({projected_fit['freq_hz']:.1f} Hz)",
        )
    axes[1].axhline(0.0, color="r", linestyle="--", linewidth=1.0, label="Zero line")
    axes[1].set_title("Residual phase vs projected vertical-bounce model")
    axes[1].set_ylabel("Phase (rad)")
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_xlim(time[0], time[min(len(time) - 1, max(1, int(0.1 / dt)))])

    # 3. FFT plot
    axes[2].plot(freqs, magnitude)
    axes[2].axvline(
        fft_peak_freq,
        color="r",
        linestyle="--",
        label=f"FFT Peak: {fft_peak_freq:.2f} Hz",
    )
    if projected_fit is not None:
        axes[2].axvline(
            projected_fit["freq_hz"],
            color="tab:orange",
            linestyle=":",
            label=f"LOS-fit: {projected_fit['freq_hz']:.2f} Hz",
        )
    axes[2].set_title("Frequency domain analysis of detrended phase")
    axes[2].set_xlabel("Frequency (Hz)")
    axes[2].set_ylabel("Magnitude")
    axes[2].set_xlim(0, 300)
    axes[2].legend()
    axes[2].grid(True)

    # 4. Zero-crossing diagnostic
    axes[3].plot(time, smoothed_phase, "b", label="Smoothed phase for ZCR")
    axes[3].axhline(0.0, color="r", linestyle="--", linewidth=1.0, label="Zero line")
    if len(crossings) > 0:
        axes[3].scatter(
            time[crossings],
            smoothed_phase[crossings],
            color="k",
            s=12,
            label="Crossings",
        )
    axes[3].set_title("Zero-crossing diagnostic")
    axes[3].set_ylabel("Phase (rad)")
    axes[3].grid(True)
    axes[3].legend()
    axes[3].set_xlim(time[0], time[min(len(time) - 1, max(1, int(0.1 / dt)))])

    # 5. Projection-aware displacement view
    axes[4].plot(
        time,
        observed_radial_displacement_m,
        label="Observed radial displacement",
        color="tab:blue",
    )
    axes[4].plot(
        time,
        truth_radial_model_m,
        label="Expected radial displacement from z-bounce",
        color="tab:green",
        alpha=0.8,
    )
    axes[4].plot(
        time,
        vertical_displacement_est_m,
        label="Deprojected vertical displacement estimate",
        color="tab:orange",
        alpha=0.8,
    )
    if projected_fit is not None:
        axes[4].plot(
            time,
            projected_fit["vertical_fit_m"],
            label=f"Best-fit vertical sine ({projected_fit['amplitude_m']:.3f} m)",
            color="tab:red",
            alpha=0.8,
        )
    axes[4].set_title("Projection-aware displacement")
    axes[4].set_ylabel("Displacement (m)")
    axes[4].grid(True)
    axes[4].legend()
    axes[4].set_xlim(time[0], time[min(len(time) - 1, max(1, int(0.1 / dt)))])

    # 6. Geometry
    axes[5].plot(time, true_range, label="True range (m)")
    axes[5].plot(time, true_los_z, label="LOS z-component")
    axes[5].set_title("Truth geometry and projection scale")
    axes[5].set_xlabel("Time (s)")
    axes[5].grid(True)
    axes[5].legend()

    plt.tight_layout()
    output_png = csv_path.replace(".csv", "_analysis.png")
    plt.savefig(output_png)
    print(f"Saved analysis plot to {output_png}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_microdoppler_fft.py <microdoppler_data.csv>")
    else:
        for csv_path in sys.argv[1:]:
            plot_microdoppler_analysis(csv_path)
