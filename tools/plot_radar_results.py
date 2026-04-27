#!/usr/bin/env python3

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "output"


def load_metadata(path: Path) -> dict[str, float | str]:
    metadata: dict[str, float | str] = {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            key = row["key"]
            value = row["value"]
            try:
                metadata[key] = float(value)
            except ValueError:
                metadata[key] = value
    return metadata


def get_float(metadata: dict[str, float | str], key: str) -> float:
    value = metadata[key]
    if isinstance(value, str):
        return float(value)
    return float(value)


def get_str(metadata: dict[str, float | str], key: str) -> str:
    value = metadata[key]
    return value if isinstance(value, str) else str(value)


def load_truth(path: Path) -> dict[str, np.ndarray]:
    rows: dict[str, list[float]] = {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise RuntimeError("truth csv is missing a header")
        rows = {field: [] for field in reader.fieldnames}
        for row in reader:
            for field in reader.fieldnames:
                rows[field].append(float(row[field]))
    return {
        field: np.asarray(values, dtype=np.float64) for field, values in rows.items()
    }


def load_complex_binary(path: Path, count: int) -> np.ndarray:
    values = np.fromfile(path, dtype=np.complex64)
    if values.size != count:
        raise RuntimeError(
            f"unexpected complex count in {path}: {values.size} != {count}"
        )
    return values


def load_estimates(path: Path) -> dict[str, np.ndarray]:
    rows: dict[str, list[float]] = {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise RuntimeError("estimates csv is missing a header")
        rows = {field: [] for field in reader.fieldnames}
        for row in reader:
            for field in reader.fieldnames:
                rows[field].append(float(row[field]))
    return {
        field: np.asarray(values, dtype=np.float64) for field, values in rows.items()
    }


def dechirp(tx: np.ndarray, rx: np.ndarray) -> np.ndarray:
    return rx * np.conj(tx)[None, :, None]


def folded_range_power_map(
    beat: np.ndarray,
    sample_rate_hz: float,
    bandwidth_hz: float,
    speed_of_light_mps: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    block_size = beat.shape[1]
    chirp_count = beat.shape[0]
    chirp_duration_s = block_size / sample_rate_hz
    chirp_slope_hz_per_s = bandwidth_hz / chirp_duration_s

    window = np.hanning(block_size).astype(np.float32)
    spectrum = np.fft.fftshift(np.fft.fft(beat * window[None, :, None], axis=1), axes=1)
    power = np.mean(np.abs(spectrum) ** 2, axis=2)

    half_bins = block_size // 2
    zero_bin = half_bins
    folded_power = np.empty((chirp_count, half_bins + 1), dtype=np.float64)
    folded_power[:, 0] = power[:, zero_bin]
    for bin_index in range(1, half_bins):
        folded_power[:, bin_index] = np.maximum(
            power[:, zero_bin + bin_index], power[:, zero_bin - bin_index]
        )
    folded_power[:, half_bins] = power[:, 0]

    beat_freq_axis_hz = (
        np.arange(half_bins + 1, dtype=np.float64) * sample_rate_hz / block_size
    )
    range_axis_m = speed_of_light_mps * beat_freq_axis_hz / (2.0 * chirp_slope_hz_per_s)
    return folded_power, range_axis_m, spectrum


def stft(
    signal: np.ndarray, sample_period_s: float, window_size: int, hop_size: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if signal.size < window_size:
        padded = np.zeros(window_size, dtype=np.complex64)
        padded[: signal.size] = signal
        signal = padded

    window = np.hanning(window_size).astype(np.float32)
    rows: list[np.ndarray] = []
    for start in range(0, signal.size - window_size + 1, hop_size):
        rows.append(
            np.fft.fftshift(np.fft.fft(signal[start : start + window_size] * window))
        )

    spectrogram = np.asarray(rows, dtype=np.complex64)
    time_axis_s = (
        np.arange(spectrogram.shape[0], dtype=np.float32) * hop_size * sample_period_s
    )
    freq_axis_hz = np.fft.fftshift(np.fft.fftfreq(window_size, d=sample_period_s))
    return spectrogram, time_axis_s, freq_axis_hz


if __name__ == "__main__":
    metadata = load_metadata(OUTPUT_DIR / "metadata.csv")
    sample_rate_hz = get_float(metadata, "sample_rate_hz")
    carrier_hz = get_float(metadata, "carrier_hz")
    bandwidth_hz = get_float(metadata, "bandwidth_hz")
    speed_of_light_mps = get_float(metadata, "speed_of_light_mps")
    block_size = int(get_float(metadata, "block_size"))
    chirp_count = int(get_float(metadata, "chirp_count"))
    chirp_duration_s = get_float(metadata, "chirp_duration_s")
    probe_num_x = int(get_float(metadata, "probe_num_x"))
    probe_num_y = int(get_float(metadata, "probe_num_y"))

    tx = load_complex_binary(
        OUTPUT_DIR / get_str(metadata, "tx_chirp_path"), block_size
    )
    rx = load_complex_binary(
        OUTPUT_DIR / get_str(metadata, "rx_burst_path"),
        chirp_count * block_size * probe_num_x * probe_num_y,
    ).reshape(chirp_count, block_size, probe_num_x * probe_num_y)
    truth = load_truth(OUTPUT_DIR / get_str(metadata, "truth_path"))

    beat = dechirp(tx, rx)
    range_power, range_axis_m, spectrum = folded_range_power_map(
        beat, sample_rate_hz, bandwidth_hz, speed_of_light_mps
    )

    peak_bins = np.argmax(range_power, axis=1)
    estimated_range_m = range_axis_m[peak_bins]

    dominant_range_bin = int(np.argmax(np.mean(range_power[:, 1:], axis=0)) + 1)
    signed_bin = block_size // 2 + dominant_range_bin
    target_series = np.mean(spectrum[:, signed_bin, :], axis=1)

    slow_spec, slow_time_s, slow_freq_hz = stft(
        target_series,
        sample_period_s=chirp_duration_s,
        window_size=min(32, chirp_count),
        hop_size=max(1, min(4, chirp_count // 8)),
    )
    wavelength_m = speed_of_light_mps / carrier_hz
    slow_velocity_mps = slow_freq_hz * wavelength_m / 2.0

    time_s = truth["time_s"]
    true_range_m = truth["range_m"]
    true_radial_velocity_mps = truth["radial_velocity_mps"]
    true_doppler_hz = truth["doppler_hz"]

    mean_est_slant_range_m = float(np.mean(estimated_range_m))
    mean_true_slant_range_m = float(np.mean(true_range_m))
    rmse_range_m = float(np.sqrt(np.mean((estimated_range_m - true_range_m) ** 2)))

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    im0 = axes[0, 0].imshow(
        10.0 * np.log10(np.maximum(range_power.T, 1e-12)),
        aspect="auto",
        origin="lower",
        extent=[time_s[0], time_s[-1], range_axis_m[0], range_axis_m[-1]],
        cmap="viridis",
    )
    axes[0, 0].plot(time_s, estimated_range_m, "w--", linewidth=1.0, label="Estimated")
    axes[0, 0].plot(time_s, true_range_m, "c-", linewidth=1.0, label="Truth")
    axes[0, 0].set_title("Range FFT Map")
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Slant Range (m)")
    axes[0, 0].legend(loc="upper right")
    fig.colorbar(im0, ax=axes[0, 0], label="dB")

    axes[0, 1].plot(time_s, true_range_m, label="True range")
    axes[0, 1].plot(time_s, estimated_range_m, label="Estimated range", linestyle="--")
    axes[0, 1].set_title("Slant Range Tracking")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Range (m)")
    axes[0, 1].legend()

    im1 = axes[1, 0].imshow(
        20.0 * np.log10(np.maximum(np.abs(slow_spec.T), 1e-12)),
        aspect="auto",
        origin="lower",
        extent=[
            slow_time_s[0],
            slow_time_s[-1],
            slow_velocity_mps[0],
            slow_velocity_mps[-1],
        ],
        cmap="magma",
    )
    axes[1, 0].set_title("Micro-Doppler Spectrogram")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Radial Velocity (m/s)")
    fig.colorbar(im1, ax=axes[1, 0], label="dB")

    axes[1, 1].plot(
        time_s, true_radial_velocity_mps, label="True radial velocity (m/s)"
    )
    axes[1, 1].plot(time_s, true_doppler_hz, "--", label="True Doppler (Hz)")
    axes[1, 1].set_title("Truth Kinematics")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Velocity / Doppler")
    axes[1, 1].legend()

    fig.tight_layout()
    output_path = OUTPUT_DIR / "radar_overview.png"
    fig.savefig(output_path, dpi=160)
    plt.close(fig)

    # Plot XYZ velocity over time if available
    estimates_path = OUTPUT_DIR / "fmcw_car_estimates.csv"
    if estimates_path.exists():
        estimates = load_estimates(estimates_path)
        est_time_s = estimates["time_s"]
        est_x = estimates["x"]
        est_y = estimates["y"]
        est_z = estimates["z"]
        est_v_rad = estimates["v_rad"]

        # Compute XYZ velocity from position derivatives
        est_vx = np.gradient(est_x, est_time_s)
        est_vy = np.gradient(est_y, est_time_s)
        est_vz = np.gradient(est_z, est_time_s)

        fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8))

        axes2[0].plot(est_time_s, est_vx, "b-", linewidth=1.0, label="vx")
        axes2[0].set_ylabel("vx (m/s)")
        axes2[0].set_title("XYZ Velocity over Time")
        axes2[0].legend()
        axes2[0].grid(True, alpha=0.3)

        axes2[1].plot(est_time_s, est_vy, "g-", linewidth=1.0, label="vy")
        axes2[1].set_ylabel("vy (m/s)")
        axes2[1].legend()
        axes2[1].grid(True, alpha=0.3)

        axes2[2].plot(est_time_s, est_vz, "r-", linewidth=1.0, label="vz")
        axes2[2].set_xlabel("Time (s)")
        axes2[2].set_ylabel("vz (m/s)")
        axes2[2].legend()
        axes2[2].grid(True, alpha=0.3)

        fig2.tight_layout()
        output_path2 = OUTPUT_DIR / "velocity_xyz_over_time.png"
        fig2.savefig(output_path2, dpi=160)
        plt.close(fig2)

        print(f"Saved XYZ velocity plot to {output_path2}")

        # Also plot radial velocity
        fig3, ax3 = plt.subplots(1, 1, figsize=(12, 5))
        ax3.plot(est_time_s, est_v_rad, "b-", linewidth=1.0, label="Estimated v_rad")
        ax3.axhline(0, color="k", linewidth=0.5, linestyle="--")
        ax3.set_title("Estimated Radial Velocity over Time")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Radial Velocity (m/s)")
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        fig3.tight_layout()
        output_path3 = OUTPUT_DIR / "velocity_over_time.png"
        fig3.savefig(output_path3, dpi=160)
        plt.close(fig3)

        print(f"Saved radial velocity plot to {output_path3}")

    print(f"Saved plot to {output_path}")
    print(f"Estimated mean slant range: {mean_est_slant_range_m:.3f} m")
    print(f"True mean slant range: {mean_true_slant_range_m:.3f} m")
    print(f"Range RMSE: {rmse_range_m:.3f} m")
