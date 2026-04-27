#!/usr/bin/env python3

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "output"
METADATA_PATH = OUTPUT_DIR / "metadata.csv"


@dataclass(frozen=True)
class RadarConfig:
    sample_rate_hz: float
    carrier_hz: float
    bandwidth_hz: float
    chirp_duration_s: float
    speed_of_light_mps: float
    block_size: int
    chirp_count: int
    probe_num_x: int
    probe_num_y: int
    probe_dx_m: float
    probe_dy_m: float

    @property
    def chirp_slope_hz_per_s(self) -> float:
        return self.bandwidth_hz / self.chirp_duration_s

    @property
    def wavelength_m(self) -> float:
        return self.speed_of_light_mps / self.carrier_hz

    @property
    def num_rx(self) -> int:
        return self.probe_num_x * self.probe_num_y


@dataclass(frozen=True)
class DetectionConfig:
    min_range_m: float = 20.0
    max_range_m: float = 500.0
    coherent_processing_interval_chirps: int = 64
    hop_chirps: int = 64
    zero_doppler_guard_bins: int = 2
    nfft_range_min: int = 4096
    static_clutter_suppression_enable: bool = False
    aoa_enable: bool = True
    azimuth_min_deg: float = -90.0
    azimuth_max_deg: float = 90.0
    azimuth_count: int = 181
    elevation_min_deg: float = 0.0
    elevation_max_deg: float = 90.0
    elevation_count: int = 91


def load_metadata(path: Path) -> dict[str, float | str]:
    metadata: dict[str, float | str] = {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            key, value = row["key"], row["value"]
            try:
                metadata[key] = float(value)
            except ValueError:
                metadata[key] = value
    return metadata


def load_radar_config(metadata: dict[str, float | str]) -> RadarConfig:
    return RadarConfig(
        sample_rate_hz=float(metadata["sample_rate_hz"]),
        carrier_hz=float(metadata["carrier_hz"]),
        bandwidth_hz=float(metadata["bandwidth_hz"]),
        chirp_duration_s=float(metadata["chirp_duration_s"]),
        speed_of_light_mps=float(metadata["speed_of_light_mps"]),
        block_size=int(float(metadata["block_size"])),
        chirp_count=int(float(metadata["chirp_count"])),
        probe_num_x=int(float(metadata["probe_num_x"])),
        probe_num_y=int(float(metadata["probe_num_y"])),
        probe_dx_m=float(metadata["probe_dx_m"]),
        probe_dy_m=float(metadata["probe_dy_m"]),
    )


def element_positions(cfg: RadarConfig) -> np.ndarray:
    positions = np.zeros((cfg.num_rx, 3), dtype=np.float64)
    for ix in range(cfg.probe_num_x):
        for iy in range(cfg.probe_num_y):
            flat = ix * cfg.probe_num_y + iy
            positions[flat, 0] = (ix - 0.5 * (cfg.probe_num_x - 1)) * cfg.probe_dx_m
            positions[flat, 1] = (iy - 0.5 * (cfg.probe_num_y - 1)) * cfg.probe_dy_m
            positions[flat, 2] = 0.0
    return positions


def make_steering_grid(
    positions_m: np.ndarray, wavelength_m: float, det: DetectionConfig
) -> tuple[np.ndarray, np.ndarray]:
    az_rad = np.deg2rad(
        np.linspace(det.azimuth_min_deg, det.azimuth_max_deg, det.azimuth_count)
    )
    el_rad = np.deg2rad(
        np.linspace(det.elevation_min_deg, det.elevation_max_deg, det.elevation_count)
    )
    az_mesh, el_mesh = np.meshgrid(az_rad, el_rad, indexing="ij")
    ux, uy, uz = (
        np.cos(el_mesh) * np.cos(az_mesh),
        np.cos(el_mesh) * np.sin(az_mesh),
        np.sin(el_mesh),
    )
    directions = np.stack((ux, uy, uz), axis=-1).reshape(-1, 3).astype(np.float32)
    phase = (2.0 * np.pi / wavelength_m) * (directions @ positions_m.T)
    return np.exp(-1j * phase).astype(np.complex64), directions


def process_batch(
    start_chirp: int,
    n_chirps: int,
    rx_mmap: np.ndarray,
    tx_conj: np.ndarray,
    cfg: RadarConfig,
    det: DetectionConfig,
    range_indices: np.ndarray,
    range_axis_sliced: np.ndarray,
    win_range: np.ndarray,
    win_doppler: np.ndarray,
    steering_conj: np.ndarray,
    directions: np.ndarray,
):

    rx_batch = rx_mmap[start_chirp : start_chirp + n_chirps].astype(np.complex64)
    beat = rx_batch * tx_conj[None, :, None]

    nfft = det.nfft_range_min
    spec_full = np.fft.fft(beat * win_range[None, :, None], n=nfft, axis=1)
    spec = spec_full[:, range_indices, :]

    if det.static_clutter_suppression_enable:
        spec -= np.mean(spec, axis=0, keepdims=True)

    rd_cube = np.fft.fftshift(
        np.fft.fft(spec * win_doppler[:, None, None], axis=0), axes=0
    )
    doppler_axis_hz = np.fft.fftshift(np.fft.fftfreq(n_chirps, d=cfg.chirp_duration_s))
    rd_power = np.mean(np.abs(rd_cube) ** 2, axis=2)

    d_mask = np.ones(n_chirps, dtype=bool)
    if det.zero_doppler_guard_bins > 0:
        z = n_chirps // 2
        d_mask[
            max(0, z - det.zero_doppler_guard_bins) : min(
                n_chirps, z + det.zero_doppler_guard_bins + 1
            )
        ] = False

    masked_power = rd_power * d_mask[:, None]
    dbin, rbin = np.unravel_index(np.argmax(masked_power), masked_power.shape)

    fd = doppler_axis_hz[dbin]
    range_correction = (fd / cfg.chirp_slope_hz_per_s) * (cfg.speed_of_light_mps / 2.0)
    corrected_range = range_axis_sliced[rbin] - range_correction

    snapshot = rd_cube[dbin, rbin, :]
    peak_val = rd_cube[dbin, rbin, 0]
    resp = np.abs(steering_conj @ snapshot) ** 2
    best_aoa = np.argmax(resp)
    dir_vec = directions[best_aoa]

    return {
        "time": (start_chirp + n_chirps / 2) * cfg.chirp_duration_s,
        "range_m": float(corrected_range),
        "doppler_hz": float(fd),
        "dir": dir_vec,
        "doppler_slice": rd_power[:, rbin],
        "vel_axis": 0.5 * doppler_axis_hz * cfg.wavelength_m,
        "phase_rad": np.angle(peak_val),
    }


def main():
    metadata = load_metadata(METADATA_PATH)
    cfg = load_radar_config(metadata)
    det = DetectionConfig()

    tx_conj = np.conj(np.fromfile(OUTPUT_DIR / "tx_chirp.bin", dtype=np.complex64))
    rx_mmap = np.memmap(
        OUTPUT_DIR / "rx_burst.bin",
        dtype=np.complex64,
        mode="r",
        shape=(cfg.chirp_count, cfg.block_size, cfg.num_rx),
    )

    nfft = det.nfft_range_min
    freqs = np.fft.fftfreq(nfft, d=1.0 / cfg.sample_rate_hz)
    range_axis_full = (
        cfg.speed_of_light_mps * np.abs(freqs) / (2.0 * cfg.chirp_slope_hz_per_s)
    )

    range_indices = np.where(
        (range_axis_full >= det.min_range_m) & (range_axis_full <= det.max_range_m)
    )[0]
    range_axis_sliced = range_axis_full[range_indices]

    win_range = np.hanning(cfg.block_size).astype(np.complex64)
    win_doppler = np.hanning(det.coherent_processing_interval_chirps).astype(
        np.complex64
    )

    pos_m = element_positions(cfg)
    steering_conj, directions = make_steering_grid(pos_m, cfg.wavelength_m, det)

    starts = np.arange(
        0, cfg.chirp_count - det.coherent_processing_interval_chirps + 1, det.hop_chirps
    )

    print(f"Processing {len(starts)} batches sequentially...")
    results = []

    for s in starts:
        res = process_batch(
            int(s),
            det.coherent_processing_interval_chirps,
            rx_mmap,
            tx_conj,
            cfg,
            det,
            range_indices,
            range_axis_sliced,
            win_range,
            win_doppler,
            steering_conj,
            directions,
        )
        results.append(res)

    times = np.array([r["time"] for r in results])
    raw_xyz = np.array([r["range_m"] * r["dir"] for r in results])
    ranges = np.array([r["range_m"] for r in results])
    v_rad = 0.5 * np.array([r["doppler_hz"] for r in results]) * cfg.wavelength_m
    phases = np.unwrap(np.array([r["phase_rad"] for r in results]))
    m_doppler = np.stack([r["doppler_slice"] for r in results])
    v_axis = results[0]["vel_axis"]

    # SMOOTHING FIX: Filter position data before calculating derivatives to eliminate AoA grid noise
    # We use a moving average window (length 11) to stabilize the track
    window_len = min(11, len(raw_xyz))

    def smooth(x):
        return np.convolve(x, np.ones(window_len) / window_len, mode="same")

    xyz = np.column_stack(
        [smooth(raw_xyz[:, 0]), smooth(raw_xyz[:, 1]), smooth(raw_xyz[:, 2])]
    )

    # Calculate Velocity Components from smoothed position derivatives
    vx = np.gradient(xyz[:, 0], times)
    vy = np.gradient(xyz[:, 1], times)
    vz = np.gradient(xyz[:, 2], times)

    # Save Results
    with open(OUTPUT_DIR / "fmcw_car_estimates.csv", "w") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "x", "y", "z", "range", "v_rad", "vx", "vy", "vz"])
        for i in range(len(times)):
            w.writerow(
                [
                    times[i],
                    xyz[i, 0],
                    xyz[i, 1],
                    xyz[i, 2],
                    ranges[i],
                    v_rad[i],
                    vx[i],
                    vy[i],
                    vz[i],
                ]
            )

    print(f"Mean XYZ: {np.mean(xyz, axis=0)}")
    print(f"Mean V_xyz: [{np.mean(vx):.2f}, {np.mean(vy):.2f}, {np.mean(vz):.2f}] m/s")

    # Plot
    fig, axs = plt.subplots(11, 1, figsize=(10, 38))

    position_components = (
        ("X Position over Time (Smoothed)", "X (m)", xyz[:, 0], "tab:blue"),
        ("Y Position over Time (Smoothed)", "Y (m)", xyz[:, 1], "tab:orange"),
        ("Z Position over Time (Smoothed)", "Z (m)", xyz[:, 2], "tab:green"),
    )
    for ax, (title, ylabel, values, color) in zip(axs[0:3], position_components):
        ax.plot(times, values, color=color, linewidth=1.5)
        ax.set_title(title)
        ax.set_ylabel(ylabel)

    velocity_components = (
        ("X Velocity (from Smoothed Derivatives)", "vx (m/s)", vx, "tab:blue"),
        ("Y Velocity (from Smoothed Derivatives)", "vy (m/s)", vy, "tab:orange"),
        ("Z Velocity (from Smoothed Derivatives)", "vz (m/s)", vz, "tab:green"),
    )
    for ax, (title, ylabel, values, color) in zip(axs[3:6], velocity_components):
        ax.plot(times, values, color=color, linewidth=1.5)
        ax.set_title(title)
        ax.set_ylabel(ylabel)

    axs[6].plot(times, ranges)
    axs[6].set_title("Corrected Range")
    axs[6].set_ylabel("Range (m)")
    axs[7].plot(times, v_rad)
    axs[7].set_title("Radial Velocity (Doppler)")
    axs[7].set_ylabel("Velocity (m/s)")
    axs[8].plot(times, phases, color="green")
    axs[8].set_title("Target Phase over Time")
    axs[8].set_ylabel("Phase (rad)")

    db_m_doppler = 10 * np.log10(m_doppler.T + 1e-12)
    im = axs[9].imshow(
        db_m_doppler,
        aspect="auto",
        origin="lower",
        extent=[times[0], times[-1], v_axis[0], v_axis[-1]],
        cmap="magma",
    )
    axs[9].set_title("Micro-Doppler Spectrogram")
    axs[9].set_ylabel("Velocity (m/s)")

    axs[10].plot(xyz[:, 0], xyz[:, 1], ".-")
    axs[10].set_title("Trajectory (XY Plane)")
    axs[10].set_xlabel("X (m)")
    axs[10].set_ylabel("Y (m)")
    axs[10].axis("equal")

    for ax in axs[:-1]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "fmcw_car_tracking.png")


if __name__ == "__main__":
    main()
