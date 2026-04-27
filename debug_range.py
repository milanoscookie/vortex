import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_metadata(path):
    metadata = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            metadata[row["key"]] = row["value"]
    return metadata


def main():
    output_dir = Path("output")
    metadata = load_metadata(output_dir / "metadata.csv")
    fs = float(metadata["sample_rate_hz"])
    n_rx = int(metadata["probe_num_x"]) * int(metadata["probe_num_y"])
    block_size = int(metadata["block_size"])
    chirp_count = int(metadata["chirp_count"])
    bw = float(metadata["bandwidth_hz"])
    c = float(metadata["speed_of_light_mps"])

    tx = np.fromfile(output_dir / "tx_chirp.bin", dtype=np.complex64)
    rx = np.fromfile(output_dir / "rx_burst.bin", dtype=np.complex64).reshape(
        chirp_count, block_size, n_rx
    )

    # Dechirp
    beat = rx[0, :, 0] * np.conj(tx)

    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(np.real(beat))
    plt.title("Real part of dechirped signal (Chirp 0, Element 0)")

    plt.subplot(2, 1, 2)
    spec = np.fft.fftshift(np.fft.fft(beat, n=4096))
    freqs = np.fft.fftshift(np.fft.fftfreq(4096, d=1 / fs))
    plt.plot(freqs / 1e6, 20 * np.log10(np.abs(spec)))
    plt.title("Spectrum of beat signal")
    plt.xlabel("Frequency (MHz)")
    plt.ylabel("dB")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("debug_beat.png")

    peak_freq = freqs[np.argmax(np.abs(spec))]
    print(f"Peak frequency: {peak_freq / 1e6:.6f} MHz")
    # Expected peak frequency = S * tau = (BW / Tc) * (2R / c)
    tc = block_size / fs
    s = bw / tc
    # For R=10m:
    expected_f = s * (2 * 10 / c)
    print(f"Expected frequency for 10m: {expected_f / 1e6:.6f} MHz")
    # Range from peak freq
    est_r = abs(peak_freq) * c / (2 * s)
    print(f"Estimated range from peak: {est_r:.3f} m")


if __name__ == "__main__":
    main()
