from pathlib import Path

import numpy as np


def main():
    tx = np.fromfile("output/tx_chirp.bin", dtype=np.complex64)
    rx = np.fromfile("output/rx_burst.bin", dtype=np.complex64).reshape(-1, 512, 64)

    print("TX samples 0-5:")
    print(tx[:5])
    print("RX (Chirp 0, Element 0) samples 0-5:")
    print(rx[0, :5, 0])

    # Dechirp
    beat = rx[0, :, 0] * np.conj(tx)
    print("Beat samples 0-5:")
    print(beat[:5])

    # Calculate phase difference
    phases = np.angle(beat)
    print("Beat phases 0-5 (rad):")
    print(phases[:5])
    print("Phase diffs (rad):")
    print(np.diff(phases[:5]))


if __name__ == "__main__":
    main()
