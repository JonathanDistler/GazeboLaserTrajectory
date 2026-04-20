#!/usr/bin/env python3

import subprocess
from collections import deque
import numpy as np
import signal
import sys
import matplotlib.pyplot as plt

print("🚀 Stable CoM + Smoothed Laser + Gaussian Heatmap")

# -----------------------------
# STREAM
# -----------------------------
proc = subprocess.Popen(
    ["gz", "topic", "-e", "-t", "/world/default/pose/info"],
    stdout=subprocess.PIPE,
    text=True,
    bufsize=1
)

# -----------------------------
# STORAGE
# -----------------------------
raw_com = deque(maxlen=2000)
filtered_com = deque(maxlen=2000)

laser_path = []

# -----------------------------
# FILTER STATE
# -----------------------------
alpha = 0.15  # smoothing factor (lower = smoother)

prev_filtered = None


# -----------------------------
# SMOOTHING FUNCTION
# -----------------------------
def smooth(x, y, z):
    global prev_filtered

    current = np.array([x, y, z])

    if prev_filtered is None:
        prev_filtered = current
        return current

    filtered = alpha * current + (1 - alpha) * prev_filtered
    prev_filtered = filtered

    return filtered


# -----------------------------
# VELOCITY (smoothed)
# -----------------------------
def get_velocity(history, window=8):
    if len(history) < window + 1:
        return np.array([0, 0, 1])

    a = np.array(history[-window])
    b = np.array(history[-1])

    v = b - a
    norm = np.linalg.norm(v)

    if norm < 1e-6:
        return np.array([0, 0, 1])

    return v / norm


# -----------------------------
# GAUSSIAN HEAT FIELD
# -----------------------------
def gaussian_heat(center, grid=40, sigma=0.25):
    xs, ys, cs = [], [], []

    cx, cy, cz = center

    for i in range(-grid, grid):
        for j in range(-grid, grid):

            x = cx + i * 0.02
            y = cy + j * 0.02

            r2 = (x - cx)**2 + (y - cy)**2
            intensity = np.exp(-r2 / (2 * sigma**2))

            xs.append(x)
            ys.append(y)
            cs.append(intensity)

    return np.array(xs), np.array(ys), np.array(cs)


# -----------------------------
# EXIT HANDLER
# -----------------------------
def exit_handler(sig, frame):
    print("\n📊 Plotting results...")

    com = np.array(filtered_com)

    # -------------------------
    # CoM trajectory (SMOOTHED)
    # -------------------------
    if len(com) > 0:
        plt.figure()
        plt.plot(com[:, 0], com[:, 1], linewidth=1.5)
        plt.title("Smoothed CoM Trajectory")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis("equal")

    # -------------------------
    # Laser path
    # -------------------------
    if len(laser_path) > 0:
        lp = np.array(laser_path)

        plt.figure()
        plt.plot(lp[:, 0], lp[:, 1], 'r')
        plt.title("Stable Laser Projection Path")
        plt.axis("equal")

    # -------------------------
    # FINAL FRAME HEATMAP (DECAY)
    # -------------------------
    if len(filtered_com) > 0:
        last = filtered_com[-1]

        x, y, c = gaussian_heat(last)

        plt.figure()
        sc = plt.scatter(x, y, c=c, cmap="inferno", s=10)
        plt.colorbar(sc)
        plt.title("Final Frame Gaussian Heatmap (Decay)")
        plt.axis("equal")

    plt.show()
    sys.exit(0)


signal.signal(signal.SIGINT, exit_handler)


# -----------------------------
# PARSER STATE
# -----------------------------
tracking = False
reading = False

x = y = z = None


# -----------------------------
# MAIN LOOP
# -----------------------------
while True:
    line = proc.stdout.readline()

    if not line:
        continue

    line = line.strip()

    # -------------------------
    # detect model
    # -------------------------
    if 'name:' in line:
        tracking = ('x500_0' in line)

    if not tracking:
        continue

    # -------------------------
    # position block
    # -------------------------
    if 'position' in line:
        reading = True
        x = y = z = None
        continue

    if reading:

        if 'x:' in line:
            try:
                x = float(line.split(':')[1])
            except:
                x = None

        elif 'y:' in line:
            try:
                y = float(line.split(':')[1])
            except:
                y = None

        elif 'z:' in line:
            try:
                z = float(line.split(':')[1])
            except:
                z = None

            reading = False

            if None in (x, y, z):
                continue

            # -------------------------
            # RAW + FILTERED CoM
            # -------------------------
            raw_com.append((x, y, z))

            filtered = smooth(x, y, z)
            filtered_com.append(filtered)

            # -------------------------
            # LASER (velocity-driven, stable)
            # -------------------------
            vel = get_velocity(filtered_com)

            laser_origin = filtered
            laser_end = filtered + vel * 1.5

            laser_path.append(laser_end)

            print(f"📡 CoM (filtered): {filtered[0]:.4f}, {filtered[1]:.4f}, {filtered[2]:.4f}")
