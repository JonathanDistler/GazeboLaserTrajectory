#!/usr/bin/env python3

import subprocess
from collections import deque
import numpy as np
import signal
import sys
import matplotlib.pyplot as plt
import math

print("Beam Physics Model (Eq. 20)")

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
filtered_com = deque(maxlen=2000)

alpha_smooth = 0.15
prev_filtered = None

# -----------------------------
# SMOOTHING
# -----------------------------
def smooth(x, y, z):
    global prev_filtered
    cur = np.array([x, y, z])

    if prev_filtered is None:
        prev_filtered = cur
        return cur

    prev_filtered = alpha_smooth * cur + (1 - alpha_smooth) * prev_filtered
    return prev_filtered

# -----------------------------
# GEOMETRY HELPERS
# -----------------------------
def spot_waist(q, D, lam, z):
    return math.sqrt((D / (math.pi * q**2))**2 +
                     ((q**2 * lam * z) / D)**2)

def fried_parm(k, z, Cn):
    return (0.423 * k**2 * Cn**2 * z)**(-3/5)

def effect_spot_waist(w0, r0, lam, z):
    return math.sqrt(w0**2 + ((lam * z) / r0)**2)

# -----------------------------
# PARAMETERS
# -----------------------------
wavelength = 1064e-9
q_factor = 1.5
aper_diam = 0.1
refrac_index = 1e-14
wavenumber = 5.9e6

P0 = 5.0
alpha_atm = 0.00015

# -----------------------------
# EQ (20): INTENSITY MODEL
# -----------------------------
def intensity_eq20(P0, alpha, z, w, x, y, theta):

    # atmospheric attenuation
    I0 = P0 * np.exp(-alpha * z)

    cos_t = np.cos(theta)
    cos_t = np.clip(cos_t, 1e-3, 1.0)

    # Eq. (20) elliptical Gaussian
    exponent = -2 * (x**2 + (y**2 / (cos_t**2))) / (w**2)

    I = I0 * cos_t * np.exp(exponent)

    return I

# -----------------------------
# HEATMAP
# -----------------------------
def beam_heatmap(P0, alpha, z, w, theta, grid=500, span=0.5):

    x = np.linspace(-span, span, grid)
    y = np.linspace(-span, span, grid)

    X, Y = np.meshgrid(x, y)

    I = intensity_eq20(P0, alpha, z, w, X, Y, theta)

    return X, Y, I

# -----------------------------
# EXIT HANDLER
# -----------------------------
def exit_handler(sig, frame):
    print("Plotting beam (Eq. 20)...")

    if len(filtered_com) == 0:
        sys.exit(0)

    last = filtered_com[-1]

    # -------------------------
    # GEOMETRY
    # -------------------------
    dist = np.linalg.norm(last)
    theta = math.atan2(last[1], last[2])

    # -------------------------
    # OPTICS
    # -------------------------
    w0 = spot_waist(q_factor, aper_diam, wavelength, dist)
    r0 = fried_parm(wavenumber, dist, refrac_index)
    weff = effect_spot_waist(w0, r0, wavelength, dist)

    # -------------------------
    # FIELD
    # -------------------------
    X, Y, I = beam_heatmap(P0, alpha_atm, dist, weff, theta)

    I = np.nan_to_num(I, nan=0.0, posinf=0.0, neginf=0.0)

    # safe floor (no collapse to zero)
    I = np.clip(I, 1e-20, None)

    I_log = np.log10(I)

    # -------------------------
    # DEBUG
    # -------------------------
    print("\n--- DEBUG ---")
    print(f"Distance: {dist:.2f} m")
    print(f"Theta (deg): {math.degrees(theta):.2f}")
    print(f"w_eff: {weff:.6f}")
    print(f"I max: {np.max(I):.6e}")
    print(f"I min: {np.min(I):.6e}")

    # -------------------------
    # PLOT
    # -------------------------
    plt.figure(figsize=(6, 5))

    plt.contourf(X, Y, I_log, levels=200, cmap="turbo")

    levels = np.linspace(np.min(I_log), np.max(I_log), 30)
    plt.contour(X, Y, I_log, levels=levels, colors='white', linewidths=0.4)

    # Drone face (1m x 1m)
    d = 0.5
    plt.plot(
        [-d, d, d, -d, -d],
        [-d, -d, d, d, -d],
        'cyan'
    )

    plt.scatter(0, 0, color='cyan', s=40)

    plt.colorbar(label="log10(Intensity)")
    plt.title(f"Beam Intensity (Eq. 20) {math.degrees(theta):.2f}-Degrees")
    plt.xlabel("Drone x-face (m)")
    plt.ylabel("Drone y-face (m)")
    plt.axis("equal")

    plt.show()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_handler)

# -----------------------------
# LOOP
# -----------------------------
tracking = False
reading = False

while True:
    line = proc.stdout.readline()
    if not line:
        continue

    line = line.strip()

    if 'name:' in line:
        tracking = ('x500_0' in line)

    if not tracking:
        continue

    if 'position' in line:
        reading = True
        x = y = z = None
        continue

    if reading:

        if 'x:' in line:
            try: x = float(line.split(':')[1])
            except: x = None

        elif 'y:' in line:
            try: y = float(line.split(':')[1])
            except: y = None

        elif 'z:' in line:
            try: z = float(line.split(':')[1])
            except: z = None
            reading = False

            if None in (x, y, z):
                continue

            filtered = smooth(x, y, z)
            filtered_com.append(filtered)

            print(f"CoM: {filtered}")