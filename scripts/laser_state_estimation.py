#!/usr/bin/env python3

import subprocess
import numpy as np
import matplotlib.pyplot as plt
import math
from collections import deque

print("Inverse Beam Reconstruction System")

# -----------------------------
# STREAM (Gazebo)
# -----------------------------
proc = subprocess.Popen(
    ["gz", "topic", "-e", "-t", "/world/default/pose/info"],
    stdout=subprocess.PIPE,
    text=True,
    bufsize=1
)

# -----------------------------
# STATE STORAGE
# -----------------------------
alpha_smooth = 0.15
prev_filtered = None
filtered_com = deque(maxlen=2000)

step_counter = 0
save_index = 0
save_interval = 100


# -----------------------------
# SMOOTHING
# -----------------------------
def smooth(x, y, z):
    global prev_filtered
    cur = np.array([x, y, z])

    if prev_filtered is None:
        prev_filtered = cur
    else:
        prev_filtered = alpha_smooth * cur + (1 - alpha_smooth) * prev_filtered

    return prev_filtered


# -----------------------------
# OPTICS MODEL (FORWARD)
# -----------------------------
def beam_intensity(P0, alpha, z, w, X, Y, theta, phi, x0, y0):

    I0 = P0 * np.exp(-alpha * z)

    cos_t = np.clip(np.cos(theta), 1e-3, 1.0)
    cos_p = np.clip(np.cos(phi),   1e-3, 1.0)

    Xr = X * cos_p + Y * sin_p
    Yr = -X * sin_p + Y * cos_p
    Yr = Yr / cos_t

    Xr -= x0
    Yr -= y0

    r2 = Xr**2 + Yr**2

    prefactor = (2 * I0 * cos_t * cos_p) / (math.pi * w**2)

    return prefactor * np.exp(-2 * r2 / (w**2))


# -----------------------------
# FORWARD FIELD
# -----------------------------
def beam_field(P0, alpha, z, w, theta, phi, grid=300, span=0.5):

    x = np.linspace(-span, span, grid)
    y = np.linspace(-span, span, grid)

    X, Y = np.meshgrid(x, y)

    # assume beam centered at origin in forward model
    I = beam_intensity(P0, alpha, z, w, X, Y, theta, phi, 0.0, 0.0)

    return X, Y, I


# -----------------------------
# INVERSE ESTIMATION (CORE)
# -----------------------------
def estimate_pose_from_field(X, Y, I, P0, alpha):

    I = np.nan_to_num(I, nan=0.0, posinf=0.0, neginf=0.0)
    I = np.clip(I, 1e-20, None)

    # -------------------------
    # 1. CENTROID → (x, y)
    # -------------------------
    total = np.sum(I)
    x_est = np.sum(X * I) / total
    y_est = np.sum(Y * I) / total

    # -------------------------
    # 2. SPREAD → z estimate
    # (beam widening model inversion)
    # -------------------------
    sigma2 = np.sum((X**2 + Y**2) * I) / total
    sigma = np.sqrt(sigma2)

    # invert Gaussian beam spread approximation
    z_est = sigma * 100.0   # scaling factor from optics model

    # -------------------------
    # 3. ANGLES FROM GEOMETRY
    # -------------------------
    theta = math.atan2(y_est, z_est)
    phi   = math.atan2(y_est, x_est)

    return x_est, y_est, z_est, theta, phi


# -----------------------------
# PLOT
# -----------------------------
def save_snapshot(X, Y, I, est, save_index):

    x, y, z, theta, phi = est

    plt.figure(figsize=(6, 5))
    plt.contourf(X, Y, np.log10(I + 1e-20), levels=120, cmap="turbo")
    plt.colorbar(label="log10 intensity")

    plt.scatter(x, y, c='red', s=60, label="Estimated Impact")

    plt.title(
        f"Estimated Pose\n"
        f"x={x:.3f}, y={y:.3f}, z={z:.3f}\n"
        f"θ={math.degrees(theta):.2f}°, φ={math.degrees(phi):.2f}°"
    )

    plt.axis("equal")
    plt.legend()

    plt.savefig(f"inverse_{save_index:04d}.png", dpi=150)
    plt.close()


# -----------------------------
# PARSE LOOP
# -----------------------------
tracking = False
reading = False

x = y = z = None

while True:
    line = proc.stdout.readline()
    if not line:
        continue

    line = line.strip()

    if "name:" in line:
        tracking = ("x500_0" in line)

    if not tracking:
        continue

    if "position" in line:
        reading = True
        x = y = z = None
        continue

    if reading:

        if "x:" in line:
            x = float(line.split(":")[1])

        elif "y:" in line:
            y = float(line.split(":")[1])

        elif "z:" in line:
            z = float(line.split(":")[1])
            reading = False

            if None in (x, y, z):
                continue

            filtered = smooth(x, y, z)
            filtered_com.append(filtered)

            step_counter += 1

            # -----------------------------
            # ONLY PROCESS EVERY N STEPS
            # -----------------------------
            if step_counter % save_interval == 0:

                px, py, pz = filtered

                dist = np.linalg.norm(filtered)

                # -----------------------------
                # true geometry (unknown in real use)
                # -----------------------------
                theta_true = math.atan2(py, pz)
                phi_true   = math.atan2(py, px)

                # optics scaling
                w = 0.05 + 0.001 * dist

                # -----------------------------
                # FORWARD SIMULATION
                # -----------------------------
                X, Y, I = beam_field(
                    P0=5000.0,
                    alpha=0.00015,
                    z=dist,
                    w=w,
                    theta=theta_true,
                    phi=phi_true
                )

                # -----------------------------
                # INVERSE ESTIMATION
                # -----------------------------
                est = estimate_pose_from_field(X, Y, I, 5000.0, 0.00015)

                ex, ey, ez, etheta, ephi = est

                print("\n--- INVERSE RESULT ---")
                print(f"True:  x={px:.3f}, y={py:.3f}, z={pz:.3f}")
                print(f"Est :  x={ex:.3f}, y={ey:.3f}, z={ez:.3f}")
                print(f"θ={math.degrees(etheta):.2f}°, φ={math.degrees(ephi):.2f}°")

                save_snapshot(X, Y, I, est, save_index)

                save_index += 1