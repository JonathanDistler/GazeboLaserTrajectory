#!/usr/bin/env python3

import asyncio
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import time
from scipy.optimize import minimize

from mavsdk import System
from mavsdk.offboard import PositionNedYaw

# =========================================================
# CONSTANTS
# =========================================================
P0 = 5000.0
alpha_atm = 0.00015
wavelength = 1064e-9

D = 0.1
M2 = 1.5

# =========================================================
# GRID
# =========================================================
def grid():
    g = 140
    span = 0.6
    x = np.linspace(-span, span, g)
    y = np.linspace(-span, span, g)
    return np.meshgrid(x, y)

X, Y = grid()

# =========================================================
# PHYSICS
# =========================================================
def beam_waist(D, M2):
    return D / (math.pi * M2)

def beam_width(z):
    return math.sqrt((beam_waist(D, M2))**2 + (wavelength*z)**2)

# =========================================================
# ROTATION
# =========================================================
def rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s],
                     [-s, c]])

# =========================================================
# FORWARD MODEL
# =========================================================
def intensity(X, Y, x0, y0, z, yaw, theta):

    w = beam_width(z)
    I0 = P0 * np.exp(-alpha_atm * z)

    cos_t = np.clip(np.cos(theta), 0.2, 1.0)

    R = rot(yaw)
    Xr = R[0,0]*X + R[0,1]*Y
    Yr = R[1,0]*X + R[1,1]*Y

    Yr = Yr / cos_t

    Xr -= x0
    Yr -= y0

    r2 = Xr**2 + Yr**2

    return (2 * I0 * cos_t / (math.pi * w**2)) * np.exp(-2 * r2 / (w**2))

# =========================================================
# ANGLE WRAPPING (CRITICAL FIX)
# =========================================================
def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def angle_error(a, b):
    return abs(wrap_angle(a - b))

# =========================================================
# LOSS FUNCTION
# =========================================================
def loss(params, X, Y, I_obs, z):

    theta, phi = params

    x0 = 0.0
    y0 = 0.0

    I_pred = intensity(X, Y, x0, y0, z, phi, theta)

    lp = np.log(I_pred + 1e-20)
    lo = np.log(I_obs + 1e-20)

    return np.mean((lp - lo)**2)

# =========================================================
# ESTIMATOR
# =========================================================
def estimate_theta_phi(X, Y, I, z):

    best = None
    best_cost = 1e9

    for _ in range(4):

        init = [
            random.uniform(0.1, 1.2),
            random.uniform(-math.pi, math.pi)
        ]

        res = minimize(
            loss,
            x0=init,
            args=(X, Y, I, z),
            method="Nelder-Mead",
            options={"maxiter": 60}
        )

        if res.fun < best_cost:
            best_cost = res.fun
            best = res.x

    return best, best_cost

# =========================================================
# VISUALIZATION
# =========================================================
def plot_intensity(I, step, theta_est, phi_est, err, t_compute):

    I_log = np.log10(I + 1e-20)

    p5, p99 = np.percentile(I_log, [5, 99])
    I_clip = np.clip(I_log, p5, p99)

    I_norm = (I_clip - p5) / (p99 - p5 + 1e-12)
    I_vis = np.power(I_norm, 0.65)

    plt.figure(figsize=(6,5))
    plt.imshow(I_vis, cmap="turbo", origin="lower")
    plt.colorbar(label="log intensity")

    plt.title(
        f"Step {step}\n"
        f"θ={theta_est:.3f}, φ={phi_est:.3f}\n"
        f"err={err:.4f} | compute={t_compute*1000:.2f} ms"
    )

    plt.tight_layout()
    plt.savefig(f"beam_{step:04d}.png", dpi=170)
    plt.close()

# =========================================================
# MAIN
# =========================================================
async def run():

    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(6)

    target = PositionNedYaw(80, 80, -40, 0)

    for _ in range(15):
        await drone.offboard.set_position_ned(target)
        await asyncio.sleep(0.05)

    await drone.offboard.start()
    await asyncio.sleep(8)

    x0 = random.uniform(-0.25, 0.25)
    y0 = random.uniform(-0.25, 0.25)

    print(f"Laser hit: {x0:.3f}, {y0:.3f}")

    step = 0

    async for pos in drone.telemetry.position_velocity_ned():

        z = abs(pos.position.down_m)
        dist = max(z, 1e-3)

        yaw_true = math.atan2(pos.position.east_m,
                              pos.position.north_m)

        theta_true = math.atan2(
            np.linalg.norm([pos.position.north_m,
                            pos.position.east_m]),
            dist
        )

        I = intensity(X, Y, x0, y0, dist, yaw_true, theta_true)

        t0 = time.perf_counter()

        (theta_est, phi_est), cost = estimate_theta_phi(X, Y, I, dist)

        t1 = time.perf_counter()
        compute_time = t1 - t0

        # ✅ FIXED ERROR METRIC (THIS IS THE IMPORTANT PART)
        err_theta = abs(theta_est - theta_true)
        err_phi = angle_error(phi_est, yaw_true)

        err = math.sqrt(err_theta**2 + err_phi**2)

        if step % 20 == 0:
            print("\n---------------------")
            print(f"Step {step}")
            print("TRUE θ, φ:", theta_true, yaw_true)
            print("EST  θ, φ:", theta_est, phi_est)
            print("ERR      :", err)
            print("TIME (ms):", compute_time * 1000)

        if step % 100 == 0:
            plot_intensity(I, step, theta_est, phi_est, err, compute_time)

        step += 1
        await asyncio.sleep(0.05)

if __name__ == "__main__":
    asyncio.run(run())