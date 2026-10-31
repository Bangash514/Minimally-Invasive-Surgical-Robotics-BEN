# BEN 2026-11-01
# Change aim: Remove the product-style name "Gen-3" from the description.
# File: lab03_chain_three_frames.py
"""
Lab 03: Three-Frame Chain — World → Robot → End-Effector → Probe
Volume 1, Chapter 2: Coordinate Frames and Homogeneous Transforms

What this lab does:
- Build a chain of three transforms representing a surgical robot scenario
- Compose them to get T_world_probe
- Extract the probe's position in the world frame

This is exactly how you compute the probe's pose for needle targeting.

Author: BEN (班班)
Date: May 6, 2026
"""

import numpy as np


def make_transform(R, t):
    """Build a 4x4 homogeneous transform from R (3x3) and t (3,)."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def Rz(deg):
    """Rotation matrix about Z-axis by given degrees."""
    th = np.deg2rad(deg)
    c, s = np.cos(th), np.sin(th)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])


# ============================================================
# Build the chain: world → robot → end-effector → probe
# ============================================================

# Robot is at world origin, no rotation
T_world_robot = make_transform(Rz(0), np.array([0, 0, 0]))

# End-effector is at (0.5, 0, 0.3) m relative to robot, rotated 45° about Z
T_robot_ee = make_transform(Rz(45), np.array([0.5, 0, 0.3]))

# Probe is mounted on end-effector, 0.1 m above (along Z)
T_ee_probe = make_transform(Rz(0), np.array([0, 0, 0.1]))

# ============================================================
# Compose the chain
# ============================================================

T_world_probe = T_world_robot @ T_robot_ee @ T_ee_probe

print("=" * 50)
print("Lab 03: Three-Frame Chain")
print("=" * 50)

print("\nT_world_robot:")
print(T_world_robot)

print("\nT_robot_ee:")
print(T_robot_ee)

print("\nT_ee_probe:")
print(T_ee_probe)

print("\nT_world_probe (composed):")
print(np.round(T_world_probe, 4))

# ============================================================
# Extract probe position in world frame
# ============================================================

probe_position = T_world_probe[:3, 3]
print(f"\nProbe origin in world frame: {np.round(probe_position, 4)} m")

# ============================================================
# Bonus: Verify with a different test point
# ============================================================

# A point at the probe tip (0, 0, 0.05) in probe frame (5cm above probe origin)
p_probe = np.array([0, 0, 0.05, 1])

# Transform to world
p_world = T_world_probe @ p_probe

print(f"\nProbe tip at (0, 0, 0.05) in probe frame")
print(f"Probe tip in world frame: {np.round(p_world[:3], 4)} m")


# ============================================================
# VERIFIED OUTPUT — May 6, 2026
# Run on Windows + Python 3.13 + NumPy
# ============================================================
"""
==================================================
Lab 03: Three-Frame Chain
==================================================

T_world_robot:
[[ 1. -0.  0.  0.]
 [ 0.  1.  0.  0.]
 [ 0.  0.  1.  0.]
 [ 0.  0.  0.  1.]]

T_robot_ee:
[[ 0.70710678 -0.70710678  0.          0.5       ]
 [ 0.70710678  0.70710678  0.          0.        ]
 [ 0.          0.          1.          0.3       ]
 [ 0.          0.          0.          1.        ]]

T_ee_probe:
[[ 1.  -0.   0.   0. ]
 [ 0.   1.   0.   0. ]
 [ 0.   0.   1.   0.1]
 [ 0.   0.   0.   1. ]]

T_world_probe (composed):
[[ 0.7071 -0.7071  0.      0.5   ]
 [ 0.7071  0.7071  0.      0.    ]
 [ 0.      0.      1.      0.4   ]
 [ 0.      0.      0.      1.    ]]

Probe origin in world frame: [0.5  0.   0.4 ] m

Probe tip at (0, 0, 0.05) in probe frame
Probe tip in world frame: [0.5  0.   0.45] m

NOTES:
- 0.7071 = 1/sqrt(2) = cos(45°) = sin(45°)
- Probe origin makes sense: 0.5m in x (from EE), 0.4m in z (0.3 + 0.1)
- Probe tip is 0.05m higher than origin: 0.4 + 0.05 = 0.45 m
- All results verified mathematically correct.
"""
