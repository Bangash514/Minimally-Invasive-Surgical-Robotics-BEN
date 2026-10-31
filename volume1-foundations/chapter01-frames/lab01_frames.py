"""
Lab 01: Coordinate Frames Basic Practice
Volume 1, Chapter 1: Notation and Frames

What this lab does:
- Define a point in frame B
- Define a transformation T_AB (frame B expressed in frame A)
- Convert the point from frame B to frame A
- Verify the math by computing the inverse transformation

Author: BEN (班班)
Date: May 6, 2026
"""

import numpy as np


def to_homogeneous(p):
    """Convert 3D point [x, y, z] to homogeneous [x, y, z, 1]."""
    return np.append(p, 1.0)


def from_homogeneous(p_h):
    """Convert homogeneous [x, y, z, 1] back to 3D [x, y, z]."""
    return p_h[:3]


def transform_point(T, p):
    """Apply 4x4 transformation matrix T to 3D point p."""
    p_h = to_homogeneous(p)
    p_transformed_h = T @ p_h
    return from_homogeneous(p_transformed_h)


# ============================================================
# Example: Frame B is rotated 90 degrees around Z relative to A
# Plus a translation of 1 unit along x-axis of A
# ============================================================

T_AB = np.array([
    [0, -1, 0, 1],   # X-axis of B in A: (0, 1, 0)
    [1,  0, 0, 0],   # Y-axis of B in A: (-1, 0, 0)
    [0,  0, 1, 0],   # Z-axis of B in A: (0, 0, 1) - same as A
    [0,  0, 0, 1]    # Homogeneous row
])

# A point at (2, 0, 0) in frame B
p_B = np.array([2.0, 0.0, 0.0])

# Convert to frame A
p_A = transform_point(T_AB, p_B)

print("=" * 50)
print("Lab 01: Coordinate Frame Transformation")
print("=" * 50)

print("\nTransformation matrix T_AB:")
print(T_AB)

print(f"\nPoint in frame B: {p_B}")
print(f"Point in frame A: {p_A}")
print("Expected result: [1, 2, 0]")
print("(translation 1 in x + 2 units rotated to A's y-axis)")

# ============================================================
# Verify by computing the inverse transformation
# ============================================================

T_BA = np.linalg.inv(T_AB)
p_B_recovered = transform_point(T_BA, p_A)

print("\n" + "=" * 50)
print("Inverse Check")
print("=" * 50)

print(f"\nInverse matrix T_BA:")
print(T_BA)

print(f"\nRecovered point in frame B: {p_B_recovered}")
print(f"Original point in frame B:  {p_B}")
print(f"Match: {np.allclose(p_B, p_B_recovered)}")

# ============================================================
# Practice Problem 1.4 — Probe to Robot Frame Transformation
# ============================================================

print("\n" + "=" * 50)
print("Practice Problem 1.4: Probe to Robot Frame")
print("=" * 50)

# Probe is rotated 90 degrees about robot x-axis
# Plus translation of 0.05 m (5 cm) along robot z-axis
T_robot_probe = np.array([
    [1, 0,  0, 0],
    [0, 0, -1, 0],
    [0, 1,  0, 0.05],
    [0, 0,  0, 1]
])

print("\nT_robot_probe:")
print(T_robot_probe)

# Test: a point at the probe origin (0, 0, 0)
# should be at (0, 0, 0.05) in robot frame
p_probe_origin = np.array([0.0, 0.0, 0.0])
p_robot = transform_point(T_robot_probe, p_probe_origin)

print(f"\nProbe origin in robot frame: {p_robot}")
print("Expected: [0, 0, 0.05] (5 cm above robot origin along z)")


# ============================================================
# VERIFIED OUTPUT (May 6, 2026)
# ============================================================
# ==================================================
# Lab 01: Coordinate Frame Transformation
# ==================================================
# Transformation matrix T_AB:
# [[ 0 -1  0  1]
#  [ 1  0  0  0]
#  [ 0  0  1  0]
#  [ 0  0  0  1]]
# Point in frame B: [2. 0. 0.]
# Point in frame A: [1. 2. 0.]
# Expected result: [1, 2, 0]
#
# ==================================================
# Inverse Check
# ==================================================
# Recovered point in frame B: [2. 0. 0.]
# Original point in frame B:  [2. 0. 0.]
# Match: True
#
# ==================================================
# Practice Problem 1.4: Probe to Robot Frame
# ==================================================
# Probe origin in robot frame: [0.   0.   0.05]
# Expected: [0, 0, 0.05] ✓
