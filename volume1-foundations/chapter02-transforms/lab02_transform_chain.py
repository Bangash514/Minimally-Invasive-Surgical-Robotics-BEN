"""
Lab 02: Transform Chain — Build, Invert, and Round-Trip
Volume 1, Chapter 2: Coordinate Frames and Homogeneous Transforms

What this lab does:
- Build a 4x4 homogeneous transform from R and t
- Apply the transform to a point
- Invert the transform using closed-form formula
- Verify the round-trip: B → A → B should give back the original point
- Solve Problem 2.1: Biopsy target location in patient frame

Author: BEN (班班)
Date: May 6, 2026
Status: Verified working on Windows + Python 3.10
"""

import numpy as np


def make_transform(R, t):
    """Build a 4x4 homogeneous transform from R (3x3) and t (3,)."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_transform(T):
    """
    Closed-form inverse of a homogeneous transform.

    For rigid transforms, this is faster and more accurate than
    np.linalg.inv() because it uses the structure of the matrix:
        T_inv = [R^T  -R^T·t]
                [ 0     1   ]
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T          # R^T
    T_inv[:3, 3] = -R.T @ t      # -R^T @ t
    return T_inv


# ============================================================
# Build T_AB: 90° rotation about Z, translation (2, 0, 0)
# ============================================================

theta = np.deg2rad(90)
R_AB = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0,              0,             1]
])
t_AB = np.array([2, 0, 0])
T_AB = make_transform(R_AB, t_AB)

# Point in frame B (homogeneous form)
p_B_h = np.array([1, 0, 0, 1])

# Transform to frame A
p_A_h = T_AB @ p_B_h

print("=" * 50)
print("Lab 02: Transform Chain")
print("=" * 50)

print("\nT_AB:")
print(T_AB)

print(f"\nPoint in frame B (homogeneous): {p_B_h}")
print(f"Point in frame A (homogeneous): {p_A_h}")
print(f"Point in frame A (3D):          {p_A_h[:3]}")

# ============================================================
# Verify inverse: p_A back to p_B
# ============================================================

T_BA = invert_transform(T_AB)
p_B_recovered = T_BA @ p_A_h

print("\n" + "=" * 50)
print("Inverse Round-Trip Check")
print("=" * 50)

print(f"\nT_BA (closed-form inverse):")
print(T_BA)

print(f"\nRecovered p_B from p_A: {p_B_recovered}")
print(f"Original p_B:           {p_B_h}")
print(f"Match: {np.allclose(p_B_h, p_B_recovered)}")

# ============================================================
# Verify chain rule: T_AB @ T_BA should equal identity
# ============================================================

print("\n" + "=" * 50)
print("Identity Check")
print("=" * 50)

print(f"\nT_AB @ T_BA (should be identity):")
print(np.round(T_AB @ T_BA, 6))


# ============================================================
# Solution to Problem 2.1 — Biopsy Target in Patient Frame
# ============================================================

print("\n" + "=" * 50)
print("Problem 2.1: Biopsy Target in Patient Frame")
print("=" * 50)

# 180 degree rotation about Z-axis
R_patient_mri = np.array([
    [-1,  0, 0],
    [ 0, -1, 0],
    [ 0,  0, 1]
])

# Translation: MRI origin in patient frame
t_patient_mri = np.array([100, 100, 0])

T_patient_mri = make_transform(R_patient_mri, t_patient_mri)

# Biopsy target in MRI frame (homogeneous)
p_mri_h = np.array([50, 30, 20, 1])

# Transform to patient frame
p_patient_h = T_patient_mri @ p_mri_h

print(f"\nBiopsy target in MRI frame:     {p_mri_h[:3]} mm")
print(f"Biopsy target in patient frame: {p_patient_h[:3]} mm")
print(f"Expected: [50, 70, 20] mm")


# ============================================================
# VERIFIED OUTPUT — May 6, 2026
# Run on Windows + Python 3.10 + NumPy
# ============================================================
"""
==================================================
Lab 02: Transform Chain
==================================================
T_AB:
[[ 6.123234e-17 -1.000000e+00  0.000000e+00  2.000000e+00]
 [ 1.000000e+00  6.123234e-17  0.000000e+00  0.000000e+00]
 [ 0.000000e+00  0.000000e+00  1.000000e+00  0.000000e+00]
 [ 0.000000e+00  0.000000e+00  0.000000e+00  1.000000e+00]]

Point in frame B (homogeneous): [1 0 0 1]
Point in frame A (homogeneous): [2. 1. 0. 1.]
Point in frame A (3D):          [2. 1. 0.]

==================================================
Inverse Round-Trip Check
==================================================
T_BA (closed-form inverse):
[[ 6.1232340e-17  1.0000000e+00  0.0000000e+00 -1.2246468e-16]
 [-1.0000000e+00  6.1232340e-17  0.0000000e+00  2.0000000e+00]
 [ 0.0000000e+00  0.0000000e+00  1.0000000e+00  0.0000000e+00]
 [ 0.0000000e+00  0.0000000e+00  0.0000000e+00  1.0000000e+00]]

Recovered p_B from p_A: [1. 0. 0. 1.]
Original p_B:           [1 0 0 1]
Match: True

==================================================
Identity Check
==================================================
T_AB @ T_BA (should be identity):
[[1. 0. 0. 0.]
 [0. 1. 0. 0.]
 [0. 0. 1. 0.]
 [0. 0. 0. 1.]]

==================================================
Problem 2.1: Biopsy Target in Patient Frame
==================================================
Biopsy target in MRI frame:     [50 30 20] mm
Biopsy target in patient frame: [50. 70. 20.] mm
Expected: [50, 70, 20] mm

NOTES:
- Tiny values like 6.123234e-17 are floating-point representations of
  what should be exactly 0. This is normal numerical behavior.
- All round-trip checks pass: code is mathematically correct.
- Problem 2.1 result matches expected answer.
"""
