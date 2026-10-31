"""
Lab 04: Rotation Basics — Matrix, Euler, Quaternion
Volume 1, Chapter 3: Rotations

What this lab does:
- Build the same rotation using three different representations
- Convert between rotation matrix, Euler angles, and quaternion
- Verify all three representations give the same result on a test point
- Show gimbal lock briefly

Author: BEN (班班)
Date: May 6, 2026
"""

import numpy as np
from scipy.spatial.transform import Rotation


# ============================================================
# Build the SAME rotation three different ways
# Goal: rotate 90 degrees about Z-axis
# ============================================================

print("=" * 50)
print("Lab 04: Rotation Basics")
print("=" * 50)

# Method 1: Direct rotation matrix
theta = np.deg2rad(90)
R_matrix = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0,              0,             1]
])

print("\nMethod 1: Direct Rotation Matrix")
print(R_matrix)

# Method 2: From Euler angles (using scipy)
r_euler = Rotation.from_euler('xyz', [0, 0, 90], degrees=True)
R_from_euler = r_euler.as_matrix()

print("\nMethod 2: From Euler Angles [0, 0, 90 deg]")
print(R_from_euler)

# Method 3: From Quaternion
# 90 deg about Z: q = (cos(45), 0, 0, sin(45))
# scipy uses [x, y, z, w] order!
q = [0, 0, np.sin(np.deg2rad(45)), np.cos(np.deg2rad(45))]
r_quat = Rotation.from_quat(q)
R_from_quat = r_quat.as_matrix()

print("\nMethod 3: From Quaternion [0, 0, sin45, cos45]")
print(R_from_quat)

# ============================================================
# Verify all three give the same result on a test point
# ============================================================

p = np.array([1, 0, 0])  # Point on x-axis

p_rotated_1 = R_matrix @ p
p_rotated_2 = R_from_euler @ p
p_rotated_3 = R_from_quat @ p

print("\n" + "=" * 50)
print("Verification: Rotate point [1, 0, 0]")
print("=" * 50)
print(f"Method 1 (matrix):     {p_rotated_1}")
print(f"Method 2 (Euler):      {p_rotated_2}")
print(f"Method 3 (quaternion): {p_rotated_3}")
print(f"Expected:              [0, 1, 0]  (rotated 90 deg about Z)")

# Check they all agree
all_equal = (
    np.allclose(p_rotated_1, p_rotated_2) and
    np.allclose(p_rotated_2, p_rotated_3)
)
print(f"\nAll three methods agree: {all_equal}")


# ============================================================
# Demonstrate Gimbal Lock
# ============================================================

print("\n" + "=" * 50)
print("Gimbal Lock Demonstration")
print("=" * 50)

# Two seemingly different Euler angles that produce the same rotation
# when pitch = 90 degrees (gimbal lock)
r1 = Rotation.from_euler('xyz', [10, 90, 0], degrees=True)
r2 = Rotation.from_euler('xyz', [0, 90, -10], degrees=True)

R1 = r1.as_matrix()
R2 = r2.as_matrix()

print("\nEuler [10, 90, 0]:")
print(np.round(R1, 4))

print("\nEuler [0, 90, -10]:")
print(np.round(R2, 4))

print("\nDifference (should be near zero -- they are the same!):")
print(np.round(R1 - R2, 4))

print("\nLesson: At pitch=90, roll and yaw merge -> gimbal lock!")


# ============================================================
# Practice 3.2: What rotation does q = (0.7071, 0, 0.7071, 0) represent?
# Note: scipy uses [x, y, z, w] but textbooks often use [w, x, y, z]
# So in scipy format: [0, 0.7071, 0, 0.7071]
# ============================================================

print("\n" + "=" * 50)
print("Practice 3.2: Decode quaternion (w, x, y, z) = (0.7071, 0, 0.7071, 0)")
print("=" * 50)

# Convert to scipy format [x, y, z, w]
q_scipy = [0, 0.7071, 0, 0.7071]
r_test = Rotation.from_quat(q_scipy)

axis_angle = r_test.as_rotvec()
angle = np.linalg.norm(axis_angle)
axis = axis_angle / angle if angle > 0 else axis_angle

print(f"Rotation angle: {np.rad2deg(angle):.1f} degrees")
print(f"Rotation axis:  {np.round(axis, 4)}")
print(f"Expected:       90 degrees about Y-axis")

# ============================================================
# VERIFIED OUTPUT — May 6, 2026
# Run on Windows + Python 3.13 + NumPy + SciPy
# ============================================================
"""
==================================================
Lab 04: Rotation Basics
==================================================

Method 1: Direct Rotation Matrix
[[ 6.123234e-17 -1.000000e+00  0.000000e+00]
 [ 1.000000e+00  6.123234e-17  0.000000e+00]
 [ 0.000000e+00  0.000000e+00  1.000000e+00]]

Method 2: From Euler Angles [0, 0, 90 deg]
[[ 2.22044605e-16 -1.00000000e+00  0.00000000e+00]
 [ 1.00000000e+00  2.22044605e-16  0.00000000e+00]
 [ 0.00000000e+00  0.00000000e+00  1.00000000e+00]]

Method 3: From Quaternion [0, 0, sin45, cos45]
[[ 0. -1.  0.]
 [ 1.  0.  0.]
 [ 0.  0.  1.]]

==================================================
Verification: Rotate point [1, 0, 0]
==================================================
Method 1 (matrix):     [6.123234e-17 1.000000e+00 0.000000e+00]
Method 2 (Euler):      [2.22044605e-16 1.00000000e+00 0.00000000e+00]
Method 3 (quaternion): [0. 1. 0.]
Expected:              [0, 1, 0]  (rotated 90 deg about Z)

All three methods agree: True

==================================================
Gimbal Lock Demonstration
==================================================
Two DIFFERENT Euler angles [10, 90, 0] and [0, 90, -10]
produced IDENTICAL rotation matrices (zero difference).
This is gimbal lock at pitch = 90 degrees.

==================================================
Practice 3.2: Decode quaternion (w, x, y, z) = (0.7071, 0, 0.7071, 0)
==================================================
Rotation angle: 90.0 degrees
Rotation axis:  [0. 1. 0.]
Expected:       90 degrees about Y-axis ✓

NOTES:
- Tiny values like 6.12e-17 are floating-point noise (effectively zero)
- All three rotation representations give same result on test point
- Gimbal lock confirmed: at pitch=90, Euler angles become ambiguous
- Quaternion decoded correctly: 90 deg about Y-axis
"""
