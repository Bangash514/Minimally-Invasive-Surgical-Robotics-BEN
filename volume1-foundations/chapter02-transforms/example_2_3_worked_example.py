"""
Worked Example 2.3 — Frame Chain
Volume 1, Chapter 2: Coordinate Frames and Homogeneous Transforms

Problem:
A point lies at (1, 0, 0) in frame B. Frame B is rotated 90° about
Z-axis of frame A, and B's origin sits at (2, 0, 0) in frame A.
Where is this point in frame A?

Expected Answer: (2, 1, 0)

Math by hand:
  R_AB = [cos90°  -sin90°   0]     [0  -1   0]
         [sin90°   cos90°   0]  =  [1   0   0]
         [  0        0      1]     [0   0   1]

  R_AB · p_B = [0, 1, 0]
  p_A = R_AB · p_B + t_AB = (0, 1, 0) + (2, 0, 0) = (2, 1, 0) ✓

Author: BEN (班班)
Date: May 6, 2026
Status: Verified working
"""

import numpy as np


def make_transform(R, t):
    """Build a 4x4 homogeneous transform from R (3x3) and t (3,)."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ============================================================
# Build the transformation T_AB
# ============================================================

# 90 degree rotation about Z
theta = np.deg2rad(90)
R_AB = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0,              0,             1]
])

# B's origin in A
t_AB = np.array([2, 0, 0])

# Build T_AB
T_AB = make_transform(R_AB, t_AB)

# ============================================================
# Apply transformation to point
# ============================================================

# Point in B (homogeneous)
p_B = np.array([1, 0, 0, 1])

# Transform to A
p_A = T_AB @ p_B

# ============================================================
# Display results
# ============================================================

print("=" * 50)
print("Worked Example 2.3: Frame Chain")
print("=" * 50)

print(f"\nPoint in B: {p_B[:3]}")
print(f"Point in A: {p_A[:3]}")
print(f"Expected:   [2 1 0]")


# ============================================================
# VERIFIED OUTPUT — May 6, 2026
# Run on Windows + Python 3.13 + NumPy
# ============================================================
"""
==================================================
Worked Example 2.3: Frame Chain
==================================================

Point in B: [1 0 0]
Point in A: [2. 1. 0.]
Expected:   [2 1 0]

NOTES:
- The rotation moves "1 unit along B's x-axis" to "1 unit along A's y-axis"
- Then translation adds B's origin position (2, 0, 0)
- Final result: (2, 1, 0) in frame A — matches expected answer
"""
