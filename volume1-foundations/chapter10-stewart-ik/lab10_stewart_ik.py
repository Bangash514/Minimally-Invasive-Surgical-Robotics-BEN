# BEN 2026-11-01
# Change aim: Compute Stewart platform leg lengths and Jacobian on a non-degenerate joint layout.
# File: lab10_stewart_ik.py
"""
Lab 10: Stewart Platform Inverse Kinematics
Volume 1, Chapter 10

What this lab does:
- Compute all 6 leg lengths for a given platform pose
- Test multiple poses (home, translate, tilt, out-of-range)
- Check leg length limits
- Compute platform Jacobian

Author: BEN (班班)
Date: May 2026 (revised 2026-11-01: non-degenerate joint layout)
"""

import numpy as np
from scipy.spatial.transform import Rotation


class StewartPlatformIK:
    def __init__(self, base_points, platform_points,
                 leg_min, leg_max):
        self.a = np.array(base_points)
        self.b = np.array(platform_points)
        self.leg_min = leg_min
        self.leg_max = leg_max

    def compute(self, position, rotation_matrix):
        p = np.array(position)
        R = np.array(rotation_matrix)
        leg_lengths = np.zeros(6)
        for i in range(6):
            B_i = p + R @ self.b[i]
            leg_lengths[i] = np.linalg.norm(B_i - self.a[i])
        valid = (np.all(leg_lengths >= self.leg_min) and
                 np.all(leg_lengths <= self.leg_max))
        return leg_lengths, valid

    def compute_from_euler(self, position,
                           euler_angles_deg):
        R = Rotation.from_euler(
            'xyz', euler_angles_deg, degrees=True
        ).as_matrix()
        return self.compute(position, R)

    def compute_jacobian(self, position,
                         rotation_matrix):
        p = np.array(position)
        R = np.array(rotation_matrix)
        Jp = np.zeros((6, 6))
        for i in range(6):
            B_i = p + R @ self.b[i]
            leg_vec = B_i - self.a[i]
            u_i = leg_vec / np.linalg.norm(leg_vec)
            r_i = R @ self.b[i]
            Jp[i, :3] = u_i
            Jp[i, 3:] = np.cross(r_i, u_i)
        return Jp


def make_hexagonal_points(radius, angle_offset_deg=0):
    # NOTE: this 6-fold symmetric layout makes the home-pose Jacobian rank 3 (singular).
    # Use make_stewart_layout() for the labs.
    points = []
    for i in range(6):
        angle = np.deg2rad(60 * i + angle_offset_deg)
        points.append([radius * np.cos(angle),
                       radius * np.sin(angle), 0.0])
    return np.array(points)


# BEN 2026-11-01
# Change aim: Build 3 pairs of joints on a circle (3-3 layout) so the Jacobian is not degenerate.
# File: lab10_stewart_ik.py
# Function: make_paired_points
def make_paired_points(radius, half_angle_deg=10.0, centre_offset_deg=0.0):
    """Six points as three pairs; each pair sits +/- half_angle around 0, 120 and 240 deg."""
    points = []
    for k in range(3):
        centre = centre_offset_deg + 120 * k
        for sign in (-1, 1):
            angle = np.deg2rad(centre + sign * half_angle_deg)
            points.append([radius * np.cos(angle),
                           radius * np.sin(angle), 0.0])
    return np.array(points)


# BEN 2026-11-01
# Change aim: Return base and platform joints whose legs cross to the neighbouring pair.
# File: lab10_stewart_ik.py
# Function: make_stewart_layout
def make_stewart_layout(base_radius=0.15, platform_radius=0.10,
                        half_angle_deg=10.0):
    """Base pairs at 0/120/240 deg, platform pairs at 60/180/300 deg, rolled by one joint.
    At the home pose the Jacobian has full rank 6 (condition number about 26)."""
    base = make_paired_points(base_radius, half_angle_deg, 0.0)
    platform = np.roll(
        make_paired_points(platform_radius, half_angle_deg, 60.0), 1, axis=0)
    return base, platform



if __name__ == "__main__":
    # Setup
    base_pts, platform_pts = make_stewart_layout()
    ik = StewartPlatformIK(base_pts, platform_pts,
                           0.20, 0.35)

    # Tests
    test_cases = [
        ([0.00, 0.00, 0.25], [0,  0, 0], "Home"),
        ([0.00, 0.00, 0.27], [0,  0, 0], "Translate up"),
        ([0.02, 0.00, 0.25], [0,  0, 0], "Translate X"),
        ([0.00, 0.00, 0.25], [10, 0, 0], "Tilt X 10deg"),
        ([0.00, 0.00, 0.40], [0,  0, 0], "Out of range"),
    ]

    print("=" * 55)
    print("Lab 10: Stewart Platform IK")
    print("=" * 55)

    for pos, euler, name in test_cases:
        lengths, valid = ik.compute_from_euler(pos, euler)
        print(f"\n{name}:")
        print(f"  Leg lengths (m): {np.round(lengths, 4)}")
        print(f"  Within limits:   {valid}")

    # Jacobian at home
    Jp   = ik.compute_jacobian([0, 0, 0.25], np.eye(3))
    cond = np.linalg.cond(Jp)
    print(f"\nJacobian rank at home:             {np.linalg.matrix_rank(Jp)} of 6")
    print(f"Jacobian condition number at home: {cond:.2f}")
