# BEN 2026-11-01
# Change aim: Solve Stewart platform forward kinematics with a damped Newton method.
# File: lab11_stewart_fk.py
"""
Lab 11: Stewart Platform Forward Kinematics
Volume 1, Chapter 11

What this lab does:
- Solve FK using Newton-Raphson iteration
- Verify: IK then FK recovers original pose
- Test multiple configurations

Author: BEN (班班)
Date: May 2026 (revised 2026-11-01: damped Newton, non-degenerate layout)
"""

import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "chapter10-stewart-ik"))
from lab10_stewart_ik import StewartPlatformIK, make_stewart_layout


class StewartPlatformFK:
    def __init__(self, base_points, platform_points):
        self.a = np.array(base_points)
        self.b = np.array(platform_points)

    def constraints(self, pose_vec, leg_lengths):
        p = pose_vec[:3]
        R = Rotation.from_euler(
            'xyz', pose_vec[3:]
        ).as_matrix()
        residuals = np.zeros(6)
        for i in range(6):
            B_i = p + R @ self.b[i]
            leg_vec = B_i - self.a[i]
            residuals[i] = (np.dot(leg_vec, leg_vec)
                            - leg_lengths[i]**2)
        return residuals

    def jacobian_numerical(self, pose_vec,
                           leg_lengths, h=1e-7):
        n = len(pose_vec)
        J = np.zeros((6, n))
        f0 = self.constraints(pose_vec, leg_lengths)
        for j in range(n):
            pose_plus = pose_vec.copy()
            pose_plus[j] += h
            J[:, j] = (self.constraints(
                pose_plus, leg_lengths) - f0) / h
        return J

    # BEN 2026-11-01
    # Change aim: Newton iteration with damping (Levenberg-Marquardt) that only accepts steps
    #             which reduce the residual, so it does not diverge near singular poses.
    # File: lab11_stewart_fk.py
    # Function: solve
    def solve(self, leg_lengths, initial_guess=None,
              tol=1e-10, max_iter=100):
        if initial_guess is None:
            initial_guess = np.array(
                [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]
            )
        x = initial_guess.astype(float).copy()
        lam = 1e-3
        r = self.constraints(x, leg_lengths)
        for iteration in range(max_iter):
            if np.linalg.norm(r) < tol:
                return x, True, iteration
            J = self.jacobian_numerical(x, leg_lengths)
            A = J.T @ J + lam * np.eye(len(x))
            try:
                dx = np.linalg.solve(A, -J.T @ r)
            except np.linalg.LinAlgError:
                return x, False, iteration
            x_new = x + dx
            r_new = self.constraints(x_new, leg_lengths)
            if np.linalg.norm(r_new) < np.linalg.norm(r):
                x, r = x_new, r_new
                lam = max(lam / 10.0, 1e-12)       # good step: trust Newton more
            else:
                lam = lam * 10.0                   # bad step: damp more
                if lam > 1e12:
                    break
        return x, bool(np.linalg.norm(r) < tol), max_iter


def make_hexagonal_points(radius, angle_offset_deg=0):
    points = []
    for i in range(6):
        angle = np.deg2rad(60 * i + angle_offset_deg)
        points.append([radius * np.cos(angle),
                       radius * np.sin(angle), 0.0])
    return np.array(points)


# Reuse IK and joint layout from lab10
base_pts, platform_pts = make_stewart_layout()
ik = StewartPlatformIK(base_pts, platform_pts,
                       0.20, 0.35)
fk = StewartPlatformFK(base_pts, platform_pts)

test_poses = [
    ([0.00, 0.00, 0.25], [0,  0, 0], "Home"),
    ([0.00, 0.00, 0.27], [0,  0, 0], "Up 2cm"),
    ([0.02, 0.00, 0.25], [0,  0, 0], "X 2cm"),
    ([0.00, 0.00, 0.25], [10, 0, 0], "Tilt X 10deg"),
    ([0.01, 0.01, 0.26], [5,  5, 0], "Combined"),
]

print("=" * 55)
print("Lab 11: Stewart Platform FK — damped Newton")
print("=" * 55)

for pos, euler, name in test_poses:
    R_true = Rotation.from_euler(
        'xyz', euler, degrees=True
    ).as_matrix()
    lengths, valid = ik.compute(pos, R_true)
    if not valid:
        print(f"{name}: Outside workspace")
        continue

    pose_rec, converged, iters = fk.solve(
        lengths,
        initial_guess=np.array([0,0,0.25,0,0,0],
                                dtype=float)
    )

    pos_err = np.linalg.norm(
        pose_rec[:3] - np.array(pos)
    ) * 1000
    ang_err = np.linalg.norm(
        np.rad2deg(pose_rec[3:]) - np.array(euler)
    )

    print(f"\n{name}:")
    print(f"  Converged:       {converged} "
          f"({iters} iters)")
    print(f"  Position error:  {pos_err:.4f} mm")
    print(f"  Angle error:     {ang_err:.4f} deg")
    print(f"  Accurate:        "
          f"{pos_err < 0.1 and ang_err < 0.01}")
