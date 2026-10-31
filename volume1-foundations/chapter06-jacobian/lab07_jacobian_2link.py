# BEN 2026-11-01
# Change aim: Renumber this lab to 07 (the IK lab is already lab06).
# File: lab07_jacobian_2link.py
"""
Lab 07: Jacobian for 2-Link Planar Robot
Volume 1, Chapter 6: The Jacobian

What this lab does:
- Compute the analytical Jacobian for the 2-link planar robot
- Verify it numerically against finite differences
- Compute manipulability at various configurations
- Detect singularities (theta_2 = 0 or 180 deg)
- Demonstrate Damped Least Squares for near-singular configurations

Author: BEN (班班)
Date: May 9, 2026
"""

import numpy as np


def fk(theta_1, theta_2, L1=1.0, L2=1.0):
    """Forward kinematics: returns (x, y) end-effector position."""
    x = L1 * np.cos(theta_1) + L2 * np.cos(theta_1 + theta_2)
    y = L1 * np.sin(theta_1) + L2 * np.sin(theta_1 + theta_2)
    return np.array([x, y])


def jacobian_analytical(theta_1, theta_2, L1=1.0, L2=1.0):
    """Analytical Jacobian by hand-derived partial derivatives."""
    s1 = np.sin(theta_1)
    c1 = np.cos(theta_1)
    s12 = np.sin(theta_1 + theta_2)
    c12 = np.cos(theta_1 + theta_2)

    J = np.array([
        [-L1*s1 - L2*s12,  -L2*s12],
        [ L1*c1 + L2*c12,   L2*c12]
    ])
    return J


def jacobian_numerical(theta_1, theta_2, L1=1.0, L2=1.0, h=1e-6):
    """Numerical Jacobian via finite differences for verification."""
    p0 = fk(theta_1, theta_2, L1, L2)
    p1 = fk(theta_1 + h, theta_2, L1, L2)
    p2 = fk(theta_1, theta_2 + h, L1, L2)

    J = np.zeros((2, 2))
    J[:, 0] = (p1 - p0) / h
    J[:, 1] = (p2 - p0) / h
    return J


def manipulability(J):
    """Yoshikawa manipulability measure: sqrt(det(J @ J.T))."""
    return np.sqrt(max(np.linalg.det(J @ J.T), 0.0))


def damped_least_squares(J, x_dot, lam=0.1):
    """Compute joint velocities using DLS for near-singular configurations."""
    n = J.shape[1]
    JJT = J @ J.T
    inv_term = np.linalg.inv(JJT + (lam**2) * np.eye(JJT.shape[0]))
    q_dot = J.T @ inv_term @ x_dot
    return q_dot


# ============================================================
# Test 1: Verify analytical vs numerical Jacobian
# ============================================================

print("=" * 60)
print("Test 1: Analytical vs Numerical Jacobian")
print("=" * 60)

test_configs = [
    (30, 60),
    (45, 90),
    (60, 30),
    (90, 45),
]

for t1_deg, t2_deg in test_configs:
    t1 = np.deg2rad(t1_deg)
    t2 = np.deg2rad(t2_deg)

    J_a = jacobian_analytical(t1, t2)
    J_n = jacobian_numerical(t1, t2)

    diff = np.max(np.abs(J_a - J_n))

    print(f"\nConfig: theta_1 = {t1_deg} deg, theta_2 = {t2_deg} deg")
    print(f"Analytical J:\n{np.round(J_a, 4)}")
    print(f"Max diff vs numerical: {diff:.2e}")
    print(f"Match: {diff < 1e-4}")

# ============================================================
# Test 2: Manipulability across theta_2
# ============================================================

print("\n" + "=" * 60)
print("Test 2: Manipulability vs theta_2")
print("=" * 60)
print("(theta_1 = 45 deg fixed, vary theta_2)")
print(f"{'theta_2 (deg)':<15}{'det(J)':<15}{'Manipulability':<18}{'Status'}")

t1 = np.deg2rad(45)
for t2_deg in [0, 15, 30, 60, 90, 120, 150, 175, 180]:
    t2 = np.deg2rad(t2_deg)
    J = jacobian_analytical(t1, t2)
    det_J = np.linalg.det(J)
    m = manipulability(J)
    status = "SINGULAR" if abs(det_J) < 0.01 else "OK"
    print(f"{t2_deg:<15}{det_J:<15.4f}{m:<18.4f}{status}")

# ============================================================
# Test 3: Damped Least Squares near singularity
# ============================================================

print("\n" + "=" * 60)
print("Test 3: DLS near singularity (theta_2 = 1 deg)")
print("=" * 60)

t1 = np.deg2rad(45)
t2 = np.deg2rad(1)  # Near singular
J = jacobian_analytical(t1, t2)

x_dot_target = np.array([0.5, 0.5])  # Desired end-effector velocity

# Standard inverse (will explode)
try:
    q_dot_inv = np.linalg.solve(J, x_dot_target)
    print(f"Plain inverse q_dot: {np.round(q_dot_inv, 4)}")
    print(f"Magnitude: {np.linalg.norm(q_dot_inv):.4f} (huge!)")
except np.linalg.LinAlgError:
    print("Plain inverse failed (singular)")

# DLS
q_dot_dls = damped_least_squares(J, x_dot_target, lam=0.1)
print(f"\nDLS q_dot (lambda=0.1): {np.round(q_dot_dls, 4)}")
print(f"Magnitude: {np.linalg.norm(q_dot_dls):.4f} (bounded, safe)")
