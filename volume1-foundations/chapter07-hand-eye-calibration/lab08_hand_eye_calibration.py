# BEN 2026-11-01
# Change aim: Check AX = XB and solve the hand-eye transform X from several robot motions.
# File: lab08_hand_eye_calibration.py
"""Lab 08: Hand-Eye Calibration (AX = XB)
Volume 1, Chapter 7

What this lab does:
- Part 1: build a simulated hand-eye transform X and check that A X = X B (concept)
- Part 2: recover X from several robot motions (Park and Martin, 1994):
    rotation by SVD alignment of the rotation vectors, translation by least squares
- Part 3: repeat with noisy measurements and report the error

All data are simulated. Needs at least two motions with different rotation axes.

Author: BEN (班班)
Date: 2026-11-01
"""
import numpy as np
from scipy.spatial.transform import Rotation


# BEN 2026-11-01
# Change aim: Build a 4x4 homogeneous transform from a rotation matrix and a translation.
# File: lab08_hand_eye_calibration.py
# Function: make_T
def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# BEN 2026-11-01
# Change aim: Solve AX = XB for X from lists of motions A_i (robot) and B_i (camera).
# File: lab08_hand_eye_calibration.py
# Function: solve_hand_eye
def solve_hand_eye(A_list, B_list):
    # rotation: align the rotation vectors, alpha_i = R_X beta_i
    alphas = np.array([Rotation.from_matrix(A[:3, :3]).as_rotvec() for A in A_list])
    betas = np.array([Rotation.from_matrix(B[:3, :3]).as_rotvec() for B in B_list])
    H = betas.T @ alphas
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.linalg.det(Vt.T @ U.T)])
    R_X = Vt.T @ D @ U.T
    # translation: (R_A - I) t_X = R_X t_B - t_A  (stack all motions)
    M = np.vstack([A[:3, :3] - np.eye(3) for A in A_list])
    v = np.concatenate([R_X @ B[:3, 3] - A[:3, 3] for A, B in zip(A_list, B_list)])
    t_X = np.linalg.lstsq(M, v, rcond=None)[0]
    return make_T(R_X, t_X)


# BEN 2026-11-01
# Change aim: Make robot poses and the matching camera poses for a known X.
# File: lab08_hand_eye_calibration.py
# Function: simulate_motions
def simulate_motions(X, n=5, seed=0, rot_noise_deg=0.0, trans_noise_m=0.0):
    rng = np.random.default_rng(seed)
    poses = [make_T(Rotation.from_rotvec(rng.normal(0, 0.6, 3)).as_matrix(),
                    rng.uniform(-0.3, 0.3, 3) + [0.5, 0, 0.3]) for _ in range(n + 1)]
    A_list, B_list = [], []
    for i in range(n):
        A = np.linalg.inv(poses[i]) @ poses[i + 1]            # robot motion
        B = np.linalg.inv(X) @ A @ X                          # camera motion (exact)
        if rot_noise_deg or trans_noise_m:
            dR = Rotation.from_rotvec(np.deg2rad(rng.normal(0, rot_noise_deg, 3))).as_matrix()
            B = B @ make_T(dR, rng.normal(0, trans_noise_m, 3))
        A_list.append(A)
        B_list.append(B)
    return A_list, B_list


# BEN 2026-11-01
# Change aim: Report rotation (deg) and translation (mm) error between two transforms.
# File: lab08_hand_eye_calibration.py
# Function: pose_error
def pose_error(X_est, X_true):
    dR = Rotation.from_matrix(X_est[:3, :3].T @ X_true[:3, :3]).magnitude()
    return np.rad2deg(dR), 1000 * np.linalg.norm(X_est[:3, 3] - X_true[:3, 3])


if __name__ == "__main__":
    print("=" * 55)
    print("Lab 08: Hand-Eye Calibration (AX = XB)")
    print("=" * 55)

    X_true = make_T(Rotation.from_euler("z", 30, degrees=True).as_matrix(),
                    [0.05, 0.02, 0.10])

    # Part 1: concept check
    A_list, B_list = simulate_motions(X_true, n=1, seed=1)
    ok = np.allclose(A_list[0] @ X_true, X_true @ B_list[0])
    print(f"\nPart 1: A X = X B holds for the simulated motion: {ok}")

    # Part 2: solve for X (exact data)
    A_list, B_list = simulate_motions(X_true, n=5, seed=1)
    X_est = solve_hand_eye(A_list, B_list)
    rot_err, trans_err = pose_error(X_est, X_true)
    print("\nPart 2: solve X from 5 exact motions")
    print(np.round(X_est, 4))
    print(f"  rotation error {rot_err:.6f} deg, translation error {trans_err:.6f} mm")

    # Part 3: noisy data
    print("\nPart 3: noisy camera motions (rotation 0.2 deg, translation 1 mm)")
    print(f"  {'motions':>8} {'rot err (deg)':>14} {'trans err (mm)':>15}")
    for n in (3, 5, 10, 20):
        A_list, B_list = simulate_motions(X_true, n=n, seed=2,
                                          rot_noise_deg=0.2, trans_noise_m=0.001)
        r, t = pose_error(solve_hand_eye(A_list, B_list), X_true)
        print(f"  {n:>8} {r:>14.3f} {t:>15.3f}")
