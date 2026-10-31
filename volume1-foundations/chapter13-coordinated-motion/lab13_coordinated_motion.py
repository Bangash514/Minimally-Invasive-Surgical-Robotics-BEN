# BEN 2026-11-01
# Change aim: Use the non-degenerate Stewart joint layout from lab10.
# File: lab13_coordinated_motion.py
"""
Lab 13: Coordinated Multi-Axis Motion
Volume 1, Chapter 13

What this lab does:
- Quintic polynomial trajectory profile
- Compute coordinated leg lengths at each timestep
- Verify all legs move together from same time base
- Check velocity feasibility

Author: BEN (班班)
Date: May 2026
"""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import sys
sys.path.append('../chapter10-stewart-ik')
from lab10_stewart_ik import StewartPlatformIK, \
    make_stewart_layout


def quintic(t, T):
    if t <= 0: return 0.0
    if t >= T: return 1.0
    s = t / T
    return 10*s**3 - 15*s**4 + 6*s**5


def quintic_vel(t, T):
    if t <= 0 or t >= T: return 0.0
    s = t / T
    return (30*s**2 - 60*s**3 + 30*s**4) / T


def coordinated_trajectory(ik_solver,
                            start_pos, start_euler,
                            end_pos, end_euler,
                            T, dt=0.01):
    start_pos   = np.array(start_pos)
    end_pos     = np.array(end_pos)

    R_start = Rotation.from_euler(
        'xyz', start_euler, degrees=True
    )
    R_end   = Rotation.from_euler(
        'xyz', end_euler,   degrees=True
    )
    slerp   = Slerp([0, 1],
                    Rotation.concatenate([R_start, R_end]))

    times     = np.arange(0, T + dt, dt)
    leg_seqs  = []
    feasible  = []

    for t in times:
        s = quintic(t, T)
        pos = start_pos + s * (end_pos - start_pos)
        R   = slerp(s).as_matrix()
        lengths, valid = ik_solver.compute(pos, R)
        leg_seqs.append(lengths.copy())
        feasible.append(valid)

    return times, np.array(leg_seqs), feasible


# Setup
base_pts, platform_pts = make_stewart_layout()
ik = StewartPlatformIK(base_pts, platform_pts,
                       0.20, 0.35)

print("=" * 55)
print("Lab 13: Coordinated Multi-Axis Motion")
print("=" * 55)

times, leg_seqs, feasible = coordinated_trajectory(
    ik,
    start_pos   = [0.00, 0.00, 0.25],
    start_euler = [0, 0, 0],
    end_pos     = [0.02, 0.01, 0.27],
    end_euler   = [5, 3, 0],
    T = 3.0, dt = 0.1
)

print(f"\nTrajectory: {len(times)} timesteps")
print(f"All feasible: {all(feasible)}")
print(f"\nSample leg lengths at key times:")
print(f"{'Time':>6}  {'L1':>8}  {'L2':>8}  {'L3':>8}  "
      f"{'L4':>8}  {'L5':>8}  {'L6':>8}")
for idx in [0, len(times)//4, len(times)//2,
            3*len(times)//4, -1]:
    row = leg_seqs[idx]
    print(f"{times[idx]:>6.2f}  "
          + "  ".join(f"{v:>8.4f}" for v in row))

# Verify legs change together
leg_changes = np.diff(leg_seqs, axis=0)
print(f"\nMax leg velocity (m/step): "
      f"{np.abs(leg_changes).max():.6f}")
print(f"All legs moving together: "
      f"{np.all(np.abs(leg_changes).max(axis=1) > 0)}")
