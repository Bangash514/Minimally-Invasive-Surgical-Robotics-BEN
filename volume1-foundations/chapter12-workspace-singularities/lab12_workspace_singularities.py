# BEN 2026-11-01
# Change aim: Use the non-degenerate Stewart joint layout from lab10.
# File: lab12_workspace_singularities.py
"""
Lab 12: Workspace Analysis and Singularity Detection
Volume 1, Chapter 12

What this lab does:
- Sample workspace and classify poses
- Compute manipulability across workspace
- Implement singularity monitor
- Map safe vs singular regions

Author: BEN (班班)
Date: May 2026
"""

import numpy as np
from scipy.spatial.transform import Rotation
import sys
sys.path.append('../chapter10-stewart-ik')
from lab10_stewart_ik import StewartPlatformIK, \
    make_stewart_layout


def compute_manipulability(Jp):
    return np.sqrt(max(np.linalg.det(Jp @ Jp.T), 0))


def workspace_analysis(ik_solver, n=10):
    results = {'dexterous': 0, 'singular': 0,
               'unreachable': 0}
    x_vals = np.linspace(-0.04, 0.04, n)
    z_vals = np.linspace(0.21, 0.34, n)

    for x in x_vals:
        for z in z_vals:
            pos = np.array([x, 0, z])
            lengths, reachable = ik_solver.compute(
                pos, np.eye(3)
            )
            if not reachable:
                results['unreachable'] += 1
                continue
            Jp   = ik_solver.compute_jacobian(
                pos, np.eye(3)
            )
            cond = np.linalg.cond(Jp)
            if cond > 100:
                results['singular'] += 1
            else:
                results['dexterous'] += 1

    return results


class SingularityMonitor:
    def __init__(self, ik_solver,
                 warning=50, danger=200, stop=1000):
        self.ik      = ik_solver
        self.WARNING = warning
        self.DANGER  = danger
        self.STOP    = stop

    def check(self, position, rotation):
        Jp    = self.ik.compute_jacobian(
            position, rotation
        )
        cond  = np.linalg.cond(Jp)
        manip = compute_manipulability(Jp)

        if cond > self.STOP:    status = 'EMERGENCY_STOP'
        elif cond > self.DANGER: status = 'DANGER'
        elif cond > self.WARNING: status = 'WARNING'
        else:                    status = 'SAFE'

        return {'status': status,
                'condition': cond,
                'manipulability': manip}


# Setup
base_pts, platform_pts = make_stewart_layout()
ik      = StewartPlatformIK(base_pts, platform_pts,
                             0.20, 0.35)
monitor = SingularityMonitor(ik)

print("=" * 55)
print("Lab 12: Workspace and Singularity Analysis")
print("=" * 55)

# Workspace analysis
print("\nWorkspace sampling (10x10 grid, XZ plane):")
results = workspace_analysis(ik, n=10)
total = sum(results.values())
for k, v in results.items():
    print(f"  {k:12s}: {v:3d}  ({v/total*100:.1f}%)")

# Singularity monitor test
print("\nSingularity monitor test:")
test_positions = [
    ([0.00, 0.00, 0.25], "Home (safe)"),
    ([0.00, 0.00, 0.27], "Up 2cm"),
    ([0.04, 0.00, 0.25], "Far X (still safe)"),
]
for pos, name in test_positions:
    result = monitor.check(pos, np.eye(3))
    print(f"  {name}: {result['status']} "
          f"(cond={result['condition']:.1f}, "
          f"manip={result['manipulability']:.4f})")

# BEN 2026-11-01
# Change aim: Show that the monitor trips on a real singularity (6-fold symmetric joint layout).
# File: lab12_workspace_singularities.py
# Function: (script section)
print("\nOld 6-fold symmetric layout (known singular at home):")
from lab10_stewart_ik import make_hexagonal_points
sym_ik = StewartPlatformIK(make_hexagonal_points(0.15, 0),
                           make_hexagonal_points(0.10, 15), 0.20, 0.35)
sym_result = SingularityMonitor(sym_ik).check([0.00, 0.00, 0.25], np.eye(3))
print(f"  Home: {sym_result['status']} "
      f"(rank {np.linalg.matrix_rank(sym_ik.compute_jacobian([0, 0, 0.25], np.eye(3)))} of 6)")
