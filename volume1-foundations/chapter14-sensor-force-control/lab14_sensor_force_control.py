# BEN 2026-11-01
# Change aim: Fix the random seed so the noisy force results are repeatable.
# File: lab14_sensor_force_control.py
"""
Lab 14: Sensor Integration and Force Control
Volume 1, Chapter 14

What this lab does:
- Force threshold safety system
- Impedance controller simulation
- Kalman filter for sensor fusion
- Simulated needle insertion force profile

Author: BEN (班班)
Date: May 2026 (revised 2026-11-01: 0.1 mm sampling, Kalman Q = 0.05)
"""

import numpy as np

np.random.seed(42)   # BEN 2026-11-01: fixed seed so the printed results repeat


class ForceThresholdSafety:
    def __init__(self, normal=5.0,
                 warning=10.0, stop=15.0):
        self.NORMAL  = normal
        self.WARNING = warning
        self.STOP    = stop
        self.history = []

    def check(self, F_measured):
        F_mag = np.linalg.norm(F_measured[:3])
        self.history.append(F_mag)
        if len(self.history) > 10:
            self.history.pop(0)
        F_avg = np.mean(self.history)

        if F_avg >= self.STOP:    return 'STOP',    F_avg
        elif F_avg >= self.WARNING: return 'WARNING', F_avg
        elif F_avg >= self.NORMAL:  return 'CAUTION', F_avg
        else:                       return 'SAFE',    F_avg


class SimpleKalmanFilter:
    def __init__(self, Q=0.001, R=0.5):
        self.Q = Q
        self.R = R
        self.x = 0.0
        self.P = 1.0

    def predict(self):
        self.P = self.P + self.Q

    def update(self, measurement):
        K     = self.P / (self.P + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * self.P

    def get(self):
        return self.x, self.P


# Simulate needle insertion
safety = ForceThresholdSafety()
kf     = SimpleKalmanFilter(Q=0.05)   # BEN 2026-11-01: Q=0.001 lagged the force (see README)

print("=" * 55)
print("Lab 14: Force Control and Sensor Fusion")
print("=" * 55)

# Simulate insertion force profile
def simulated_insertion_force(depth_mm):
    if depth_mm < 5:    return depth_mm * 0.3
    elif depth_mm < 8:  return 3.0 + (depth_mm-5)*2.5
    elif depth_mm < 10: return 10.5 - (depth_mm-8)*4.0
    else:               return 2.5 + (depth_mm-10)*0.2

print("\nSimulated needle insertion:")
print(f"{'Depth':>6}  {'True F':>8}  "
      f"{'Noisy F':>9}  {'KF Est':>8}  Status")

# BEN 2026-11-01
# Change aim: Sample every 0.1 mm (not every 3 mm) so the Kalman filter can follow the force,
#             print every 3 mm, and report RMS errors and the status counts.
raw_sq, kf_sq = [], []
counts = {'SAFE': 0, 'CAUTION': 0, 'WARNING': 0, 'STOP': 0}
for i in range(300):
    depth   = 0.1 * i
    true_F  = simulated_insertion_force(depth)
    noisy_F = true_F + np.random.normal(0, 0.5)
    F_vec   = np.array([noisy_F, 0, 0, 0, 0, 0])

    kf.predict()
    kf.update(noisy_F)
    kf_est, _ = kf.get()

    status, F_avg = safety.check(F_vec)
    counts[status] += 1
    raw_sq.append((noisy_F - true_F) ** 2)
    kf_sq.append((kf_est - true_F) ** 2)
    if i % 30 == 0:
        print(f"{depth:>5.1f}mm  {true_F:>8.2f}N  "
              f"{noisy_F:>9.2f}N  {kf_est:>8.2f}N  "
              f"{status}")

print(f"\nRMS error over 300 samples: raw {np.sqrt(np.mean(raw_sq)):.2f} N, "
      f"Kalman {np.sqrt(np.mean(kf_sq)):.2f} N")
print("Status counts:", counts)
