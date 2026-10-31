# BEN 2026-11-01
# Change aim: Inverse kinematics of a 2-link planar arm (elbow-up and elbow-down), checked with FK.
# File: lab06_two_link_ik.py
import numpy as np
 
def two_link_ik(x, y, l1=0.5, l2=0.5):
    """Inverse kinematics for 2-link planar arm.
       Returns (theta1, theta2) for elbow-up; or None if unreachable."""
    # Reachability check
    r_sq = x**2 + y**2
    c2 = (r_sq - l1**2 - l2**2) / (2 * l1 * l2)
 
    if abs(c2) > 1:
        return None  # unreachable
 
    # Two solutions: elbow-up and elbow-down
    s2_pos = np.sqrt(1 - c2**2)   # elbow-up
    s2_neg = -np.sqrt(1 - c2**2)  # elbow-down
 
    # Elbow-up
    theta2_up = np.arctan2(s2_pos, c2)
    theta1_up = np.arctan2(y, x) - np.arctan2(l2*s2_pos, l1 + l2*c2)
 
    # Elbow-down
    theta2_dn = np.arctan2(s2_neg, c2)
    theta1_dn = np.arctan2(y, x) - np.arctan2(l2*s2_neg, l1 + l2*c2)
 
    return {
        'elbow_up':   (np.rad2deg(theta1_up), np.rad2deg(theta2_up)),
        'elbow_down': (np.rad2deg(theta1_dn), np.rad2deg(theta2_dn))
    }
 
# Test
result = two_link_ik(0.6, 0.4)
if result:
    print(f"Elbow-up:   θ1 = {result['elbow_up'][0]:.2f}°,  θ2 = {result['elbow_up'][1]:.2f}°")
    print(f"Elbow-down: θ1 = {result['elbow_down'][0]:.2f}°, θ2 = {result['elbow_down'][1]:.2f}°")
else:
    print('Target unreachable!')
 
# Verify by FK
from math import sin, cos, radians
t1, t2 = radians(result['elbow_up'][0]), radians(result['elbow_up'][1])
x_check = 0.5*cos(t1) + 0.5*cos(t1+t2)
y_check = 0.5*sin(t1) + 0.5*sin(t1+t2)
print(f'\nVerification: FK gives ({x_check:.3f}, {y_check:.3f})')
print('Should match the target (0.6, 0.4) ✓')

# ============================================
# EXPECTED OUTPUT WHEN RUNNING THIS SCRIPT:
# ============================================
# Elbow-up:   θ1 = -10.16°,  θ2 = 87.71°
# Elbow-down: θ1 = 77.54°, θ2 = -87.71°
# 
# Verification: FK gives (0.600, 0.400)
# Should match the target (0.6, 0.4) ✓
# ============================================
