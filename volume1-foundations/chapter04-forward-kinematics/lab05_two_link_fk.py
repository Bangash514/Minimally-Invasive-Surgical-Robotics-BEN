# BEN 2026-11-01
# Change aim: Forward kinematics of a 2-link planar arm (the real FK lab; lab06 is the inverse).
# File: lab05_two_link_fk.py
"""Lab 05: Forward Kinematics of a 2-Link Planar Robot
Volume 1, Chapter 4

What this lab does:
- Compute the elbow and tip position from the two joint angles
- Check the reach limits (max l1 + l2, min |l1 - l2|)
- Reproduce the target (0.6, 0.4) from the joint angles that lab06 (IK) finds

Author: BEN (班班)
Date: 2026-11-01
"""
import numpy as np


# BEN 2026-11-01
# Change aim: Return elbow and tip positions for joint angles given in degrees.
# File: lab05_two_link_fk.py
# Function: two_link_fk
def two_link_fk(theta1_deg, theta2_deg, l1=0.5, l2=0.5):
    t1 = np.deg2rad(theta1_deg)
    t12 = np.deg2rad(theta1_deg + theta2_deg)
    elbow = np.array([l1 * np.cos(t1), l1 * np.sin(t1)])
    tip = elbow + np.array([l2 * np.cos(t12), l2 * np.sin(t12)])
    return elbow, tip


if __name__ == "__main__":
    print("=" * 55)
    print("Lab 05: Two-Link Forward Kinematics")
    print("=" * 55)

    cases = [(0, 0, "Fully stretched"),
             (90, 0, "Straight up"),
             (0, 90, "Elbow bent 90 deg"),
             (0, 180, "Folded back (minimum reach)")]
    for t1, t2, name in cases:
        elbow, tip = two_link_fk(t1, t2)
        print(f"{name:<28} theta=({t1:>3}, {t2:>3})  "
              f"elbow={np.round(elbow, 3)}  tip={np.round(tip, 3)}  "
              f"reach={np.linalg.norm(tip):.3f} m")

    print("\nJoint angles found by lab06 (IK) for the target (0.6, 0.4):")
    for label, (t1, t2) in [("Elbow-up", (-10.16, 87.71)),
                            ("Elbow-down", (77.54, -87.71))]:
        _, tip = two_link_fk(t1, t2)
        print(f"  {label:<10} theta=({t1:>6.2f}, {t2:>6.2f}) -> tip = "
              f"({tip[0]:.3f}, {tip[1]:.3f})")
