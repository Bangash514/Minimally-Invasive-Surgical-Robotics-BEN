# BEN 2026-11-01
# Change aim: Add the header; cubic trajectory with zero start and end velocity.
# File: lab09_cubic_trajectory.py
import numpy as np
import matplotlib.pyplot as plt
 
def cubic_trajectory(q0, qf, T, n_points=100):
    """Generate cubic polynomial trajectory."""
    a0 = q0
    a1 = 0
    a2 = 3 * (qf - q0) / T**2
    a3 = -2 * (qf - q0) / T**3
 
    t = np.linspace(0, T, n_points)
    q = a0 + a1*t + a2*t**2 + a3*t**3
    qd = a1 + 2*a2*t + 3*a3*t**2          # velocity
    qdd = 2*a2 + 6*a3*t                    # acceleration
    return t, q, qd, qdd
 
# Example: 0° to 90° in 2 seconds
t, q, qd, qdd = cubic_trajectory(q0=0, qf=90, T=2.0)
 
# Plot
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
axes[0].plot(t, q); axes[0].set_ylabel('Position (deg)')
axes[0].grid(True); axes[0].set_title('Cubic Polynomial Trajectory')
axes[1].plot(t, qd); axes[1].set_ylabel('Velocity (deg/s)')
axes[1].grid(True)
axes[2].plot(t, qdd); axes[2].set_ylabel('Acceleration (deg/s²)')
axes[2].set_xlabel('Time (s)'); axes[2].grid(True)
plt.tight_layout()
plt.show()
 
# Verify boundary conditions
print('q(0)  =', q[0],  '(should be 0)')
print('q(T)  =', q[-1], '(should be 90)')
print('q\'(0) =', qd[0],  '(should be 0)')
print('q\'(T) =', qd[-1], '(should be 0)')

# ============================================
# ACTUAL OUTPUT WHEN RUN (May 9, 2026):
# ============================================
# q(0)  = 0.0 (should be 0)
# q(T)  = 90.0 (should be 90)
# q'(0) = 0.0 (should be 0)
# q'(T) = 0.0 (should be 0)
# ============================================
