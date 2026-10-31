# BEN 2026-11-01
# Change aim: Make the phantom simulation parameter match the TRE it produces.
# File: lab18_validation_metrics.py
"""
Lab 18: Clinical Validation and Accuracy Metrics
Volume 1, Chapter 18

What this lab does:
- Target Registration Error (TRE) computation
- Phantom validation simulation
- Statistical analysis of accuracy
- Pass/fail assessment against clinical thresholds

Author: BEN (班班)
Date: May 2026 (revised 2026-11-01: sigma-based simulation)
"""

import numpy as np


def compute_tre(planned, actual):
    errors = np.linalg.norm(
        planned - actual, axis=1
    )
    return {
        'mean_mm':  np.mean(errors),
        'std_mm':   np.std(errors),
        'max_mm':   np.max(errors),
        'p95_mm':   np.percentile(errors, 95),
        'n':        len(errors),
        'pass_2mm': np.mean(errors) < 2.0,
        'pass_2_5': np.mean(errors) < 2.5
    }


# BEN 2026-11-01
# Change aim: Simulate phantom validation where the error is Gaussian per axis with a given sigma.
# File: lab18_validation_metrics.py
# Function: phantom_validation_simulation
def phantom_validation_simulation(
        n_targets=20, n_reps=5,
        sigma_mm=0.6):
    """
    Simulate phantom validation study.
    Each measurement error is Gaussian with standard deviation sigma_mm on each axis,
    so the mean 3D error (TRE) is about 1.6 * sigma_mm.
    Returns TRE statistics across all measurements.
    """
    rng = np.random.default_rng(42)
    errors = np.linalg.norm(
        rng.normal(0, sigma_mm, (n_targets * n_reps, 3)), axis=1)
    return {
        'mean_mm':   np.mean(errors),
        'std_mm':    np.std(errors),
        'max_mm':    np.max(errors),
        'p95_mm':    np.percentile(errors, 95),
        'n':         len(errors),
        'pass_2mm':  np.mean(errors) < 2.0,
        'pass_2_5':  np.mean(errors) < 2.5
    }


print("=" * 55)
print("Lab 18: Clinical Validation Metrics")
print("=" * 55)

# TRE with known ground truth
np.random.seed(42)
n_targets = 15
planned = np.random.uniform(-25, 25, (n_targets, 3))
noise   = np.random.normal(0, 1.5, (n_targets, 3))
actual  = planned + noise

tre = compute_tre(planned, actual)
print(f"\nTRE Analysis ({n_targets} targets):")
print(f"  Mean TRE:  {tre['mean_mm']:.2f} mm")
print(f"  Std TRE:   {tre['std_mm']:.2f} mm")
print(f"  Max TRE:   {tre['max_mm']:.2f} mm")
print(f"  P95 TRE:   {tre['p95_mm']:.2f} mm")
print(f"  Pass <2mm: {tre['pass_2mm']}")
print(f"  Pass <2.5: {tre['pass_2_5']}")

# Phantom validation simulation
print("\nPhantom Validation Simulation:")
print("  (20 targets × 5 repetitions = 100 measurements)")

good = phantom_validation_simulation(sigma_mm=0.6)
poor = phantom_validation_simulation(sigma_mm=1.6)

print(f"\n  {'Metric':<15} {'Good System':>12} "
      f"{'Poor System':>12}")
print(f"  {'-'*39}")
for key in ['mean_mm', 'std_mm', 'max_mm',
            'p95_mm']:
    label = key.replace('_mm', ' (mm)')
    print(f"  {label:<15} {good[key]:>12.2f} "
          f"{poor[key]:>12.2f}")
print(f"  {'Mean <2mm':<15} "
      f"{str(good['pass_2mm']):>12} "
      f"{str(poor['pass_2mm']):>12}")
print(f"  {'Mean <2.5mm':<15} "
      f"{str(good['pass_2_5']):>12} "
      f"{str(poor['pass_2_5']):>12}")

# Clinical thresholds reference
print("\nClinical Thresholds Reference:")
thresholds = [
    ("Excellent", "< 2.0 mm"),
    ("Good",      "2.0 - 2.5 mm"),
    ("Acceptable","2.5 - 3.5 mm"),
    ("Poor",      "> 3.5 mm"),
]
for level, threshold in thresholds:
    print(f"  {level:<12}: {threshold}")
