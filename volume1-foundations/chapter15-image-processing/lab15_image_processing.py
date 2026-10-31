"""
Lab 15: Image Processing for Medical Robotics
Volume 1, Chapter 15

What this lab does:
- Intensity normalization
- Gaussian smoothing
- Mutual information computation
- Simulate MRI-TRUS MI metric surface

Author: BEN (班班)
Date: May 2026
"""

import numpy as np
from scipy.ndimage import gaussian_filter


def normalize_intensity(image, p_low=1, p_high=99):
    low  = np.percentile(image, p_low)
    high = np.percentile(image, p_high)
    return np.clip((image - low) / (high - low), 0, 1)


def smooth_image(image, sigma_mm, spacing_mm):
    sigma_vox = sigma_mm / np.array(spacing_mm)
    return gaussian_filter(image, sigma=sigma_vox)


def mutual_information(A, B, bins=32):
    hist, _, _ = np.histogram2d(
        A.ravel(), B.ravel(), bins=bins
    )
    pab = hist / hist.sum()
    pa  = pab.sum(axis=1)
    pb  = pab.sum(axis=0)
    eps = 1e-12
    HA  = -(pa  * np.log(pa  + eps)).sum()
    HB  = -(pb  * np.log(pb  + eps)).sum()
    HAB = -(pab * np.log(pab + eps)).sum()
    MI  = HA + HB - HAB
    NMI = (HA + HB) / (HAB + eps)
    return MI, NMI


# Generate synthetic MRI-like and TRUS-like images
np.random.seed(42)
size = 64

# Synthetic MRI: bright prostate region in center
mri = np.zeros((size, size))
cx, cy = size//2, size//2
for i in range(size):
    for j in range(size):
        if (i-cx)**2 + (j-cy)**2 < 20**2:
            mri[i,j] = 0.8 + np.random.normal(0, 0.05)
        else:
            mri[i,j] = 0.1 + np.random.normal(0, 0.02)

# Synthetic TRUS: different intensities, same anatomy
trus_aligned = np.zeros((size, size))
for i in range(size):
    for j in range(size):
        if (i-cx)**2 + (j-cy)**2 < 20**2:
            trus_aligned[i,j] = 0.3 + \
                np.random.normal(0, 0.1)  # different scale
        else:
            trus_aligned[i,j] = 0.7 + \
                np.random.normal(0, 0.05)

print("=" * 55)
print("Lab 15: Image Processing")
print("=" * 55)

# Test normalization
mri_norm  = normalize_intensity(mri)
trus_norm = normalize_intensity(trus_aligned)
print(f"\nAfter normalization:")
print(f"  MRI  range: [{mri_norm.min():.3f}, "
      f"{mri_norm.max():.3f}]")
print(f"  TRUS range: [{trus_norm.min():.3f}, "
      f"{trus_norm.max():.3f}]")

# MI at aligned position
MI_aligned, NMI_aligned = mutual_information(
    mri_norm, trus_norm
)
print(f"\nAligned images:")
print(f"  MI  = {MI_aligned:.4f}")
print(f"  NMI = {NMI_aligned:.4f}")

# MI after misalignment (shift by 10 pixels)
trus_shifted = np.roll(trus_aligned, 10, axis=0)
MI_shifted, NMI_shifted = mutual_information(
    mri_norm, normalize_intensity(trus_shifted)
)
print(f"\nMisaligned images (10 pixel shift):")
print(f"  MI  = {MI_shifted:.4f}")
print(f"  NMI = {NMI_shifted:.4f}")
print(f"\nNMI higher when aligned: "
      f"{NMI_aligned > NMI_shifted}")

# Smoothing demo
mri_smooth = smooth_image(mri_norm, sigma_mm=2.0,
                          spacing_mm=[1.0, 1.0])
print(f"\nAfter Gaussian smoothing (sigma=2mm):")
print(f"  Original std: {mri_norm.std():.4f}")
print(f"  Smoothed std: {mri_smooth.std():.4f}")
