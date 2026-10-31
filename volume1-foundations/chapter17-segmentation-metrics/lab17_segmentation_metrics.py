# BEN 2026-11-01
# Change aim: Rename and re-describe this lab: it computes segmentation metrics, not a U-Net.
# File: lab17_segmentation_metrics.py
"""
Lab 17: Segmentation Metrics (Dice and Hausdorff)
Volume 1, Chapter 17

What this lab does:
- Dice loss and evaluation metrics
- Dice score and Hausdorff distance computation
- Synthetic segmentation evaluation

Author: BEN (班班)
Date: May 2026
Note: This lab only evaluates synthetic masks. It does not train a U-Net.
"""

import numpy as np


def dice_score(y_true, y_pred, threshold=0.5):
    y_pred_bin   = (y_pred > threshold).astype(float)
    intersection = (y_true * y_pred_bin).sum()
    return (2 * intersection) / \
           (y_true.sum() + y_pred_bin.sum() + 1e-8)


def dice_loss(y_true, y_pred, smooth=1e-6):
    intersection = (y_true * y_pred).sum()
    return 1 - (2*intersection + smooth) / \
               (y_true.sum() + y_pred.sum() + smooth)


def hausdorff_distance(mask_true, mask_pred):
    from scipy.spatial.distance import directed_hausdorff
    pts_true = np.argwhere(mask_true)
    pts_pred = np.argwhere(mask_pred)
    if len(pts_true) == 0 or len(pts_pred) == 0:
        return float('inf')
    d1 = directed_hausdorff(pts_true, pts_pred)[0]
    d2 = directed_hausdorff(pts_pred, pts_true)[0]
    return max(d1, d2)


# Synthetic evaluation
np.random.seed(42)
size = 64
cx, cy = 32, 32
radius = 15

# Ground truth mask
y_true = np.zeros((size, size))
for i in range(size):
    for j in range(size):
        if (i-cx)**2 + (j-cy)**2 < radius**2:
            y_true[i,j] = 1.0

print("=" * 55)
print("Lab 17: Segmentation Metrics")
print("=" * 55)

scenarios = [
    ("Perfect prediction",    0,  0),
    ("Small offset (2px)",    2,  0),
    ("Larger offset (5px)",   5,  0),
    ("Slightly smaller (-2)", 0, -2),
    ("Slightly larger (+2)",  0,  2),
]

print(f"\n{'Scenario':<28}  {'Dice':>6}  "
      f"{'Hausdorff':>10}  {'Pass(>0.85)':>12}")

for name, shift, size_change in scenarios:
    r_new = radius + size_change
    y_pred = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            cx2, cy2 = cx + shift, cy
            if (i-cx2)**2 + (j-cy2)**2 < r_new**2:
                y_pred[i,j] = 0.9

    ds  = dice_score(y_true, y_pred)
    hd  = hausdorff_distance(
        y_true.astype(bool),
        (y_pred > 0.5)
    )
    ok  = "✓ PASS" if ds > 0.85 else "✗ FAIL"
    print(f"{name:<28}  {ds:>6.4f}  "
          f"{hd:>10.2f}px  {ok:>12}")
