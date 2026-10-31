# Minimally Invasive Surgical Robotics (C++ / Eigen)

Simulation workbook with the **fundamentals behind minimally invasive surgical robotics**: coordinate frames, rotations, forward and inverse kinematics, the Jacobian and singularities, a Stewart platform, hand-eye calibration, trajectories, force control, a safety state machine and accuracy metrics.

The main code is **C++ (Eigen)**, the language used in industrial and medical robotics. Each C++ program has a **Python prototype** (17 chapter folders, 19 scripts), and the C++ results were checked against the Python ones.

> Status: simulation and learning code. It uses simulated data only, not a real robot or patient data, and is **not for clinical use**.

## Contents

| Chapter folder | Python lab | C++ program (`cpp-exercises/`) | What it shows |
|---|---|---|---|
| `chapter01-frames`, `chapter02-transforms` | lab01, lab02, lab03, example 2.3 | `transforms_chain` | Frames, homogeneous transforms, inverse, three-frame chain |
| `chapter03-rotations` | lab04 | `rotations_basics` | Matrix, Euler angles, quaternion, gimbal lock |
| `chapter04-forward-kinematics`, `chapter05-inverse-kinematics`, `chapter06-jacobian` | lab05, lab06, lab07 | `two_link_arm` | 2-link arm FK, IK (two solutions), Jacobian, manipulability, damped least squares |
| `chapter07-hand-eye-calibration` | lab08 | `hand_eye_calibration` | AX = XB solved with Park and Martin (1994) |
| `chapter08-trajectory-planning` | lab09 | `trajectory_cubic` | Cubic trajectory, zero start and end velocity |
| `chapter10-stewart-ik`, `chapter11-stewart-fk` | lab10, lab11 | `stewart_platform` | Leg lengths, Jacobian, forward kinematics by damped Newton |
| `chapter12-workspace-singularities` | lab12 | `workspace_singularities` | Workspace sampling, singularity monitor |
| `chapter13-coordinated-motion` | lab13 | `coordinated_motion` | Quintic motion with slerp and coordinated leg lengths |
| `chapter14-sensor-force-control` | lab14 | `force_control` | Force thresholds, Kalman filter on a noisy force |
| `chapter15-image-processing` | lab15 | not ported | Normalization, smoothing, mutual information |
| `chapter16-system-architecture` | lab16 | `state_machine` | Clinical workflow state machine with emergency stop |
| `chapter17-segmentation-metrics` | lab17 | `segmentation_metrics` | Dice and Hausdorff on synthetic masks (no U-Net training) |
| `chapter18-clinical-validation` | lab18 | `validation_metrics` | Target registration error (TRE), phantom study simulation |

`cpp-exercises/include/stewart.hpp` holds the shared Stewart platform code.

## Results
- **Build:** all 12 C++ programs build with CMake and Eigen 3.4 with `-Wall -Wextra` and no warnings. All 19 Python scripts run without errors.
- **C++ against Python (same numbers):** Stewart leg lengths agree to 5e-5 m (display rounding); the coordinated motion gives the same 31 timesteps and the same maximum leg change (0.001871 m); the workspace grid gives 88 dexterous, 0 singular and 12 unreachable poses; Dice and Hausdorff match for all five scenarios (for example Dice 0.9168 and 2.00 px for the 2 px offset).
- **2-link arm:** both IK solutions for the target (0.6, 0.4) give back (0.600, 0.400) through FK. The analytical Jacobian matches finite differences (max difference below 1e-6).
- **Stewart platform:** with a 6-fold symmetric joint layout the home Jacobian has **rank 3** (singular, the monitor reports `EMERGENCY_STOP`). With the paired 3-3 layout it has **rank 6, condition number 25.9**, and forward kinematics recovers all five test poses with 0.0000 mm and 0.0000 deg error.
- **Hand-eye calibration:** exact data gives zero error. With noisy motions (0.2 deg, 1 mm) the error falls as motions are added (3 motions: 0.65 deg and 1.3 mm; 20 motions: 0.08 deg and 0.29 mm in C++).
- **Kalman filter:** with the original settings (3 mm sampling, Q = 0.001) the filter lagged the force and had about 4 times the error of the raw readings. With 0.1 mm sampling and Q = 0.05 the RMS error is 0.39 N, against 0.50 N for the raw noisy readings (C++).
- **Random numbers:** the hand-eye, force and phantom results use random noise, so C++ and Python values differ slightly (different generators); the trends are the same.

## Build and run
```
# C++ (Eigen 3.3 or newer, CMake)
cd cpp-exercises
mkdir build && cd build
cmake .. && make
./stewart_platform

# Python (from a chapter folder; needs numpy and scipy, matplotlib for some plots)
cd volume1-foundations/chapter11-stewart-fk
python lab11_stewart_fk.py
```

## Update 2026-11-01
- Added the C++ (Eigen) programs and the shared `stewart.hpp`.
- Stewart platform: replaced the 6-fold symmetric joint layout (rank 3 at home) with a paired 3-3 layout; `lab10` no longer prints when imported.
- `lab11`: fixed the import and replaced plain Newton with a damped Newton method.
- `lab05` is now a real forward-kinematics lab; the Jacobian lab is `lab07`; `lab08` now solves for X; `lab17` was renamed because it computes metrics, not a U-Net; `lab18` simulation parameter now matches the TRE it produces.
- `lab14`: Kalman filter retuned and samples every 0.1 mm; fixed random seed.

## Limits
- Simulation only: no ROS, no real robot control, no motion planning with obstacles.
- The Stewart platform values are example dimensions, not a specific device.
- `lab17` does not train a segmentation network. `lab15` (mutual information) is not ported to C++.

## Use and contact
If you use, adapt or contribute to this work, please credit the author and **contact or inform me by email: owaiskhan_bangash@yahoo.com**

## Author
BEN, Software Engineer. GitHub: [Bangash514](https://github.com/Bangash514)
Started May 6, 2026. Updated 2026-11-01.
