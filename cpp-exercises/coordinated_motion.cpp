// BEN 2026-11-01
// Change aim: C++ (Eigen) version of lab 13: quintic motion with coordinated Stewart leg lengths.
// File: coordinated_motion.cpp
#include "stewart.hpp"
#include <iomanip>
#include <iostream>
#include <vector>

using namespace gen3;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// BEN 2026-11-01
// Change aim: Quintic time scaling s(t): zero velocity and acceleration at both ends.
// File: coordinated_motion.cpp
// Function: quintic
static double quintic(double t, double T)
{
    if (t <= 0) return 0.0;
    if (t >= T) return 1.0;
    const double s = t / T;
    return 10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s;
}

// BEN 2026-11-01
// Change aim: Move position linearly and orientation by slerp; collect the six leg lengths each step.
// File: coordinated_motion.cpp
// Function: main
int main()
{
    Stewart sp;
    makeLayout(sp.a, sp.b);
    sp.legMin = 0.20;
    sp.legMax = 0.35;

    const Vector3d p0(0.00, 0.00, 0.25), p1(0.02, 0.01, 0.27);
    const Eigen::Quaterniond q0(eulerXYZ(0, 0, 0));
    const Eigen::Quaterniond q1(eulerXYZ(deg2rad(5), deg2rad(3), 0));
    const double T = 3.0, dt = 0.1;
    const int n = static_cast<int>(std::ceil((T + dt) / dt));   // same length rule as numpy.arange

    std::vector<double> times;
    std::vector<Vector6d> legs;
    bool allFeasible = true;
    for (int i = 0; i < n; ++i) {
        const double t = i * dt;
        const double s = quintic(t, T);
        const Vector3d p = p0 + s * (p1 - p0);
        const Matrix3d R = q0.slerp(s, q1).toRotationMatrix();
        bool valid = false;
        legs.push_back(sp.legLengths(p, R, valid));
        times.push_back(t);
        allFeasible = allFeasible && valid;
    }

    std::cout << "Trajectory: " << n << " timesteps\nAll feasible: " << (allFeasible ? "true" : "false") << "\n\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Sample leg lengths at key times:\n   Time        L1        L2        L3        L4        L5        L6\n";
    const int idx[5] = {0, n / 4, n / 2, 3 * n / 4, n - 1};
    for (int k = 0; k < 5; ++k) {
        std::cout << std::setw(7) << std::setprecision(2) << times[idx[k]] << std::setprecision(4);
        for (int j = 0; j < 6; ++j) std::cout << std::setw(10) << legs[idx[k]](j);
        std::cout << "\n";
    }

    double maxStep = 0.0;
    bool allMoving = true;
    for (int i = 1; i < n; ++i) {
        const double step = (legs[i] - legs[i - 1]).cwiseAbs().maxCoeff();
        maxStep = std::max(maxStep, step);
        if (step <= 0.0) allMoving = false;
    }
    std::cout << "\nMax leg change (m/step): " << std::setprecision(6) << maxStep
              << "\nAll legs moving together: " << (allMoving ? "true" : "false") << "\n";
    return 0;
}
