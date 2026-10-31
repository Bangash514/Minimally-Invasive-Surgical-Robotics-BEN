// BEN 2026-11-01
// Change aim: C++ (Eigen) version of lab 12: workspace sampling and a singularity monitor.
// File: workspace_singularities.cpp
#include "stewart.hpp"
#include <iostream>
#include <string>

using namespace gen3;
using Eigen::Matrix3d;
using Eigen::Vector3d;

struct MonitorResult {
    std::string status;
    double condition;
    double manipulability;
    int rank;
};

// BEN 2026-11-01
// Change aim: Classify a pose from the Jacobian condition number (warning 50, danger 200, stop 1000).
// File: workspace_singularities.cpp
// Function: checkPose
static MonitorResult checkPose(const Stewart& sp, const Vector3d& p, const Matrix3d& R)
{
    const Matrix6d J = sp.jacobian(p, R);
    int rank;
    double cond;
    rankAndCond(J, rank, cond);
    const double manip = std::sqrt(std::max((J * J.transpose()).determinant(), 0.0));
    std::string status = "SAFE";
    if (cond > 1000) status = "EMERGENCY_STOP";
    else if (cond > 200) status = "DANGER";
    else if (cond > 50) status = "WARNING";
    return {status, cond, manip, rank};
}

// BEN 2026-11-01
// Change aim: Count dexterous, singular (cond > 100) and unreachable poses on a 10x10 XZ grid.
// File: workspace_singularities.cpp
// Function: main
int main()
{
    Stewart sp;
    makeLayout(sp.a, sp.b);
    sp.legMin = 0.20;
    sp.legMax = 0.35;

    const int n = 10;
    int dexterous = 0, singular = 0, unreachable = 0;
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < n; ++k) {
            const double x = -0.04 + 0.08 * i / (n - 1);
            const double z = 0.21 + 0.13 * k / (n - 1);
            bool valid = false;
            sp.legLengths(Vector3d(x, 0, z), Matrix3d::Identity(), valid);
            if (!valid) { ++unreachable; continue; }
            MonitorResult r = checkPose(sp, Vector3d(x, 0, z), Matrix3d::Identity());
            if (r.condition > 100) ++singular; else ++dexterous;
        }
    std::cout << "Workspace sampling (10x10 grid, XZ plane): dexterous " << dexterous
              << ", singular " << singular << ", unreachable " << unreachable << "\n\n";

    std::cout.setf(std::ios::fixed);
    std::cout.precision(4);
    std::cout << "Singularity monitor, paired 3-3 layout:\n";
    const double poses[3][3] = {{0, 0, 0.25}, {0, 0, 0.27}, {0.04, 0, 0.25}};
    const char* names[3] = {"Home", "Up 2cm", "Far X"};
    for (int i = 0; i < 3; ++i) {
        const MonitorResult r = checkPose(sp, Vector3d(poses[i][0], poses[i][1], poses[i][2]), Matrix3d::Identity());
        std::cout << "  " << names[i] << ": " << r.status << " (cond = " << r.condition
                  << ", manip = " << r.manipulability << ", rank " << r.rank << " of 6)\n";
    }

    Stewart sym;
    sym.a = makeHexagonPoints(0.15, 0);
    sym.b = makeHexagonPoints(0.10, 15);
    sym.legMin = 0.20;
    sym.legMax = 0.35;
    const MonitorResult rs = checkPose(sym, Vector3d(0, 0, 0.25), Matrix3d::Identity());
    std::cout << "\nOld 6-fold symmetric layout, home: " << rs.status << " (rank " << rs.rank << " of 6)\n";
    return 0;
}
