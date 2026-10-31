// BEN 2026-11-01
// Change aim: C++ (Eigen) version of labs 10-11: Stewart platform IK, Jacobian and damped-Newton FK.
// File: stewart_platform.cpp
#include "stewart.hpp"
#include <iostream>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace gen3;

struct TestPose {
    double x, y, z, rxDeg, ryDeg, rzDeg;
    const char* name;
};

// BEN 2026-11-01
// Change aim: Run IK for several poses, report the Jacobian, then recover each pose with FK.
// File: stewart_platform.cpp
// Function: main
int main()
{
    std::cout.setf(std::ios::fixed);
    std::cout.precision(4);

    Stewart sp;
    makeLayout(sp.a, sp.b);
    sp.legMin = 0.20;
    sp.legMax = 0.35;

    const std::vector<TestPose> poses = {
        {0.00, 0.00, 0.25, 0, 0, 0, "Home"},
        {0.00, 0.00, 0.27, 0, 0, 0, "Up 2cm"},
        {0.02, 0.00, 0.25, 0, 0, 0, "X 2cm"},
        {0.00, 0.00, 0.25, 10, 0, 0, "Tilt X 10deg"},
        {0.01, 0.01, 0.26, 5, 5, 0, "Combined"},
        {0.00, 0.00, 0.40, 0, 0, 0, "Out of range"},
    };

    std::cout << "== Inverse kinematics ==\n";
    for (const TestPose& t : poses) {
        bool valid = false;
        const Matrix3d R = eulerXYZ(deg2rad(t.rxDeg), deg2rad(t.ryDeg), deg2rad(t.rzDeg));
        const Vector6d L = sp.legLengths(Vector3d(t.x, t.y, t.z), R, valid);
        std::cout << t.name << ": legs (m) = " << L.transpose() << "  within limits: "
                  << (valid ? "true" : "false") << "\n";
    }

    int rank;
    double cond;
    rankAndCond(sp.jacobian(Vector3d(0, 0, 0.25), Matrix3d::Identity()), rank, cond);
    std::cout << "\nJacobian at home: rank " << rank << " of 6, condition number " << cond << "\n";

    std::cout << "\n== Forward kinematics (damped Newton, start at home) ==\n";
    for (const TestPose& t : poses) {
        bool valid = false;
        const Matrix3d R = eulerXYZ(deg2rad(t.rxDeg), deg2rad(t.ryDeg), deg2rad(t.rzDeg));
        const Vector6d L = sp.legLengths(Vector3d(t.x, t.y, t.z), R, valid);
        if (!valid) { std::cout << t.name << ": outside workspace\n"; continue; }
        Vector6d x;
        x << 0, 0, 0.25, 0, 0, 0;
        int iters = 0;
        const bool ok = sp.forward(L, x, iters);
        const double posErrMm = 1000.0 * (x.head<3>() - Vector3d(t.x, t.y, t.z)).norm();
        const double angErrDeg = (x.tail<3>().unaryExpr([](double v) { return rad2deg(v); })
                                  - Vector3d(t.rxDeg, t.ryDeg, t.rzDeg)).norm();
        std::cout << t.name << ": converged " << (ok ? "true" : "false") << " (" << iters
                  << " iters)  position error " << posErrMm << " mm  angle error " << angErrDeg << " deg\n";
    }
    return 0;
}
