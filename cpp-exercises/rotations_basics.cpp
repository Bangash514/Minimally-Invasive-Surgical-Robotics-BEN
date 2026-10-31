// BEN 2026-11-01
// Change aim: C++ (Eigen) version of lab 04: the same rotation as matrix, Euler angles and quaternion.
// File: rotations_basics.cpp
#include "stewart.hpp"   // for deg2rad, rad2deg and eulerXYZ
#include <iostream>

using namespace gen3;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// BEN 2026-11-01
// Change aim: Build one 90 degree rotation about Z three ways and compare; then show gimbal lock.
// File: rotations_basics.cpp
// Function: main
int main()
{
    std::cout.setf(std::ios::fixed);
    std::cout.precision(4);

    // Method 1: direct rotation matrix
    const double th = deg2rad(90.0);
    Matrix3d R1;
    R1 << std::cos(th), -std::sin(th), 0,
          std::sin(th),  std::cos(th), 0,
          0,             0,            1;

    // Method 2: Euler angles (x, y, z)
    const Matrix3d R2 = eulerXYZ(0, 0, deg2rad(90.0));

    // Method 3: quaternion. NOTE: Eigen's constructor is (w, x, y, z); SciPy uses (x, y, z, w).
    const Eigen::Quaterniond q(std::cos(deg2rad(45.0)), 0, 0, std::sin(deg2rad(45.0)));
    const Matrix3d R3 = q.toRotationMatrix();

    const Vector3d p(1, 0, 0);
    std::cout << "Rotate point (1, 0, 0) by 90 deg about Z\n";
    std::cout << "Method 1 (matrix):     " << (R1 * p).transpose() << "\n";
    std::cout << "Method 2 (Euler):      " << (R2 * p).transpose() << "\n";
    std::cout << "Method 3 (quaternion): " << (R3 * p).transpose() << "\n";
    std::cout << "All three agree: "
              << ((R1 * p).isApprox(R2 * p, 1e-12) && (R2 * p).isApprox(R3 * p, 1e-12) ? "true" : "false") << "\n";

    // Gimbal lock: at pitch = 90 deg, roll and yaw act on the same axis.
    const Matrix3d G1 = eulerXYZ(deg2rad(10), deg2rad(90), 0);
    const Matrix3d G2 = eulerXYZ(0, deg2rad(90), deg2rad(-10));
    std::cout << "\nGimbal lock: Euler (10, 90, 0) and (0, 90, -10) give the same rotation: "
              << (G1.isApprox(G2, 1e-9) ? "true" : "false")
              << "  (max difference " << (G1 - G2).cwiseAbs().maxCoeff() << ")\n";

    // Practice 3.2: decode the quaternion (w, x, y, z) = (0.7071, 0, 0.7071, 0)
    const Eigen::Quaterniond qt(0.7071, 0, 0.7071, 0);
    const Eigen::AngleAxisd aa(qt.normalized());
    std::cout << "\nQuaternion (0.7071, 0, 0.7071, 0): angle " << rad2deg(aa.angle())
              << " deg about axis " << aa.axis().transpose() << "\n";
    return 0;
}
