// BEN 2026-11-01
// Change aim: C++ (Eigen) version of labs 02-03: compose a three-frame transform chain.
// File: transforms_chain.cpp
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

static const double kPi = 3.14159265358979323846;

// BEN 2026-11-01
// Change aim: Rotation matrix about the Z axis for an angle in degrees.
// File: transforms_chain.cpp
// Function: rotZ
static Matrix3d rotZ(double deg)
{
    return Eigen::AngleAxisd(deg * kPi / 180.0, Vector3d::UnitZ()).toRotationMatrix();
}

// BEN 2026-11-01
// Change aim: Build a 4x4 homogeneous transform from a rotation and a translation.
// File: transforms_chain.cpp
// Function: makeTransform
static Matrix4d makeTransform(const Matrix3d& R, const Vector3d& t)
{
    Matrix4d T = Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = R;
    T.topRightCorner<3, 1>() = t;
    return T;
}

// BEN 2026-11-01
// Change aim: Compose world -> robot -> end-effector -> probe and check the result.
// File: transforms_chain.cpp
// Function: main
int main()
{
    std::cout.setf(std::ios::fixed);
    std::cout.precision(4);

    const Matrix4d T_world_robot = makeTransform(rotZ(0), Vector3d(0, 0, 0));
    const Matrix4d T_robot_ee = makeTransform(rotZ(45), Vector3d(0.5, 0, 0.3));
    const Matrix4d T_ee_probe = makeTransform(rotZ(0), Vector3d(0, 0, 0.1));

    const Matrix4d T_world_probe = T_world_robot * T_robot_ee * T_ee_probe;
    std::cout << "T_world_probe:\n" << T_world_probe << "\n\n";
    std::cout << "Probe origin in world frame: "
              << T_world_probe.topRightCorner<3, 1>().transpose() << " m\n";

    // a point 5 cm further along the probe z axis, in the probe frame
    const Eigen::Vector4d tip_probe(0, 0, 0.05, 1.0);
    const Eigen::Vector4d tip_world = T_world_probe * tip_probe;
    std::cout << "Probe tip (5 cm along probe z) in world frame: "
              << tip_world.head<3>().transpose() << " m\n";

    // checks: R^T R = I, det R = 1, T^-1 T = I
    const Matrix3d R = T_world_probe.topLeftCorner<3, 3>();
    std::cout << "\nR^T R = I:      " << ((R.transpose() * R).isIdentity(1e-12) ? "true" : "false") << "\n";
    std::cout << "det(R) = 1:     " << (std::abs(R.determinant() - 1.0) < 1e-12 ? "true" : "false") << "\n";
    std::cout << "T^-1 T = I:     " << ((T_world_probe.inverse() * T_world_probe).isIdentity(1e-12) ? "true" : "false") << "\n";
    return 0;
}
