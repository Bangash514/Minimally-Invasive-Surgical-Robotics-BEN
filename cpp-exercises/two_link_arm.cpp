// BEN 2026-11-01
// Change aim: C++ (Eigen) version of labs 05-07: 2-link arm FK, IK, Jacobian and damped least squares.
// File: two_link_arm.cpp
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

using Eigen::Matrix2d;
using Eigen::Vector2d;

static const double kPi = 3.14159265358979323846;
static inline double deg2rad(double d) { return d * kPi / 180.0; }
static inline double rad2deg(double r) { return r * 180.0 / kPi; }

// BEN 2026-11-01
// Change aim: Forward kinematics: tip position from the two joint angles (radians).
// File: two_link_arm.cpp
// Function: fk
static Vector2d fk(double t1, double t2, double l1, double l2)
{
    return Vector2d(l1 * std::cos(t1) + l2 * std::cos(t1 + t2),
                    l1 * std::sin(t1) + l2 * std::sin(t1 + t2));
}

// BEN 2026-11-01
// Change aim: Inverse kinematics for a target point; returns false when it is out of reach.
// File: two_link_arm.cpp
// Function: ik
static bool ik(double x, double y, double l1, double l2, double sign, double& t1, double& t2)
{
    const double c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    if (std::abs(c2) > 1.0) return false;
    t2 = sign * std::acos(c2);
    t1 = std::atan2(y, x) - std::atan2(l2 * std::sin(t2), l1 + l2 * std::cos(t2));
    return true;
}

// BEN 2026-11-01
// Change aim: Analytical Jacobian of the tip position with respect to the joint angles.
// File: two_link_arm.cpp
// Function: jacobian
static Matrix2d jacobian(double t1, double t2, double l1, double l2)
{
    Matrix2d J;
    J << -l1 * std::sin(t1) - l2 * std::sin(t1 + t2), -l2 * std::sin(t1 + t2),
          l1 * std::cos(t1) + l2 * std::cos(t1 + t2),  l2 * std::cos(t1 + t2);
    return J;
}

// BEN 2026-11-01
// Change aim: Numerical Jacobian by finite differences, to verify the analytical one.
// File: two_link_arm.cpp
// Function: jacobianNumeric
static Matrix2d jacobianNumeric(double t1, double t2, double l1, double l2, double h = 1e-6)
{
    Matrix2d J;
    const Vector2d p0 = fk(t1, t2, l1, l2);
    J.col(0) = (fk(t1 + h, t2, l1, l2) - p0) / h;
    J.col(1) = (fk(t1, t2 + h, l1, l2) - p0) / h;
    return J;
}

// BEN 2026-11-01
// Change aim: Yoshikawa manipulability sqrt(det(J J^T)).
// File: two_link_arm.cpp
// Function: manipulability
static double manipulability(const Matrix2d& J)
{
    return std::sqrt(std::max((J * J.transpose()).determinant(), 0.0));
}

// BEN 2026-11-01
// Change aim: Joint velocities by damped least squares, safe near singular poses.
// File: two_link_arm.cpp
// Function: dampedLeastSquares
static Vector2d dampedLeastSquares(const Matrix2d& J, const Vector2d& xdot, double lambda)
{
    const Matrix2d JJt = J * J.transpose() + lambda * lambda * Matrix2d::Identity();
    return J.transpose() * JJt.inverse() * xdot;
}

// BEN 2026-11-01
// Change aim: Run FK/IK round trip, Jacobian check, manipulability and DLS demo.
// File: two_link_arm.cpp
// Function: main
int main()
{
    std::cout.setf(std::ios::fixed);
    std::cout.precision(4);

    std::cout << "== IK and FK round trip (l1 = l2 = 0.5, target 0.6, 0.4) ==\n";
    const double l = 0.5;
    const double signs[2] = {1.0, -1.0};
    const char* names[2] = {"Elbow-up  ", "Elbow-down"};
    for (int i = 0; i < 2; ++i) {
        double t1, t2;
        if (!ik(0.6, 0.4, l, l, signs[i], t1, t2)) { std::cout << "unreachable\n"; continue; }
        const Vector2d tip = fk(t1, t2, l, l);
        std::cout << names[i] << " theta1 = " << rad2deg(t1) << " deg, theta2 = " << rad2deg(t2)
                  << " deg -> FK tip = (" << tip.x() << ", " << tip.y() << ")\n";
    }

    std::cout << "\n== Analytical vs numerical Jacobian (L1 = L2 = 1) ==\n";
    const int configs[4][2] = {{30, 60}, {45, 90}, {60, 30}, {90, 45}};
    for (int i = 0; i < 4; ++i) {
        const double a = deg2rad(configs[i][0]), b = deg2rad(configs[i][1]);
        const double diff = (jacobian(a, b, 1, 1) - jacobianNumeric(a, b, 1, 1)).cwiseAbs().maxCoeff();
        std::cout << "theta = (" << configs[i][0] << ", " << configs[i][1] << ") max diff = "
                  << std::scientific << diff << std::fixed << "  match: " << (diff < 1e-4 ? "true" : "false") << "\n";
    }

    std::cout << "\n== Manipulability vs theta2 (theta1 = 45 deg) ==\n";
    const int t2s[] = {0, 15, 30, 60, 90, 120, 150, 175, 180};
    for (int t2d : t2s) {
        const Matrix2d J = jacobian(deg2rad(45), deg2rad(t2d), 1, 1);
        std::cout << "theta2 = " << t2d << "  det(J) = " << J.determinant()
                  << "  manipulability = " << manipulability(J) << "\n";
    }

    std::cout << "\n== Damped least squares near a singularity (theta2 = 179 deg) ==\n";
    const Matrix2d Js = jacobian(deg2rad(45), deg2rad(179), 1, 1);
    const Vector2d xdot(0.1, 0.0);
    const Vector2d plain = Js.inverse() * xdot;
    const Vector2d damped = dampedLeastSquares(Js, xdot, 0.1);
    std::cout << "plain inverse |q_dot| = " << plain.norm() << "   DLS |q_dot| = " << damped.norm() << "\n";
    return 0;
}
