// BEN 2026-11-01
// Change aim: Shared Stewart platform code (layout, IK, Jacobian, damped-Newton FK) for the C++ labs.
// File: stewart.hpp
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace gen3 {

typedef Eigen::Matrix<double, 6, 3> Points;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// BEN 2026-11-01
// Change aim: Rotation matrix from x-y-z Euler angles (extrinsic: R = Rz Ry Rx), radians.
// File: stewart.hpp
// Function: eulerXYZ
inline Eigen::Matrix3d eulerXYZ(double rx, double ry, double rz)
{
    return (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// BEN 2026-11-01
// Change aim: Six equally spaced points on a circle (the old 6-fold symmetric layout, singular at home).
// File: stewart.hpp
// Function: makeHexagonPoints
inline Points makeHexagonPoints(double radius, double offsetDeg)
{
    Points pts;
    for (int i = 0; i < 6; ++i) {
        const double a = deg2rad(60.0 * i + offsetDeg);
        pts.row(i) << radius * std::cos(a), radius * std::sin(a), 0.0;
    }
    return pts;
}

// BEN 2026-11-01
// Change aim: Six joints as three pairs, each pair at +/- half_angle around 0, 120 and 240 deg.
// File: stewart.hpp
// Function: makePairedPoints
inline Points makePairedPoints(double radius, double halfAngleDeg, double centreOffsetDeg)
{
    Points pts;
    int row = 0;
    for (int k = 0; k < 3; ++k)
        for (int sign = -1; sign <= 1; sign += 2) {
            const double a = deg2rad(centreOffsetDeg + 120.0 * k + sign * halfAngleDeg);
            pts.row(row++) << radius * std::cos(a), radius * std::sin(a), 0.0;
        }
    return pts;
}

// BEN 2026-11-01
// Change aim: Base pairs at 0/120/240 deg, platform pairs at 60/180/300 deg rolled by one joint.
// File: stewart.hpp
// Function: makeLayout
inline void makeLayout(Points& base, Points& platform)
{
    base = makePairedPoints(0.15, 10.0, 0.0);
    const Points p = makePairedPoints(0.10, 10.0, 60.0);
    for (int i = 0; i < 6; ++i) platform.row(i) = p.row((i + 5) % 6);   // roll by +1
}

// BEN 2026-11-01
// Change aim: Rank and condition number of a Jacobian from its singular values.
// File: stewart.hpp
// Function: rankAndCond
inline void rankAndCond(const Matrix6d& J, int& rank, double& cond)
{
    Eigen::JacobiSVD<Matrix6d> svd(J);
    const Vector6d s = svd.singularValues();
    rank = 0;
    for (int i = 0; i < 6; ++i)
        if (s(i) > 1e-9 * s(0)) ++rank;
    cond = (s(5) > 1e-300) ? s(0) / s(5) : INFINITY;
}

struct Stewart {
    Points a;      // base joints
    Points b;      // platform joints (platform frame)
    double legMin, legMax;

    // BEN 2026-11-01
    // Change aim: Inverse kinematics: six leg lengths for a pose; valid if inside the leg limits.
    // File: stewart.hpp
    // Function: Stewart::legLengths
    Vector6d legLengths(const Eigen::Vector3d& p, const Eigen::Matrix3d& R, bool& valid) const
    {
        Vector6d L;
        for (int i = 0; i < 6; ++i)
            L(i) = (p + R * b.row(i).transpose() - a.row(i).transpose()).norm();
        valid = (L.minCoeff() >= legMin) && (L.maxCoeff() <= legMax);
        return L;
    }

    // BEN 2026-11-01
    // Change aim: Jacobian rows [u_i^T, (R b_i x u_i)^T] mapping platform twist to leg speeds.
    // File: stewart.hpp
    // Function: Stewart::jacobian
    Matrix6d jacobian(const Eigen::Vector3d& p, const Eigen::Matrix3d& R) const
    {
        Matrix6d J;
        for (int i = 0; i < 6; ++i) {
            const Eigen::Vector3d r = R * b.row(i).transpose();
            const Eigen::Vector3d u = (p + r - a.row(i).transpose()).normalized();
            J.block<1, 3>(i, 0) = u.transpose();
            J.block<1, 3>(i, 3) = r.cross(u).transpose();
        }
        return J;
    }

    // BEN 2026-11-01
    // Change aim: Residuals: squared leg length of the trial pose minus the measured squared length.
    // File: stewart.hpp
    // Function: Stewart::residual
    Vector6d residual(const Vector6d& x, const Vector6d& L) const
    {
        const Eigen::Vector3d p = x.head<3>();
        const Eigen::Matrix3d R = eulerXYZ(x(3), x(4), x(5));
        Vector6d r;
        for (int i = 0; i < 6; ++i)
            r(i) = (p + R * b.row(i).transpose() - a.row(i).transpose()).squaredNorm() - L(i) * L(i);
        return r;
    }

    // BEN 2026-11-01
    // Change aim: Forward kinematics by damped Newton (Levenberg-Marquardt) from an initial guess.
    // File: stewart.hpp
    // Function: Stewart::forward
    bool forward(const Vector6d& L, Vector6d& x, int& iters, double tol = 1e-10, int maxIter = 100) const
    {
        double lambda = 1e-3;
        Vector6d r = residual(x, L);
        for (iters = 0; iters < maxIter; ++iters) {
            if (r.norm() < tol) return true;
            Matrix6d J;
            const double h = 1e-7;
            for (int j = 0; j < 6; ++j) {
                Vector6d xp = x;
                xp(j) += h;
                J.col(j) = (residual(xp, L) - r) / h;
            }
            const Matrix6d A = J.transpose() * J + lambda * Matrix6d::Identity();
            const Vector6d dx = A.ldlt().solve(-J.transpose() * r);
            const Vector6d xNew = x + dx;
            const Vector6d rNew = residual(xNew, L);
            if (rNew.norm() < r.norm()) {
                x = xNew;
                r = rNew;
                lambda = std::max(lambda / 10.0, 1e-12);
            } else {
                lambda *= 10.0;
                if (lambda > 1e12) break;
            }
        }
        return r.norm() < tol;
    }
};

}  // namespace gen3
