// BEN 2026-11-01
// Change aim: C++ (Eigen) version of lab 08: solve the hand-eye transform X from AX = XB.
// File: hand_eye_calibration.cpp
#include "stewart.hpp"   // for deg2rad, rad2deg
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace gen3;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

// BEN 2026-11-01
// Change aim: Build a 4x4 homogeneous transform from a rotation and a translation.
// File: hand_eye_calibration.cpp
// Function: makeT
static Matrix4d makeT(const Matrix3d& R, const Vector3d& t)
{
    Matrix4d T = Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = R;
    T.topRightCorner<3, 1>() = t;
    return T;
}

// BEN 2026-11-01
// Change aim: Rotation vector (axis * angle) of a rotation matrix.
// File: hand_eye_calibration.cpp
// Function: rotVec
static Vector3d rotVec(const Matrix3d& R)
{
    const Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}

// BEN 2026-11-01
// Change aim: Rotation matrix from a rotation vector.
// File: hand_eye_calibration.cpp
// Function: fromRotVec
static Matrix3d fromRotVec(const Vector3d& v)
{
    const double a = v.norm();
    if (a < 1e-15) return Matrix3d::Identity();
    return Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}

// BEN 2026-11-01
// Change aim: Solve AX = XB (Park and Martin): rotation by SVD alignment, translation by least squares.
// File: hand_eye_calibration.cpp
// Function: solveHandEye
static Matrix4d solveHandEye(const std::vector<Matrix4d>& A, const std::vector<Matrix4d>& B)
{
    Matrix3d H = Matrix3d::Zero();
    for (size_t i = 0; i < A.size(); ++i)
        H += rotVec(B[i].topLeftCorner<3, 3>()) * rotVec(A[i].topLeftCorner<3, 3>()).transpose();
    Eigen::JacobiSVD<Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Matrix3d D = Matrix3d::Identity();
    D(2, 2) = (V * U.transpose()).determinant();
    const Matrix3d RX = V * D * U.transpose();

    Eigen::MatrixXd M(3 * A.size(), 3);
    Eigen::VectorXd v(3 * A.size());
    for (size_t i = 0; i < A.size(); ++i) {
        M.block<3, 3>(3 * i, 0) = A[i].topLeftCorner<3, 3>() - Matrix3d::Identity();
        v.segment<3>(3 * i) = RX * B[i].topRightCorner<3, 1>() - A[i].topRightCorner<3, 1>();
    }
    const Vector3d tX = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(v);
    return makeT(RX, tX);
}

// BEN 2026-11-01
// Change aim: Make robot motions A_i and camera motions B_i = X^-1 A_i X, with optional noise.
// File: hand_eye_calibration.cpp
// Function: simulateMotions
static void simulateMotions(const Matrix4d& X, int n, unsigned seed, double rotNoiseDeg, double transNoiseM,
                            std::vector<Matrix4d>& A, std::vector<Matrix4d>& B)
{
    std::mt19937 gen(seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    std::uniform_real_distribution<double> ud(-0.3, 0.3);
    std::vector<Matrix4d> poses;
    for (int i = 0; i <= n; ++i)
        poses.push_back(makeT(fromRotVec(Vector3d(0.6 * nd(gen), 0.6 * nd(gen), 0.6 * nd(gen))),
                              Vector3d(ud(gen) + 0.5, ud(gen), ud(gen) + 0.3)));
    A.clear();
    B.clear();
    for (int i = 0; i < n; ++i) {
        const Matrix4d a = poses[i].inverse() * poses[i + 1];
        Matrix4d b = X.inverse() * a * X;
        if (rotNoiseDeg > 0 || transNoiseM > 0) {
            const Vector3d rv(deg2rad(rotNoiseDeg) * nd(gen), deg2rad(rotNoiseDeg) * nd(gen), deg2rad(rotNoiseDeg) * nd(gen));
            const Vector3d tv(transNoiseM * nd(gen), transNoiseM * nd(gen), transNoiseM * nd(gen));
            b = b * makeT(fromRotVec(rv), tv);
        }
        A.push_back(a);
        B.push_back(b);
    }
}

// BEN 2026-11-01
// Change aim: Rotation error (deg) and translation error (mm) between an estimate and the truth.
// File: hand_eye_calibration.cpp
// Function: poseError
static void poseError(const Matrix4d& est, const Matrix4d& truth, double& rotDeg, double& transMm)
{
    const Matrix3d dR = est.topLeftCorner<3, 3>().transpose() * truth.topLeftCorner<3, 3>();
    rotDeg = rad2deg(Eigen::AngleAxisd(dR).angle());
    transMm = 1000.0 * (est.topRightCorner<3, 1>() - truth.topRightCorner<3, 1>()).norm();
}

// BEN 2026-11-01
// Change aim: Solve with exact data, then with noisy data and 3, 5, 10 and 20 motions.
// File: hand_eye_calibration.cpp
// Function: main
int main()
{
    std::cout << std::fixed << std::setprecision(4);
    const Matrix4d X = makeT(Eigen::AngleAxisd(deg2rad(30), Vector3d::UnitZ()).toRotationMatrix(),
                             Vector3d(0.05, 0.02, 0.10));

    std::vector<Matrix4d> A, B;
    simulateMotions(X, 1, 1, 0, 0, A, B);
    std::cout << "A X = X B holds for one simulated motion: "
              << ((A[0] * X).isApprox(X * B[0], 1e-9) ? "true" : "false") << "\n\n";

    simulateMotions(X, 5, 1, 0, 0, A, B);
    const Matrix4d Xe = solveHandEye(A, B);
    double r, t;
    poseError(Xe, X, r, t);
    std::cout << "Solve X from 5 exact motions:\n" << Xe << "\n  rotation error " << r
              << " deg, translation error " << t << " mm\n\n";

    std::cout << "Noisy camera motions (rotation 0.2 deg, translation 1 mm)\n"
              << "  motions  rot err (deg)  trans err (mm)\n";
    const int counts[4] = {3, 5, 10, 20};
    for (int n : counts) {
        simulateMotions(X, n, 2, 0.2, 0.001, A, B);
        poseError(solveHandEye(A, B), X, r, t);
        std::cout << std::setw(9) << n << std::setw(15) << r << std::setw(16) << t << "\n";
    }
    return 0;
}
