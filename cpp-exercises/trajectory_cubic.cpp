// BEN 2026-11-01
// Change aim: C++ version of lab 09: cubic polynomial trajectory with zero start and end velocity.
// File: trajectory_cubic.cpp
#include <iostream>
#include <vector>

struct Sample {
    double t, q, qd, qdd;
};

// BEN 2026-11-01
// Change aim: Sample the cubic q(t) = a0 + a2 t^2 + a3 t^3 from q0 to qf in time T.
// File: trajectory_cubic.cpp
// Function: cubicTrajectory
static std::vector<Sample> cubicTrajectory(double q0, double qf, double T, int n)
{
    const double a0 = q0, a1 = 0.0;
    const double a2 = 3.0 * (qf - q0) / (T * T);
    const double a3 = -2.0 * (qf - q0) / (T * T * T);
    std::vector<Sample> out;
    for (int i = 0; i < n; ++i) {
        const double t = T * i / (n - 1);
        out.push_back({t, a0 + a1 * t + a2 * t * t + a3 * t * t * t,
                       a1 + 2 * a2 * t + 3 * a3 * t * t,
                       2 * a2 + 6 * a3 * t});
    }
    return out;
}

// BEN 2026-11-01
// Change aim: Move 0 to 90 degrees in 2 s and check the four boundary conditions.
// File: trajectory_cubic.cpp
// Function: main
int main()
{
    const std::vector<Sample> s = cubicTrajectory(0.0, 90.0, 2.0, 100);
    std::cout << "q(0)   = " << s.front().q << "  (should be 0)\n";
    std::cout << "q(T)   = " << s.back().q << "  (should be 90)\n";
    std::cout << "q'(0)  = " << s.front().qd << "  (should be 0)\n";
    std::cout << "q'(T)  = " << s.back().qd << "  (should be 0)\n";
    std::cout << "peak velocity = " << (3.0 * 90.0 / (2.0 * 2.0)) << " deg/s at t = 1 s (analytic)\n";
    std::cout << "\n   t(s)    q(deg)  qd(deg/s)\n";
    for (int i = 0; i < 100; i += 11)
        std::cout << s[i].t << "  " << s[i].q << "  " << s[i].qd << "\n";
    return 0;
}
