// BEN 2026-11-01
// Change aim: C++ version of lab 14: force threshold safety and a scalar Kalman filter on a noisy force.
// File: force_control.cpp
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>

// BEN 2026-11-01
// Change aim: Moving-average force check with CAUTION, WARNING and STOP levels.
// File: force_control.cpp
// Function: ForceThresholdSafety
class ForceThresholdSafety {
public:
    ForceThresholdSafety(double normal = 5.0, double warning = 10.0, double stop = 15.0)
        : normal_(normal), warning_(warning), stop_(stop) {}

    // BEN 2026-11-01
    // Change aim: Add one force magnitude (N) and return the status of the last 10 values' mean.
    // File: force_control.cpp
    // Function: ForceThresholdSafety::check
    std::string check(double force)
    {
        history_.push_back(std::abs(force));
        if (history_.size() > 10) history_.pop_front();
        avg_ = std::accumulate(history_.begin(), history_.end(), 0.0) / history_.size();
        if (avg_ >= stop_) return "STOP";
        if (avg_ >= warning_) return "WARNING";
        if (avg_ >= normal_) return "CAUTION";
        return "SAFE";
    }
    double average() const { return avg_; }

private:
    double normal_, warning_, stop_;
    std::deque<double> history_;
    double avg_ = 0.0;
};

// BEN 2026-11-01
// Change aim: Scalar Kalman filter (random-walk model) to smooth the force measurement.
// File: force_control.cpp
// Function: SimpleKalmanFilter
class SimpleKalmanFilter {
public:
    SimpleKalmanFilter(double Q = 0.001, double R = 0.5) : Q_(Q), R_(R) {}
    void predict() { P_ += Q_; }
    void update(double z)
    {
        const double K = P_ / (P_ + R_);
        x_ += K * (z - x_);
        P_ *= (1.0 - K);
    }
    double estimate() const { return x_; }

private:
    double Q_, R_;
    double x_ = 0.0, P_ = 1.0;
};

// BEN 2026-11-01
// Change aim: Simulated needle insertion force (N) as a function of depth (mm).
// File: force_control.cpp
// Function: insertionForce
static double insertionForce(double depth)
{
    if (depth < 5) return depth * 0.3;
    if (depth < 8) return 3.0 + (depth - 5) * 2.5;
    if (depth < 10) return 10.5 - (depth - 8) * 4.0;
    return 2.5 + (depth - 10) * 0.2;
}

// BEN 2026-11-01
// Change aim: Simulate an insertion in 0.1 mm steps, filter the noisy force and report the errors.
//              (Q = 0.05: with Q = 0.001 the filter lags the force and is worse than the raw reading.)
// File: force_control.cpp
// Function: main
int main()
{
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 0.5);
    ForceThresholdSafety safety;
    SimpleKalmanFilter kf(0.05, 0.5);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << " Depth    True F    Noisy F    KF Est  Status\n";
    double sumSqRaw = 0.0, sumSqKf = 0.0;
    int n = 0, counts[4] = {0, 0, 0, 0};
    const char* levels[4] = {"SAFE", "CAUTION", "WARNING", "STOP"};
    for (int i = 0; i < 300; ++i) {
        const double depth = 0.1 * i;
        const double trueF = insertionForce(depth);
        const double noisyF = trueF + noise(gen);
        kf.predict();
        kf.update(noisyF);
        const std::string status = safety.check(noisyF);
        sumSqRaw += (noisyF - trueF) * (noisyF - trueF);
        sumSqKf += (kf.estimate() - trueF) * (kf.estimate() - trueF);
        ++n;
        for (int k = 0; k < 4; ++k)
            if (status == levels[k]) ++counts[k];
        if (i % 30 == 0)
            std::cout << std::setw(5) << depth << "mm " << std::setw(7) << trueF << "N " << std::setw(8) << noisyF
                      << "N " << std::setw(7) << kf.estimate() << "N  " << status << "\n";
    }
    std::cout << "\nRMS error over " << n << " samples: raw " << std::sqrt(sumSqRaw / n) << " N, Kalman "
              << std::sqrt(sumSqKf / n) << " N\n";
    std::cout << "Status counts: SAFE " << counts[0] << ", CAUTION " << counts[1] << ", WARNING " << counts[2]
              << ", STOP " << counts[3] << "\n";
    return 0;
}
