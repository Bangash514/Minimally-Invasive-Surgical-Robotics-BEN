// BEN 2026-11-01
// Change aim: C++ version of lab 18: target registration error (TRE) and a simulated phantom study.
// File: validation_metrics.cpp
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

struct TreStats {
    double mean, stddev, maxv, p95;
    int n;
};

// BEN 2026-11-01
// Change aim: Percentile with linear interpolation (the same rule as numpy.percentile).
// File: validation_metrics.cpp
// Function: percentile
static double percentile(std::vector<double> v, double q)
{
    std::sort(v.begin(), v.end());
    const double idx = q / 100.0 * (v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (idx - lo) * (v[hi] - v[lo]);
}

// BEN 2026-11-01
// Change aim: Summarise a list of error lengths: mean, standard deviation, maximum and 95th percentile.
// File: validation_metrics.cpp
// Function: summarise
static TreStats summarise(const std::vector<double>& e)
{
    double mean = 0;
    for (double x : e) mean += x;
    mean /= e.size();
    double var = 0;
    for (double x : e) var += (x - mean) * (x - mean);
    return {mean, std::sqrt(var / e.size()), *std::max_element(e.begin(), e.end()), percentile(e, 95.0),
            static_cast<int>(e.size())};
}

// BEN 2026-11-01
// Change aim: TRE = distance between planned and actual target positions (mm).
// File: validation_metrics.cpp
// Function: computeTre
static TreStats computeTre(const Eigen::MatrixXd& planned, const Eigen::MatrixXd& actual)
{
    std::vector<double> e;
    for (int i = 0; i < planned.rows(); ++i) e.push_back((planned.row(i) - actual.row(i)).norm());
    return summarise(e);
}

// BEN 2026-11-01
// Change aim: Phantom study: each measurement error is Gaussian with sigma on every axis.
// File: validation_metrics.cpp
// Function: phantomSimulation
static TreStats phantomSimulation(int nTargets, int nReps, double sigmaMm, unsigned seed)
{
    std::mt19937 gen(seed);
    std::normal_distribution<double> nd(0.0, sigmaMm);
    std::vector<double> e;
    for (int i = 0; i < nTargets * nReps; ++i) {
        const Eigen::Vector3d err(nd(gen), nd(gen), nd(gen));
        e.push_back(err.norm());
    }
    return summarise(e);
}

// BEN 2026-11-01
// Change aim: Print one row of statistics with the pass flags for mean TRE below 2 and 2.5 mm.
// File: validation_metrics.cpp
// Function: printStats
static void printStats(const char* label, const TreStats& s)
{
    std::cout << std::left << std::setw(20) << label << std::right << std::setw(8) << s.mean << std::setw(8) << s.stddev
              << std::setw(8) << s.maxv << std::setw(8) << s.p95 << std::setw(10) << (s.mean < 2.0 ? "true" : "false")
              << std::setw(10) << (s.mean < 2.5 ? "true" : "false") << "\n";
}

// BEN 2026-11-01
// Change aim: Run a known-noise TRE check and a good and a poor phantom system.
// File: validation_metrics.cpp
// Function: main
int main()
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::left << std::setw(20) << "Case" << std::right << std::setw(8) << "mean" << std::setw(8) << "std"
              << std::setw(8) << "max" << std::setw(8) << "p95" << std::setw(10) << "mean<2" << std::setw(10) << "mean<2.5" << "\n";

    std::mt19937 gen(42);
    std::uniform_real_distribution<double> ud(-25.0, 25.0);
    std::normal_distribution<double> nd(0.0, 1.5);
    Eigen::MatrixXd planned(15, 3), actual(15, 3);
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 3; ++j) {
            planned(i, j) = ud(gen);
            actual(i, j) = planned(i, j) + nd(gen);
        }
    printStats("TRE (15 targets)", computeTre(planned, actual));
    printStats("Good system", phantomSimulation(20, 5, 0.6, 42));
    printStats("Poor system", phantomSimulation(20, 5, 1.6, 42));
    std::cout << "\n(error values in mm; for Gaussian noise the mean TRE is about 1.6 x sigma)\n";
    return 0;
}
