// BEN 2026-11-01
// Change aim: C++ version of lab 17: Dice score and Hausdorff distance on synthetic circle masks.
// File: segmentation_metrics.cpp
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

typedef std::vector<std::vector<double> > Mask;
typedef std::vector<std::pair<int, int> > Points;

// BEN 2026-11-01
// Change aim: Draw a filled circle (value v) on a size x size grid.
// File: segmentation_metrics.cpp
// Function: circleMask
static Mask circleMask(int size, int cx, int cy, int radius, double v)
{
    Mask m(size, std::vector<double>(size, 0.0));
    for (int i = 0; i < size; ++i)
        for (int j = 0; j < size; ++j)
            if ((i - cx) * (i - cx) + (j - cy) * (j - cy) < radius * radius) m[i][j] = v;
    return m;
}

// BEN 2026-11-01
// Change aim: Dice = 2 |A and B| / (|A| + |B|), with the prediction thresholded at 0.5.
// File: segmentation_metrics.cpp
// Function: diceScore
static double diceScore(const Mask& truth, const Mask& pred, double threshold = 0.5)
{
    double inter = 0, sumT = 0, sumP = 0;
    for (size_t i = 0; i < truth.size(); ++i)
        for (size_t j = 0; j < truth[i].size(); ++j) {
            const double t = truth[i][j], p = pred[i][j] > threshold ? 1.0 : 0.0;
            inter += t * p;
            sumT += t;
            sumP += p;
        }
    return 2.0 * inter / (sumT + sumP + 1e-8);
}

// BEN 2026-11-01
// Change aim: Collect the pixel coordinates where a mask is above the threshold.
// File: segmentation_metrics.cpp
// Function: pointsOf
static Points pointsOf(const Mask& m, double threshold)
{
    Points pts;
    for (size_t i = 0; i < m.size(); ++i)
        for (size_t j = 0; j < m[i].size(); ++j)
            if (m[i][j] > threshold) pts.push_back(std::make_pair((int)i, (int)j));
    return pts;
}

// BEN 2026-11-01
// Change aim: Largest distance from a point of A to its nearest point of B.
// File: segmentation_metrics.cpp
// Function: directedHausdorff
static double directedHausdorff(const Points& A, const Points& B)
{
    double worst = 0.0;
    for (const auto& a : A) {
        double best = 1e18;
        for (const auto& b : B) {
            const double d = std::hypot(a.first - b.first, a.second - b.second);
            best = std::min(best, d);
        }
        worst = std::max(worst, best);
    }
    return worst;
}

// BEN 2026-11-01
// Change aim: Evaluate five synthetic prediction scenarios against a circle of radius 15.
// File: segmentation_metrics.cpp
// Function: main
int main()
{
    const int size = 64, cx = 32, cy = 32, radius = 15;
    const Mask truth = circleMask(size, cx, cy, radius, 1.0);
    struct Scenario { const char* name; int shift; int sizeChange; };
    const std::vector<Scenario> scenarios = {
        {"Perfect prediction", 0, 0}, {"Small offset (2px)", 2, 0}, {"Larger offset (5px)", 5, 0},
        {"Slightly smaller (-2)", 0, -2}, {"Slightly larger (+2)", 0, 2}};

    std::cout << std::fixed << std::left << std::setw(26) << "Scenario" << std::right << std::setw(8) << "Dice"
              << std::setw(12) << "Hausdorff" << std::setw(12) << "Pass(>0.85)" << "\n";
    for (const Scenario& s : scenarios) {
        const Mask pred = circleMask(size, cx + s.shift, cy, radius + s.sizeChange, 0.9);
        const double dice = diceScore(truth, pred);
        const Points pt = pointsOf(truth, 0.5), pp = pointsOf(pred, 0.5);
        const double hd = std::max(directedHausdorff(pt, pp), directedHausdorff(pp, pt));
        std::cout << std::left << std::setw(26) << s.name << std::right << std::setprecision(4) << std::setw(8) << dice
                  << std::setprecision(2) << std::setw(10) << hd << "px" << std::setw(12)
                  << (dice > 0.85 ? "PASS" : "FAIL") << "\n";
    }
    return 0;
}
