#ifndef SPLINE_H
#define SPLINE_H

#include <vector>
#include <array>
#include <cstddef>
#include <cmath>
#include <iostream>

class Spline {
public:
    // Constructor with optional tension parameter
    Spline(double tension = 0.2);

    void setTension(double tension);

    // Main interpolation function for 3D points
    std::vector<std::array<double, 3>> interpolate(
        const std::vector<std::array<double, 3>>& points, 
        int base_points_per_segment = 10
    );

private:
    double tension_;

    // Helper functions
    std::vector<double> solveTridiagonal(
        const std::vector<double>& a,
        const std::vector<double>& b,
        const std::vector<double>& c,
        const std::vector<double>& d
    );

    std::vector<double> computeSecondDerivatives(
        const std::vector<double>& x,
        const std::vector<double>& y
    );

    double interpolatePoint(
        const std::vector<double>& x,
        const std::vector<double>& y,
        const std::vector<double>& y2,
        double xi,
        size_t i
    );

    bool isRedundantPoint(
        const std::array<double, 3>& p1,
        const std::array<double, 3>& p2,
        double threshold = 0.3
    );

    bool isCurvatureAcceptable(
        const std::array<double, 3>& p1,
        const std::array<double, 3>& p2,
        const std::array<double, 3>& p3,
        double max_angle = M_PI/6
    );

    bool isPathValid(
        const std::vector<std::array<double, 3>>& original_path,
        const std::vector<std::array<double, 3>>& smoothed_path,
        double max_deviation = 3.0
    );

    // New helper functions for 3D
    static double distance3D(const std::array<double, 3>& p1, 
                           const std::array<double, 3>& p2);
    
    static std::array<double, 3> lerp3D(
        const std::array<double, 3>& p1,
        const std::array<double, 3>& p2,
        double t
    );
};

#endif // SPLINE_H