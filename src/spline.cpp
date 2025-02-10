#include "spline.h"
#include <stdexcept>
#include <algorithm>

Spline::Spline(double tension) {
    setTension(tension);
}

void Spline::setTension(double tension) {
    if (tension < 0.0 || tension > 1.0) {
        throw std::invalid_argument("Tension must be between 0 and 1");
    }
    tension_ = tension;
}

double Spline::distance3D(const std::array<double, 3>& p1, 
                         const std::array<double, 3>& p2) {
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    double dz = p2[2] - p1[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::array<double, 3> Spline::lerp3D(
    const std::array<double, 3>& p1,
    const std::array<double, 3>& p2,
    double t) {
    return {
        p1[0] + (p2[0] - p1[0]) * t,
        p1[1] + (p2[1] - p1[1]) * t,
        p1[2] + (p2[2] - p1[2]) * t
    };
}

std::vector<std::array<double, 3>> Spline::interpolate(
    const std::vector<std::array<double, 3>>& points,
    int base_points_per_segment
) {
    if (points.size() < 2) {
        return points;
    }

    try {
        // Calculate path parameterization
        std::vector<double> t(points.size());
        std::vector<double> x, y, z;
        x.reserve(points.size());
        y.reserve(points.size());
        z.reserve(points.size());

        // Initialize first point
        t[0] = 0.0;
        x.push_back(points[0][0]);
        y.push_back(points[0][1]);
        z.push_back(points[0][2]);

        double total_length = 0.0;
        for (size_t i = 1; i < points.size(); ++i) {
            double segment_length = distance3D(points[i-1], points[i]);
            
            if (segment_length < std::numeric_limits<double>::epsilon()) {
                continue;
            }
            
            total_length += segment_length;
            t[i] = total_length;
            x.push_back(points[i][0]);
            y.push_back(points[i][1]);
            z.push_back(points[i][2]);
        }

        if (total_length <= std::numeric_limits<double>::epsilon()) {
            return points;
        }

        // Normalize parameter to [0,1]
        for (auto& ti : t) {
            ti /= total_length;
        }

        // Compute second derivatives for each dimension
        std::vector<double> y2x = computeSecondDerivatives(t, x);
        std::vector<double> y2y = computeSecondDerivatives(t, y);
        std::vector<double> y2z = computeSecondDerivatives(t, z);

        // Generate interpolated points
        std::vector<std::array<double, 3>> interpolated_points;
        interpolated_points.reserve(points.size() * base_points_per_segment);
        interpolated_points.push_back(points[0]);

        for (size_t i = 0; i < points.size() - 1; ++i) {
            double segment_length = distance3D(points[i], points[i+1]);
            int num_points = std::max(5, 
                static_cast<int>(base_points_per_segment * segment_length / (total_length / points.size()))
            );

            for (int j = 1; j <= num_points; ++j) {
                double ti = t[i] + (t[i+1] - t[i]) * j / num_points;
                
                try {
                    double xi = interpolatePoint(t, x, y2x, ti, i);
                    double yi = interpolatePoint(t, y, y2y, ti, i);
                    double zi = interpolatePoint(t, z, y2z, ti, i);
                    
                    if (std::isfinite(xi) && std::isfinite(yi) && std::isfinite(zi)) {
                        std::array<double, 3> new_point = {xi, yi, zi};
                        
                        if (!isRedundantPoint(interpolated_points.back(), new_point)) {
                            if (interpolated_points.size() < 2 || 
                                isCurvatureAcceptable(
                                    interpolated_points[interpolated_points.size()-2],
                                    interpolated_points.back(),
                                    new_point
                                )) {
                                interpolated_points.push_back(new_point);
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    continue;
                }
            }
        }

        if (!isPathValid(points, interpolated_points)) {
            if (tension_ > 0.5) {
                tension_ *= 0.9;
                return interpolate(points, base_points_per_segment);
            }
            return points;
        }

        return interpolated_points;

    } catch (const std::exception& e) {
        std::cerr << "Error in spline interpolation: " << e.what() << std::endl;
        return points;
    }
}

std::vector<double> Spline::solveTridiagonal(
    const std::vector<double>& a,
    const std::vector<double>& b,
    const std::vector<double>& c,
    const std::vector<double>& d
) {
    if (b.size() != d.size() || (b.size() != a.size() + 1) || (b.size() != c.size() + 1)) {
        throw std::invalid_argument("Invalid matrix dimensions for tridiagonal solver");
    }

    size_t n = b.size();
    std::vector<double> c_prime(n), d_prime(n), x(n);

    // Handle potential division by zero
    if (std::abs(b[0]) < std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Zero diagonal element in tridiagonal solver");
    }

    // Forward sweep
    c_prime[0] = c[0] / b[0];
    d_prime[0] = d[0] / b[0];

    for (size_t i = 1; i < n; ++i) {
        double denominator = b[i] - a[i] * c_prime[i - 1];
        if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) {
            throw std::runtime_error("Zero denominator in tridiagonal solver");
        }
        double m = 1.0 / denominator;
        c_prime[i] = i < n - 1 ? c[i] * m : 0;
        d_prime[i] = (d[i] - a[i] * d_prime[i - 1]) * m;
    }

    // Back substitution
    x[n - 1] = d_prime[n - 1];
    for (int i = n - 2; i >= 0; --i) {
        x[i] = d_prime[i] - c_prime[i] * x[i + 1];
    }

    return x;
}

double Spline::interpolatePoint(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<double>& y2,
    double xi,
    size_t i
) {
    if (i >= x.size() - 1) {
        throw std::out_of_range("Invalid index for interpolation");
    }

    double h = x[i + 1] - x[i];
    if (std::abs(h) < std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Zero interval in interpolation");
    }

    double a = (x[i + 1] - xi) / h;
    double b = (xi - x[i]) / h;
    
    return a * y[i] + b * y[i + 1] + 
           ((a * a * a - a) * y2[i] + (b * b * b - b) * y2[i + 1]) * (h * h) / 6.0;
}

bool Spline::isRedundantPoint(
    const std::array<double, 3>& p1,
    const std::array<double, 3>& p2,
    double threshold
) {
    return distance3D(p1, p2) < threshold;
}

bool Spline::isCurvatureAcceptable(
    const std::array<double, 3>& p1,
    const std::array<double, 3>& p2,
    const std::array<double, 3>& p3,
    double max_angle
) {
    std::array<double, 3> v1 = {
        p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    };
    
    std::array<double, 3> v2 = {
        p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]
    };
    
    double mag1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
    double mag2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]);
    
    if (mag1 < std::numeric_limits<double>::epsilon() || 
        mag2 < std::numeric_limits<double>::epsilon()) {
        return true;
    }
    
    double dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
    double cos_angle = std::min(std::max(dot / (mag1 * mag2), -1.0), 1.0);
    double angle = std::acos(cos_angle);
    
    return angle <= max_angle;
}

bool Spline::isPathValid(
    const std::vector<std::array<double, 3>>& original_path,
    const std::vector<std::array<double, 3>>& smoothed_path,
    double max_deviation
) {
    if (smoothed_path.empty()) return false;
    if (original_path.size() < 2) return true;

    for (const auto& smooth_point : smoothed_path) {
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < original_path.size() - 1; ++i) {
            const auto& p1 = original_path[i];
            const auto& p2 = original_path[i + 1];
            
            std::array<double, 3> v1 = {
                p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
            };
            std::array<double, 3> v2 = {
                smooth_point[0] - p1[0],
                smooth_point[1] - p1[1],
                smooth_point[2] - p1[2]
            };
            
            double len_sq = v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];
            
            if (len_sq < std::numeric_limits<double>::epsilon()) {
                double dist = distance3D(smooth_point, p1);
                min_dist = std::min(min_dist, dist);
                continue;
            }
            
            double t = (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]) / len_sq;
            t = std::min(std::max(t, 0.0), 1.0);
            
            std::array<double, 3> projection = {
                p1[0] + t * v1[0],
                p1[1] + t * v1[1],
                p1[2] + t * v1[2]
            };
            
            double dist = distance3D(smooth_point, projection);
            min_dist = std::min(min_dist, dist);
        }
        
        if (min_dist > max_deviation) {
            return false;
        }
    }
    return true;
}

std::vector<double> Spline::computeSecondDerivatives(
    const std::vector<double>& x,
    const std::vector<double>& y
) {
    size_t n = x.size();
    if (n < 2) {
        throw std::invalid_argument("Need at least two points for spline interpolation");
    }

    // Initialize vectors for the tridiagonal system
    std::vector<double> a(n-1), b(n), c(n-1), d(n);

    // First point
    b[0] = 2.0 * (x[1] - x[0]);
    c[0] = x[1] - x[0];
    d[0] = 6.0 * ((y[1] - y[0]) / (x[1] - x[0]));

    // Internal points
    for (size_t i = 1; i < n - 1; ++i) {
        double hi = x[i + 1] - x[i];
        double hi_1 = x[i] - x[i - 1];
        
        a[i-1] = hi_1;
        b[i] = 2.0 * (hi_1 + hi) * tension_;
        c[i] = hi;
        d[i] = 6.0 * ((y[i + 1] - y[i]) / hi - (y[i] - y[i - 1]) / hi_1);
    }

    // Last point
    a[n-2] = x[n-1] - x[n-2];
    b[n-1] = 2.0 * (x[n-1] - x[n-2]);
    d[n-1] = 6.0 * (-(y[n-1] - y[n-2]) / (x[n-1] - x[n-2]));

    return solveTridiagonal(a, b, c, d);
}