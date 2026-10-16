#pragma once

#include "terrain_planner/planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace terrain_planner::detail {

inline constexpr double epsilon = 1e-9;
inline constexpr double pi = 3.14159265358979323846;
inline double xy_distance(const Point& a, const Point& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}
inline double distance(const Point& a, const Point& b) {
    return std::hypot(xy_distance(a, b), a[2] - b[2]);
}
inline double clamp(double x, double lo, double hi) {
    return std::clamp(x, lo, hi);
}
inline bool finite(const Point& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}
inline int cell(double value, double resolution) {
    const double key = std::floor(value / resolution);
    if (!std::isfinite(key) || key < std::numeric_limits<int>::min() + 16.0 ||
        key > std::numeric_limits<int>::max() - 16.0)
        throw std::invalid_argument("Coordinate is outside the supported grid range");
    return static_cast<int>(key);
}
inline double interpolate(const std::vector<double>& s, const std::vector<double>& v, double q) {
    if (q <= s.front())
        return v.front();
    if (q >= s.back())
        return v.back();
    const auto hi = static_cast<std::size_t>(std::upper_bound(s.begin(), s.end(), q) - s.begin());
    const double t = (q - s[hi - 1]) / (s[hi] - s[hi - 1]);
    return v[hi - 1] * (1 - t) + v[hi] * t;
}
inline std::vector<double> linspace(double total, std::size_t count) {
    std::vector<double> s(count);
    for (std::size_t i = 0; i < count; ++i)
        s[i] = total * static_cast<double>(i) / static_cast<double>(count - 1);
    return s;
}
Path interpolate_path(const Path& path, const std::vector<double>& source_s,
                      const std::vector<double>& query);

} // namespace terrain_planner::detail
