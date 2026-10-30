#include "internal.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <LBFGSB.h>
#include <chrono>
#include <numeric>
#include <set>

namespace terrain_planner {
namespace {
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using Sparse = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;
using namespace detail;

struct Reference {
    const Path& points;
    std::vector<double> s;
    explicit Reference(const Path& path) : points(path), s(arc_length(path)) {}
    Path at(const std::vector<double>& query) const {
        return interpolate_path(points, s, query);
    }
};

struct Terrain {
    std::vector<double> s, lower, upper;
    Terrain(const Path& path, double radius) {
        auto sampled = resample(path, .025);
        s = std::move(sampled.second);
        const auto& p = sampled.first;
        const int half = static_cast<int>(std::ceil(radius / (s[1] - s[0])));
        lower.resize(s.size());
        upper.resize(s.size());
        for (int i = 0; i < static_cast<int>(s.size()); ++i) {
            lower[i] = upper[i] = p[i][2];
            for (int j = std::max(0, i - half);
                 j <= std::min(static_cast<int>(p.size()) - 1, i + half); ++j) {
                lower[i] = std::min(lower[i], p[j][2]);
                upper[i] = std::max(upper[i], p[j][2]);
            }
        }
    }
    bool contains(double z, double query) const {
        return std::isfinite(z) && z >= interpolate(s, lower, query) - .080001 &&
               z <= interpolate(s, upper, query) + .080001;
    }
    void project(const GridMap& grid, Path& p, const Path& expected,
                 const std::vector<double>& query) const {
        for (std::size_t i = 0; i < p.size(); ++i) {
            const auto [z, supported] = grid.height(p[i], expected[i][2]);
            p[i][2] = !supported && xy_distance(p[i], expected[i]) > 1e-8 && contains(z, query[i])
                          ? z
                          : expected[i][2];
        }
    }
};

struct Clearance {
    std::vector<double> s, required;
    double target;
    Clearance(const GridMap& grid, const Path& path, double requested) : target(requested) {
        auto sampled = resample(path, .025);
        s = std::move(sampled.second);
        required.resize(s.size());
        for (std::size_t i = 0; i < s.size(); ++i)
            required[i] = std::max(0.0, std::min(target, grid.field(sampled.first[i]).distance));
        for (std::size_t i = 1; i < s.size(); ++i)
            required[i] = std::min(required[i], required[i - 1] + .25 * (s[i] - s[i - 1]));
        for (std::size_t i = s.size() - 1; i > 0; --i)
            required[i - 1] = std::min(required[i - 1], required[i] + .25 * (s[i] - s[i - 1]));
    }
    double at(double query) const {
        return interpolate(s, required, query);
    }
};

std::pair<std::vector<double>, std::vector<std::pair<double, double>>>
protected_weights(const Reference& ref, const std::vector<double>& s) {
    std::vector<double> weights(s.size(), 1);
    std::vector<std::pair<double, double>> spans;
    for (std::size_t i = 0; i + 1 < ref.points.size(); ++i) {
        const double dz = std::abs(ref.points[i + 1][2] - ref.points[i][2]);
        if (dz <= .150001 &&
            !(xy_distance(ref.points[i], ref.points[i + 1]) > .260001 && dz > .080001))
            continue;
        const double begin = ref.s[i] - .2, end = ref.s[i + 1] + .2;
        spans.emplace_back(std::max(0.0, begin), std::min(ref.s.back(), end));
        for (std::size_t j = 0; j < s.size(); ++j) {
            const double t = clamp(std::max({0.0, begin - s[j], s[j] - end}) / 2, 0, 1);
            weights[j] = std::min(weights[j], t * t * t * (10 - 15 * t + 6 * t * t));
        }
    }
    weights.front() = weights.back() = 0;
    return {weights, spans};
}

Validation check_path(const GridMap& grid, const Path& p, const Reference& reference,
                      const std::vector<double>& s, double radius, const Terrain& terrain,
                      const Clearance* clearance = nullptr) {
    const auto expected = reference.at(s);
    Validation out;
    out.sample_count = p.size();
    out.clearance_target_m = clearance ? clearance->target : 0;
    out.clearance_floor_m = std::numeric_limits<double>::infinity();
    std::vector<std::pair<int, int>> keys;
    std::vector<double> zs;
    for (const auto& point : p) {
        keys.emplace_back(cell(point[0], grid.resolution()), cell(point[1], grid.resolution()));
        zs.push_back(grid.height(point, point[2]).first);
    }
    for (std::size_t i = 0; i < p.size(); ++i) {
        const bool occupied = grid.occupied(p[i]);
        const auto height = grid.height(p[i], p[i][2]);
        bool height_ok = height.second && terrain.contains(p[i][2], s[i]);
        if (i == 0 || i + 1 == p.size())
            height_ok = height_ok && std::abs(p[i][2] - expected[i][2]) < 1e-8;
        const double deviation = xy_distance(p[i], expected[i]);
        const bool deviation_ok = deviation <= radius + 1e-6;
        out.max_deviation_m = std::max(out.max_deviation_m, deviation);
        const double floor = std::max(
            0.0, std::min(clearance ? clearance->at(s[i]) : 0, grid.field(expected[i]).distance));
        out.clearance_floor_m = std::min(out.clearance_floor_m, floor);
        const bool clearance_ok = grid.field(p[i]).distance >= floor - 1e-6;
        bool diagonal_ok = true, transition_ok = true;
        if (i > 0) {
            const int dx = std::abs(keys[i].first - keys[i - 1].first),
                      dy = std::abs(keys[i].second - keys[i - 1].second);
            if (std::max(dx, dy) > 1)
                diagonal_ok = false;
            else if (dx == 1 && dy == 1) {
                for (const auto& key : {std::make_pair(keys[i - 1].first, keys[i].second),
                                        std::make_pair(keys[i].first, keys[i - 1].second)}) {
                    bool bridge = false;
                    for (double z : grid.heights(key.first, key.second))
                        if (std::abs(z - p[i][2]) <= .100001 &&
                            std::abs(z - p[i - 1][2]) <= .100001 && terrain.contains(z, s[i]))
                            bridge = true;
                    diagonal_ok = diagonal_ok && bridge;
                }
            }
            const double dz = std::abs(zs[i] - zs[i - 1]);
            if (dz > .150001 &&
                (std::max(deviation, xy_distance(p[i - 1], expected[i - 1])) > 1e-6 ||
                 dz > .200001))
                transition_ok = false;
            out.sample_spacing_m = std::max(out.sample_spacing_m, xy_distance(p[i - 1], p[i]));
        }
        out.outside_count += !occupied;
        out.height_violation_count += occupied && !height_ok;
        out.diagonal_violation_count += !diagonal_ok;
        out.transition_violation_count += !transition_ok;
        out.deviation_violation_count += !deviation_ok;
        out.clearance_violation_count += !clearance_ok;
        if (!occupied || !height_ok || !diagonal_ok || !transition_ok || !deviation_ok ||
            !clearance_ok)
            out.invalid_indices.push_back(i);
    }
    out.valid = out.invalid_indices.empty();
    return out;
}

Path project_clearance(const GridMap& grid, Path p, const std::vector<double>& required,
                       const std::vector<bool>& movable) {
    for (int iteration = 0; iteration < 4; ++iteration) {
        bool active = false;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const auto field = grid.field(p[i]);
            const double norm2 =
                field.gradient[0] * field.gradient[0] + field.gradient[1] * field.gradient[1];
            if (movable[i] && field.distance < required[i] + .005 && norm2 > 1e-8) {
                const double step =
                    std::min(.1, std::max(0.0, required[i] - field.distance + .005));
                for (int dim = 0; dim < 2; ++dim)
                    p[i][dim] += field.gradient[dim] * step / norm2;
                active = true;
            }
        }
        if (!active)
            break;
    }
    return p;
}

Path straighten(const GridMap& grid, const Reference& reference, const std::vector<double>& s,
                const Options& options, const Terrain& terrain, const Clearance& clearance,
                int& segments) {
    const auto expected = reference.at(s);
    auto out = expected;
    const auto weights = protected_weights(reference, s).first;
    auto baseline = check_path(grid, expected, reference, s, options.max_deviation, terrain);
    const std::set<std::size_t> inherited(baseline.invalid_indices.begin(),
                                          baseline.invalid_indices.end());
    std::vector<std::size_t> boundaries;
    for (std::size_t i = 0; i < s.size(); ++i)
        if (weights[i] < 1e-10 || inherited.count(i))
            boundaries.push_back(i);
    for (std::size_t region = 1; region < boundaries.size(); ++region) {
        const std::size_t begin = boundaries[region - 1], end = boundaries[region];
        if (end - begin < 2)
            continue;
        const auto stride = std::max<std::size_t>(1, std::round(.3 / (s[1] - s[0])));
        std::vector<std::size_t> nodes;
        for (auto i = begin; i < end; i += stride)
            nodes.push_back(i);
        nodes.push_back(end);
        std::size_t cursor = 0;
        while (cursor + 1 < nodes.size()) {
            const auto lo = nodes[cursor];
            auto chosen = cursor + 1;
            for (auto next = nodes.size() - 1; next > cursor + 1; --next) {
                const auto hi = nodes[next];
                Path chord(expected.begin() + lo, expected.begin() + hi + 1),
                    local_expected = chord;
                std::vector<double> local_s(s.begin() + lo, s.begin() + hi + 1);
                bool valid = true;
                double max_move = 0;
                for (std::size_t j = 0; j < chord.size(); ++j) {
                    const double t =
                        (local_s[j] - local_s.front()) / (local_s.back() - local_s.front());
                    for (int dim = 0; dim < 2; ++dim)
                        chord[j][dim] = expected[lo][dim] * (1 - t) + expected[hi][dim] * t;
                    max_move = std::max(max_move, xy_distance(chord[j], local_expected[j]));
                }
                if (max_move > options.max_deviation * .9)
                    continue;
                terrain.project(grid, chord, local_expected, local_s);
                const auto checked =
                    check_path(grid, chord, reference, local_s, options.max_deviation, terrain);
                for (auto index : checked.invalid_indices)
                    if ((index != 0 && index + 1 != chord.size()) || !inherited.count(lo + index))
                        valid = false;
                if (!valid)
                    continue;
                for (std::size_t j = 0; j < chord.size(); ++j)
                    if (grid.field(chord[j]).distance < clearance.at(local_s[j]) - 1e-6)
                        valid = false;
                if (!valid)
                    continue;
                chosen = next;
                std::copy(chord.begin(), chord.end(), out.begin() + lo);
                if (max_move > .001)
                    ++segments;
                break;
            }
            cursor = chosen;
        }
    }
    return out;
}

// The recurrence also differentiates the basis analytically; only four entries per row are nonzero.
double basis_value(const std::vector<double>& knots, int i, int degree, double x, int derivative) {
    if (derivative > degree || i < 0 || i + degree + 1 >= static_cast<int>(knots.size()))
        return 0;
    if (degree == 0)
        return derivative == 0 && knots[i] <= x && x < knots[i + 1] ? 1 : 0;
    const double left = knots[i + degree] - knots[i], right = knots[i + degree + 1] - knots[i + 1];
    if (derivative > 0)
        return (left > 0 ? degree / left * basis_value(knots, i, degree - 1, x, derivative - 1)
                         : 0) -
               (right > 0
                    ? degree / right * basis_value(knots, i + 1, degree - 1, x, derivative - 1)
                    : 0);
    return (left > 0 ? (x - knots[i]) / left * basis_value(knots, i, degree - 1, x, 0) : 0) +
           (right > 0
                ? (knots[i + degree + 1] - x) / right * basis_value(knots, i + 1, degree - 1, x, 0)
                : 0);
}

Sparse design(const std::vector<double>& knots, const std::vector<double>& s, int derivative = 0) {
    const int controls = static_cast<int>(knots.size()) - 4;
    std::vector<Triplet> entries;
    for (int row = 0; row < static_cast<int>(s.size()); ++row) {
        const double x = s[row] >= knots.back() ? std::nextafter(knots.back(), 0.0) : s[row];
        const int span = std::clamp(
            static_cast<int>(std::upper_bound(knots.begin(), knots.end(), x) - knots.begin()) - 1,
            3, controls - 1);
        for (int i = span - 3; i <= span; ++i) {
            const double value = basis_value(knots, i, 3, x, derivative);
            if (value != 0)
                entries.emplace_back(row, i, value);
        }
    }
    Sparse basis(static_cast<int>(s.size()), controls);
    basis.setFromTriplets(entries.begin(), entries.end());
    return basis;
}

Matrix xy_matrix(const Path& p) {
    Matrix out(p.size(), 2);
    for (std::size_t i = 0; i < p.size(); ++i) {
        out(i, 0) = p[i][0];
        out(i, 1) = p[i][1];
    }
    return out;
}

struct SmoothingSpline {
    Matrix values, seconds;
    SmoothingSpline(const std::vector<double>& s, const Matrix& targets,
                    const std::vector<double>& weights, double lambda) {
        const int n = static_cast<int>(s.size());
        std::vector<Triplet> qt, rt, wt;
        for (int j = 0; j < n - 2; ++j) {
            const double left = s[j + 1] - s[j], right = s[j + 2] - s[j + 1];
            qt.emplace_back(j, j, 1 / left);
            qt.emplace_back(j + 1, j, -1 / left - 1 / right);
            qt.emplace_back(j + 2, j, 1 / right);
            rt.emplace_back(j, j, (left + right) / 3);
            if (j + 1 < n - 2) {
                rt.emplace_back(j, j + 1, right / 6);
                rt.emplace_back(j + 1, j, right / 6);
            }
        }
        for (int i = 0; i < n; ++i)
            wt.emplace_back(i, i, (i == 0 || i + 1 == n) ? 0 : 1 / weights[i]);
        Sparse q(n, n - 2), r(n - 2, n - 2), inverse_weight(n, n);
        q.setFromTriplets(qt.begin(), qt.end());
        r.setFromTriplets(rt.begin(), rt.end());
        inverse_weight.setFromTriplets(wt.begin(), wt.end());
        const Sparse system = r + lambda * Sparse(q.transpose() * inverse_weight * q);
        Eigen::SimplicialLDLT<Sparse> solve(system);
        if (solve.info() != Eigen::Success)
            throw std::runtime_error("Smoothing spline factorization failed");
        const Matrix interior = solve.solve(q.transpose() * targets);
        if (solve.info() != Eigen::Success || !interior.allFinite())
            throw std::runtime_error("Smoothing spline solve failed");
        values = targets - lambda * inverse_weight * q * interior;
        seconds = Matrix::Zero(n, 2);
        seconds.middleRows(1, n - 2) = interior;
    }
    double minimum_speed(const std::vector<double>& s) const {
        double result = std::numeric_limits<double>::infinity();
        for (int i = 0; i < values.rows(); ++i) {
            Eigen::RowVector2d derivative;
            if (i + 1 < values.rows()) {
                const double h = s[i + 1] - s[i];
                derivative = (values.row(i + 1) - values.row(i)) / h -
                             h * (2 * seconds.row(i) + seconds.row(i + 1)) / 6;
            } else {
                const double h = s[i] - s[i - 1];
                derivative = (values.row(i) - values.row(i - 1)) / h +
                             h * (seconds.row(i - 1) + 2 * seconds.row(i)) / 6;
            }
            result = std::min(result, derivative.norm());
        }
        return result;
    }
};

void classify(Validation& checked, const Validation& baseline, const Path& p, const Path& ref) {
    std::set<std::size_t> inherited;
    for (auto i : baseline.invalid_indices)
        if (distance(p[i], ref[i]) < 1e-8)
            inherited.insert(i);
    checked.inherited_violation_count = 0;
    for (auto i : checked.invalid_indices)
        checked.inherited_violation_count += inherited.count(i) != 0;
    checked.new_violation_count =
        static_cast<int>(checked.invalid_indices.size()) - checked.inherited_violation_count;
}

std::vector<std::size_t> new_failures(const Validation& checked, const Validation& baseline,
                                      const Path& p, const Path& ref) {
    std::set<std::size_t> inherited;
    for (auto i : baseline.invalid_indices)
        if (distance(p[i], ref[i]) < 1e-8)
            inherited.insert(i);
    std::vector<std::size_t> failures;
    for (auto i : checked.invalid_indices)
        if (!inherited.count(i))
            failures.push_back(i);
    return failures;
}

Path navigation_reference(const Path& ground, const Options& options, bool ground_valid,
                          NavigationInfo& info) {
    const auto sampled = resample(ground, .025);
    const auto& query = sampled.second;
    const double radius = options.elevation_radius;
    const double spacing = query[1] - query[0];
    const int half = static_cast<int>(std::ceil(radius / spacing));
    std::vector<double> kernel, filtered(query.size(), 0);
    double sum = 0;
    for (int j = -half; j <= half; ++j) {
        const double value = std::max(0.0, 1 - std::pow(j * spacing / radius, 2));
        kernel.push_back(value * value * value);
        sum += kernel.back();
    }
    for (int i = 0; i < static_cast<int>(query.size()); ++i)
        for (int j = -half; j <= half; ++j) {
            const int index = std::max(0, std::min(static_cast<int>(query.size()) - 1, i + j));
            filtered[i] += sampled.first[index][2] * kernel[j + half] / sum;
        }
    const auto distance_s = arc_length(ground);
    const double minimum = std::max(.05, options.navigation_height - .15);
    const double maximum = options.navigation_height + .15;
    std::vector<bool> fixed(ground.size(), false);
    fixed.front() = fixed.back() = true;
    Path output = ground;
    bool accepted = false;
    for (int pass = 0; pass < 6; ++pass) {
        std::vector<double> anchors;
        for (std::size_t i = 0; i < ground.size(); ++i)
            if (fixed[i])
                anchors.push_back(distance_s[i]);
        accepted = true;
        for (std::size_t i = 0; i < ground.size(); ++i) {
            const auto found = std::lower_bound(anchors.begin(), anchors.end(), distance_s[i]);
            double nearest = std::numeric_limits<double>::infinity();
            if (found != anchors.end())
                nearest = std::abs(*found - distance_s[i]);
            if (found != anchors.begin())
                nearest = std::min(nearest, std::abs(*std::prev(found) - distance_s[i]));
            const double t = clamp(nearest / radius, 0, 1);
            const double blend = t * t * t * (10 - 15 * t + 6 * t * t);
            output[i][2] = ground[i][2] + options.navigation_height;
            if (!fixed[i])
                output[i][2] += blend * (interpolate(query, filtered, distance_s[i]) - ground[i][2]);
            const double height = output[i][2] - ground[i][2];
            if (height < minimum - 1e-8 || height > maximum + 1e-8) {
                fixed[i] = true;
                accepted = false;
            }
        }
        if (accepted)
            break;
    }
    if (!accepted)
        for (std::size_t i = 0; i < ground.size(); ++i)
            output[i][2] = ground[i][2] + options.navigation_height;
    info.height_m = options.navigation_height;
    info.smoothing_radius_m = radius;
    info.minimum_allowed_height_m = minimum;
    info.maximum_allowed_height_m = maximum;
    info.min_ground_offset_m = std::numeric_limits<double>::infinity();
    info.max_ground_offset_m = -std::numeric_limits<double>::infinity();
    info.height_envelope_valid = true;
    for (std::size_t i = 0; i < ground.size(); ++i) {
        const double height = output[i][2] - ground[i][2];
        info.min_ground_offset_m = std::min(info.min_ground_offset_m, height);
        info.max_ground_offset_m = std::max(info.max_ground_offset_m, height);
        const double adjustment = std::abs(height - options.navigation_height);
        info.smoothed_samples += adjustment > 1e-6;
        info.max_smoothing_adjustment_m = std::max(info.max_smoothing_adjustment_m, adjustment);
        info.height_envelope_valid = info.height_envelope_valid && finite(output[i]) &&
                                    height >= minimum - 1e-8 && height <= maximum + 1e-8;
        if (i > 0)
            info.length3d_m += distance(output[i], output[i - 1]);
    }
    const auto dense = resample(output, .05).first;
    double previous_pitch = 0;
    for (std::size_t i = 1; i < dense.size(); ++i) {
        const double pitch = std::atan2(dense[i][2] - dense[i - 1][2], xy_distance(dense[i], dense[i - 1]));
        if (i > 1)
            info.max_pitch_change_deg = std::max(info.max_pitch_change_deg,
                                               std::abs(pitch - previous_pitch) * 180 / pi);
        previous_pitch = pitch;
    }
    info.ground_valid = ground_valid;
    info.valid = ground_valid && info.height_envelope_valid;
    return output;
}

double percentile(std::vector<double> values, double fraction) {
    if (values.empty())
        return 0;
    std::sort(values.begin(), values.end());
    const double index = fraction * (values.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(index)),
               hi = static_cast<std::size_t>(std::ceil(index));
    return values[lo] * (1 - (index - lo)) + values[hi] * (index - lo);
}

std::vector<double> heading_turns(const Path& path) {
    std::vector<double> angles, turns;
    for (std::size_t i = 1; i < path.size(); ++i)
        angles.push_back(std::atan2(path[i][1] - path[i - 1][1], path[i][0] - path[i - 1][0]));
    for (std::size_t i = 1; i < angles.size(); ++i)
        turns.push_back(std::remainder(angles[i] - angles[i - 1], 2 * pi));
    return turns;
}

} // namespace

Metrics metrics(const GridMap& grid, const Path& path, double target_clearance) {
    Metrics out;
    out.length_m = arc_length(path).back();
    for (std::size_t i = 1; i < path.size(); ++i)
        out.length3d_m += detail::distance(path[i - 1], path[i]);
    const auto coarse = resample(path, .2), dense = resample(path, .05);
    const auto turns = heading_turns(coarse.first), dense_turns = heading_turns(dense.first);
    std::vector<double> curvature;
    double square_sum = 0;
    for (std::size_t i = 0; i < turns.size(); ++i) {
        const double length = .5 * (xy_distance(coarse.first[i], coarse.first[i + 1]) +
                                    xy_distance(coarse.first[i + 1], coarse.first[i + 2]));
        const double k = std::abs(turns[i]) / std::max(length, 1e-6);
        curvature.push_back(k);
        square_sum += k * k;
        out.turning_deg += std::abs(turns[i]) * 180 / pi;
        out.sharp_turns += std::abs(turns[i]) * 180 / pi > 5;
        out.profile_s.push_back(std::round(coarse.second[i + 1] * 1000) / 1000);
        out.profile_curvature.push_back(std::round(k * 100000) / 100000);
    }
    out.curvature_rms = curvature.empty() ? 0 : std::sqrt(square_sum / curvature.size());
    out.curvature_p95 = percentile(curvature, .95);
    for (std::size_t i = 0; i < dense_turns.size(); ++i) {
        const double degrees = std::abs(dense_turns[i]) * 180 / pi;
        out.max_heading_change_deg = std::max(out.max_heading_change_deg, degrees);
        if (degrees > 5) {
            ++out.dense_sharp_turns;
            out.sharp_turn_points.push_back(dense.first[i + 1]);
        }
    }
    std::vector<double> distances;
    int below = 0;
    for (const auto& p : dense.first) {
        const double d = grid.field(p).distance;
        distances.push_back(d);
        below += d < target_clearance;
    }
    out.min_clearance_m = std::max(0.0, *std::min_element(distances.begin(), distances.end()));
    out.clearance_p05_m = std::max(0.0, percentile(distances, .05));
    out.below_clearance_percent = 100.0 * below / distances.size();
    return out;
}

OptimizationResult optimize(const GridMap& grid, const Path& path, Options options) {
    const auto started = std::chrono::steady_clock::now();
    if (path.size() < 2 || path.size() > 20000)
        throw std::invalid_argument("Route must contain 2 to 20000 XYZ points");
    if (!(options.smoothness >= .1 && options.smoothness <= 60 && options.max_deviation >= .05 &&
          options.max_deviation <= 3 && options.clearance >= 0 && options.clearance <= 1 &&
          options.control_spacing >= .4 && options.control_spacing <= 4 &&
          options.navigation_height >= .1 && options.navigation_height <= 1.5 &&
          options.elevation_radius >= .3 && options.elevation_radius <= 3))
        throw std::invalid_argument("Optimization parameters are outside the supported limits");
    const auto uniform = resample(path, .025);
    const Reference reference(path);
    const double total = reference.s.back();
    if (total < .1)
        throw std::invalid_argument("Route length must be between 0.1 and 1000 m");
    const Terrain terrain(path, options.max_deviation);
    const Clearance clearance(grid, path, options.clearance);
    OptimizationResult out;
    out.options = options;
    out.source_route = path;
    out.shortcut_path = straighten(grid, reference, uniform.second, options, terrain, clearance,
                                   out.shortcut_segments);
    out.shortcut_length_saved_m = total - arc_length(out.shortcut_path).back();
    const auto sample_s = linspace(total, std::max<std::size_t>(12, std::ceil(total / .15) + 1));
    const auto original = reference.at(sample_s);
    const Matrix target = xy_matrix(interpolate_path(out.shortcut_path, uniform.second, sample_s));
    const Matrix original_target = xy_matrix(original);
    std::vector<double> sample_clearance;
    for (std::size_t i = 0; i < sample_s.size(); ++i)
        sample_clearance.push_back(std::max(
            0.0, std::min(clearance.at(sample_s[i]) + .02, grid.field(original[i]).distance)));
    const int segments = std::max(4, static_cast<int>(std::ceil(total / options.control_spacing)));
    std::vector<double> knots(4, 0);
    for (int i = 1; i < segments; ++i)
        knots.push_back(total * i / segments);
    knots.insert(knots.end(), 4, total);
    const int n = static_cast<int>(knots.size()) - 4;
    const auto basis = design(knots, sample_s), second = design(knots, sample_s, 2);
    const Sparse normal = basis.transpose() * basis;
    Eigen::SimplicialLDLT<Sparse> least_squares(normal);
    if (least_squares.info() != Eigen::Success)
        throw std::runtime_error("Control-point initialization failed");
    Matrix controls = least_squares.solve(basis.transpose() * target);
    if (!controls.allFinite())
        throw std::runtime_error("Control-point initialization is not finite");
    controls.row(0) = original_target.row(0);
    controls.row(n - 1) = original_target.row(original_target.rows() - 1);
    std::vector<double> control_s(n);
    for (int i = 0; i < n; ++i)
        control_s[i] = (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3;
    const auto control_ref = reference.at(control_s);
    std::vector<bool> fixed(n, false);
    fixed.front() = fixed.back() = true;
    const double tangent_s = std::min(.5, total / 3);
    for (int end = 0; end < 2; ++end) {
        const auto& endpoint = end == 0 ? path.front() : path.back();
        if (grid.field(endpoint).distance >= options.clearance)
            continue;
        const int index = end == 0 ? 1 : n - 2;
        const double query = end == 0 ? tangent_s : total - tangent_s;
        const Point direction = reference.at({query})[0];
        const double norm = xy_distance(direction, endpoint),
                     span = end == 0 ? knots[4] - knots[3]
                                     : knots[knots.size() - 4] - knots[knots.size() - 5];
        if (norm < 1e-9)
            continue;
        Point tangent = endpoint;
        for (int dim = 0; dim < 2; ++dim)
            tangent[dim] += (direction[dim] - endpoint[dim]) / norm * span / 3;
        if (std::abs(tangent[0] - control_ref[index][0]) <= options.max_deviation &&
            std::abs(tangent[1] - control_ref[index][1]) <= options.max_deviation) {
            fixed[index] = true;
            controls(index, 0) = tangent[0];
            controls(index, 1) = tangent[1];
        }
    }
    std::vector<std::pair<int, int>> dofs;
    for (int i = 1; i + 1 < n; ++i)
        if (!fixed[i])
            for (int dim = 0; dim < 2; ++dim)
                dofs.emplace_back(i, dim);
    Vector x(dofs.size()), lower(dofs.size()), upper(dofs.size());
    for (int j = 0; j < x.size(); ++j) {
        const auto [i, dim] = dofs[j];
        lower[j] = control_ref[i][dim] - options.max_deviation;
        upper[j] = control_ref[i][dim] + options.max_deviation;
        x[j] = clamp(controls(i, dim), lower[j], upper[j]);
    }
    auto protection = protected_weights(reference, sample_s);
    out.protected_spans = protection.second;
    auto objective = [&](const Vector& vars, Vector& gradient) {
        Matrix c = controls;
        for (int j = 0; j < vars.size(); ++j)
            c(dofs[j].first, dofs[j].second) = vars[j];
        const Matrix curve = basis * c, bend = second * c;
        Matrix grad = 2 * (curve - target);
        double value = (curve - target).squaredNorm() + options.smoothness * bend.squaredNorm();
        const double length_weight = 1.5 / (sample_s[1] - sample_s[0]);
        for (int i = 0; i < curve.rows(); ++i) {
            const auto field = grid.field({curve(i, 0), curve(i, 1), 0});
            const double gap = std::min(0.0, field.distance - sample_clearance[i]),
                         collision = std::min(0.0, field.distance - .035);
            const Eigen::RowVector2d delta = curve.row(i) - original_target.row(i);
            const double norm = delta.norm(),
                         deviation = std::max(0.0, norm - options.max_deviation * .95),
                         terrain_weight = 1 - protection.first[i];
            value += 5000 * gap * gap + 3000 * collision * collision +
                     1000 * deviation * deviation +
                     150 * delta.squaredNorm() * terrain_weight * terrain_weight;
            for (int dim = 0; dim < 2; ++dim)
                grad(i, dim) += (10000 * gap + 6000 * collision) * field.gradient[dim] +
                                2000 * deviation / std::max(norm, 1e-9) * delta[dim] +
                                300 * delta[dim] * terrain_weight * terrain_weight;
            if (i > 0) {
                const Eigen::RowVector2d direction = curve.row(i) - curve.row(i - 1);
                const double length = direction.norm();
                value += length_weight * length;
                const auto unit = (length_weight / std::max(length, 1e-9)) * direction;
                grad.row(i) += unit;
                grad.row(i - 1) -= unit;
            }
        }
        const Matrix control_grad =
            basis.transpose() * grad + 2 * options.smoothness * (second.transpose() * bend);
        gradient.resize(vars.size());
        for (int j = 0; j < vars.size(); ++j)
            gradient[j] = control_grad(dofs[j].first, dofs[j].second);
        if (!std::isfinite(value) || !gradient.allFinite())
            throw std::runtime_error("Non-finite optimizer objective");
        return value;
    };
    LBFGSpp::LBFGSBParam<double> parameters;
    parameters.max_iterations = 220;
    parameters.max_linesearch = 30;
    parameters.epsilon = 1e-5;
    parameters.epsilon_rel = 0;
    parameters.past = 1;
    parameters.delta = 1e-9;
    if (x.size() > 0) {
        LBFGSpp::LBFGSBSolver<double> solver(parameters);
        double value = 0;
        try {
            out.solver.iterations = solver.minimize(objective, x, value, lower, upper);
            out.solver.converged = out.solver.iterations < parameters.max_iterations ||
                                   solver.final_grad_norm() <= parameters.epsilon;
            out.solver.message =
                out.solver.converged ? "L-BFGS-B converged" : "L-BFGS-B iteration limit";
        } catch (const std::runtime_error& error) {
            out.solver.message = std::string("L-BFGS-B stopped: ") + error.what();
        }
        if (!x.allFinite())
            throw std::runtime_error("L-BFGS-B produced non-finite controls");
        for (int j = 0; j < x.size(); ++j)
            controls(dofs[j].first, dofs[j].second) = clamp(x[j], lower[j], upper[j]);
    } else {
        out.solver.converged = true;
        out.solver.message = "All controls are fixed";
    }

    std::vector<double> s = uniform.second;
    s.insert(s.end(), reference.s.begin(), reference.s.end());
    std::sort(s.begin(), s.end());
    s.erase(std::unique(s.begin(), s.end()), s.end());
    auto ref = reference.at(s);
    const Matrix raw_xy = design(knots, s) * controls;
    Matrix delta = raw_xy - xy_matrix(ref);
    auto weight = protected_weights(reference, s).first;
    std::vector<double> repair(s.size(), 1), required(s.size());
    std::vector<bool> movable(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        weight[i] *= std::min(1.0, options.max_deviation / std::max(delta.row(i).norm(), 1e-9));
        movable[i] = weight[i] > 1e-10;
        required[i] = std::max(0.0, std::min(clearance.at(s[i]), grid.field(ref[i]).distance));
    }
    const auto baseline =
        check_path(grid, ref, reference, s, options.max_deviation, terrain, &clearance);
    Path candidate = ref;
    for (int pass = 0; pass < 9; ++pass) {
        out.solver.repair_passes = pass;
        for (std::size_t i = 0; i < s.size(); ++i)
            for (int dim = 0; dim < 2; ++dim)
                candidate[i][dim] = ref[i][dim] + delta(i, dim) * weight[i] * repair[i];
        candidate = project_clearance(grid, std::move(candidate), required, movable);
        terrain.project(grid, candidate, ref, s);
        candidate.front() = path.front();
        candidate.back() = path.back();
        const auto checked =
            check_path(grid, candidate, reference, s, options.max_deviation, terrain, &clearance);
        const auto failures = new_failures(checked, baseline, candidate, ref);
        if (failures.empty() || pass == 8)
            break;
        std::vector<double> bad_s;
        for (auto i : failures)
            bad_s.push_back(s[i]);
        for (std::size_t i = 0; i < s.size(); ++i) {
            const auto found = std::lower_bound(bad_s.begin(), bad_s.end(), s[i]);
            double nearest = std::numeric_limits<double>::infinity();
            if (found != bad_s.end())
                nearest = std::abs(*found - s[i]);
            if (found != bad_s.begin())
                nearest = std::min(nearest, std::abs(*std::prev(found) - s[i]));
            const double t = clamp((nearest - .15) / 2, 0, 1);
            repair[i] = std::min(repair[i], t * t * t * (10 - 15 * t + 6 * t * t));
        }
    }

    const auto dense_s = linspace(total, std::max<std::size_t>(5, std::ceil(total / .025) + 1));
    out.reference = reference.at(dense_s);
    Matrix targets = xy_matrix(interpolate_path(candidate, s, dense_s));
    std::vector<double> weights(dense_s.size(), 1), floors(dense_s.size());
    const auto protected_dense = protected_weights(reference, dense_s).first;
    std::vector<bool> dense_fixed(dense_s.size(), false), dense_movable(dense_s.size(), true);
    out.baseline_validation = check_path(grid, out.reference, reference, dense_s,
                                         options.max_deviation, terrain, &clearance);
    for (auto i : out.baseline_validation.invalid_indices)
        dense_fixed[i] = true;
    for (std::size_t i = 0; i < dense_s.size(); ++i) {
        dense_fixed[i] = dense_fixed[i] || protected_dense[i] < 1e-10;
        if (dense_fixed[i]) {
            targets(i, 0) = out.reference[i][0];
            targets(i, 1) = out.reference[i][1];
            weights[i] = 1e10;
            dense_movable[i] = false;
        }
        floors[i] = std::max(
            0.0, std::min(clearance.at(dense_s[i]), grid.field(out.reference[i]).distance));
    }
    for (int iteration = 0; iteration < 8; ++iteration) {
        out.solver.refinement_passes = iteration + 1;
        const SmoothingSpline curve(dense_s, targets, weights,
                                    .12 * std::sqrt(options.smoothness / 6));
        out.path = out.reference;
        for (std::size_t i = 0; i < out.path.size(); ++i) {
            out.path[i][0] = curve.values(i, 0);
            out.path[i][1] = curve.values(i, 1);
        }
        terrain.project(grid, out.path, out.reference, dense_s);
        out.path.front() = path.front();
        out.path.back() = path.back();
        out.validation = check_path(grid, out.path, reference, dense_s, options.max_deviation,
                                    terrain, &clearance);
        classify(out.validation, out.baseline_validation, out.path, out.reference);
        out.solver.minimum_speed = curve.minimum_speed(dense_s);
        const auto failures =
            new_failures(out.validation, out.baseline_validation, out.path, out.reference);
        if (failures.empty() || iteration == 7)
            break;
        std::vector<bool> boundary_failure(dense_s.size(), false);
        for (std::size_t i = 0; i < dense_s.size(); ++i)
            boundary_failure[i] = grid.field(out.path[i]).distance < floors[i] - 1e-6;
        const auto projected = project_clearance(grid, out.path, floors, dense_movable);
        for (int i = 0; i < static_cast<int>(dense_s.size()); ++i) {
            bool affected = false;
            for (int j = std::max(0, i - 4);
                 j <= std::min(static_cast<int>(dense_s.size()) - 1, i + 4); ++j)
                affected = affected || boundary_failure[j];
            if (affected && !dense_fixed[i]) {
                targets(i, 0) = projected[i][0];
                targets(i, 1) = projected[i][1];
                weights[i] = std::min(1e8, weights[i] * 10);
            }
        }
        for (auto index : failures) {
            const double norm = xy_distance(out.path[index], out.reference[index]);
            for (auto i = index == 0 ? 0 : index - 1; i < std::min(dense_s.size(), index + 2);
                 ++i) {
                if (norm > options.max_deviation + 1e-6) {
                    const double scale = std::min(
                        1.0, options.max_deviation * .96 /
                                 std::max(xy_distance(out.path[i], out.reference[i]), 1e-9));
                    for (int dim = 0; dim < 2; ++dim)
                        targets(i, dim) = out.reference[i][dim] +
                                          (out.path[i][dim] - out.reference[i][dim]) * scale;
                    weights[i] = std::min(1e8, weights[i] * 10);
                } else if (!boundary_failure[index]) {
                    targets(i, 0) = out.reference[i][0];
                    targets(i, 1) = out.reference[i][1];
                    weights[i] = std::min(1e12, weights[i] * 1000);
                }
            }
        }
    }
    const Matrix final_raw = design(knots, dense_s) * controls;
    out.raw_candidate = out.reference;
    for (std::size_t i = 0; i < out.path.size(); ++i) {
        out.raw_candidate[i][0] = final_raw(i, 0);
        out.raw_candidate[i][1] = final_raw(i, 1);
        if (xy_distance(out.path[i], out.reference[i]) < .001)
            out.retained_indices.push_back(i);
        out.solver.constraint_samples += weights[i] > 1;
        out.solver.height_rematched_samples +=
            std::abs(out.path[i][2] - out.reference[i][2]) > 1e-8;
    }
    out.navigation_path = navigation_reference(out.path, options, out.validation.valid,
                                               out.navigation);
    out.baseline = metrics(grid, path, options.clearance);
    out.optimized = metrics(grid, out.path, options.clearance);
    out.solver.control_points = n;
    out.solver.retained_percent = 100.0 * out.retained_indices.size() / out.path.size();
    out.solver.time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();
    return out;
}

} // namespace terrain_planner
