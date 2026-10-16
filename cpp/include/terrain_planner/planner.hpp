#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace terrain_planner {

using Point = std::array<double, 3>;
using Key = std::array<std::int32_t, 3>;
using Path = std::vector<Point>;

struct Options {
    double smoothness = 6.0;
    double max_deviation = 0.8;
    double clearance = 0.39;
    double control_spacing = 1.5;
};

struct FieldSample {
    double distance;
    std::array<double, 2> gradient;
};

class GridMap {
  public:
    explicit GridMap(std::vector<Key> keys, double resolution = 0.1);
    const std::vector<Key>& keys() const {
        return keys_;
    }
    double resolution() const {
        return resolution_;
    }
    const std::vector<double>& heights(int x, int y) const;
    bool occupied(const Point& point) const;
    std::pair<double, bool> height(const Point& point, double expected_z) const;
    FieldSample field(const Point& point) const;

  private:
    std::vector<Key> keys_;
    double resolution_;
    std::map<std::pair<int, int>, std::vector<double>> columns_;
    int origin_x_, origin_y_, width_, height_;
    std::vector<double> distance_;
};

struct SearchOptions {
    double ground_neighbor_radius = 0.16;
    double ground_height_tolerance = 0.04;
    double maximum_slope_deg = 25;
    double minimum_step_height = 0.05;
    double maximum_step_height = 0.20;
    double maximum_step_horizontal_gap = 0.30;
    double boundary_height_tolerance = 0.20;
    double preferred_boundary_clearance_m = 0.39;
    std::size_t minimum_component_size = 3;
    std::size_t max_iterations = 250000;
    double maximum_planning_time_ms = 60000;
};

struct SearchResult {
    bool success = false;
    std::string status;
    Path path;
    std::size_t iterations = 0;
    double time_ms = 0;
    double total_cost = 0;
    double connectivity_cost = 0;
    double minimum_boundary_clearance_m = 0;
};

// An immutable prepared graph can be reused for successive exact endpoint queries.
class GraphPlanner {
  public:
    explicit GraphPlanner(const GridMap& grid, SearchOptions options = {});
    SearchResult plan(const Point& start, const Point& goal, double budget_ms = -1) const;

  private:
    struct Impl;
    std::shared_ptr<const Impl> impl_;
};

SearchResult plan_path(const GridMap& grid, const Point& start, const Point& goal,
                       SearchOptions options = {});

struct Validation {
    bool valid = true;
    int outside_count = 0, height_violation_count = 0, diagonal_violation_count = 0;
    int transition_violation_count = 0, deviation_violation_count = 0,
        clearance_violation_count = 0;
    int inherited_violation_count = 0, new_violation_count = 0;
    double clearance_floor_m = 0, clearance_target_m = 0;
    double max_deviation_m = 0, sample_spacing_m = 0;
    std::size_t sample_count = 0;
    std::vector<std::size_t> invalid_indices;
};

struct Metrics {
    double length_m = 0, length3d_m = 0, turning_deg = 0;
    double curvature_rms = 0, curvature_p95 = 0, max_heading_change_deg = 0;
    int sharp_turns = 0, dense_sharp_turns = 0;
    double min_clearance_m = 0, clearance_p05_m = 0, below_clearance_percent = 0;
    Path sharp_turn_points;
    std::vector<double> profile_s, profile_curvature;
};

struct SolverInfo {
    bool converged = false;
    int iterations = 0, repair_passes = 0, control_points = 0, refinement_passes = 0;
    std::string message;
    double time_ms = 0, retained_percent = 0, minimum_speed = 0;
    int constraint_samples = 0, height_rematched_samples = 0;
};

struct OptimizationResult {
    Options options;
    Path path, reference, raw_candidate, source_route, shortcut_path;
    Metrics baseline, optimized;
    Validation validation, baseline_validation;
    SolverInfo solver;
    int shortcut_segments = 0;
    double shortcut_length_saved_m = 0;
    std::vector<std::size_t> retained_indices;
    std::vector<std::pair<double, double>> protected_spans;
};

std::vector<double> arc_length(const Path& path);
std::pair<Path, std::vector<double>> resample(const Path& path, double spacing = 0.05);
Metrics metrics(const GridMap& grid, const Path& path, double target_clearance);
OptimizationResult optimize(const GridMap& grid, const Path& reference, Options options = {});

} // namespace terrain_planner
