#include "terrain_planner/json.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>

using namespace terrain_planner;
void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

std::vector<Key> rectangle(int width, int height, int x0 = 0, int z = 0) {
    std::vector<Key> keys;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            keys.push_back({x + x0, y, z});
    return keys;
}

int main() {
    int failures = 0;
    auto test = [&](const std::string& name, const std::function<void()>& fn) {
        try {
            fn();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    };
    test("signed distance and negative cells", [] {
        GridMap grid({{-1, 0, 0}, {0, 0, 0}, {1, 0, 0}});
        require(grid.occupied({-.05, .05, .05}), "Negative coordinates must use floor");
        require(std::abs(grid.field({-.05, .05, .05}).distance - .05) < 1e-12,
                "Incorrect inside EDT");
        require(std::abs(grid.field({-.05, .15, .05}).distance + .05) < 1e-12,
                "Incorrect outside EDT");
    });
    test("reusable graph and exact endpoints", [] {
        GraphPlanner graph(GridMap(rectangle(12, 9)));
        const Point a{.15, .15, .05}, b{1.05, .75, .05};
        auto forward = graph.plan(a, b), reverse = graph.plan(b, a);
        require(forward.success && reverse.success, "Open map should be connected");
        for (int dim = 0; dim < 3; ++dim)
            require(std::abs(forward.path.front()[dim] - a[dim]) < 1e-9 &&
                        std::abs(forward.path.back()[dim] - b[dim]) < 1e-9,
                    "Endpoints changed");
        require(!graph.plan({.15, .15, .85}, b).success, "Wrong floor accepted");
        require(graph.plan(a, b, 0).status == "time_limit", "Zero query budget ignored");
    });
    test("unknown gap", [] {
        auto keys = rectangle(3, 3), other = rectangle(3, 3, 5);
        keys.insert(keys.end(), other.begin(), other.end());
        auto result = plan_path(GridMap(keys), {.05, .05, .05}, {.55, .05, .05});
        require(!result.success && result.status == "goal_not_on_start_component", "Gap crossed");
    });
    test("diagonal corner support", [] {
        GridMap grid({{-2, 0, 0}, {-1, 0, 0}, {0, 0, 0}, {1, 1, 0}, {2, 1, 0}, {3, 1, 0}});
        require(!plan_path(grid, {.05, .05, .05}, {.15, .15, .05}).success,
                "Unsupported diagonal crossed");
    });
    test("separate floors", [] {
        auto keys = rectangle(5, 5), upper = rectangle(5, 5, 0, 8);
        keys.insert(keys.end(), upper.begin(), upper.end());
        require(!plan_path(GridMap(keys), {.05, .05, .05}, {.45, .45, .85}).success,
                "Floors merged");
    });
    test("explicit supported step", [] {
        auto keys = rectangle(2, 3), step = rectangle(2, 3, 3, 2);
        keys.insert(keys.end(), step.begin(), step.end());
        auto result = plan_path(GridMap(keys), {.05, .05, .05}, {.45, .25, .25});
        require(result.success, "Supported step rejected");
        bool transition = false;
        for (std::size_t i = 1; i < result.path.size(); ++i)
            transition = transition || std::abs(result.path[i][2] - result.path[i - 1][2]) > .19;
        require(transition, "Step was flattened");
    });
    test("search expansion limit", [] {
        SearchOptions options;
        options.max_iterations = 1;
        const auto result =
            plan_path(GridMap(rectangle(10, 10)), {.05, .05, .05}, {.95, .95, .05}, options);
        require(!result.success && result.status == "expansion_limit", "Search limit ignored");
    });
    test("whole-route smoothing", [] {
        GridMap grid(rectangle(140, 80));
        Path route;
        for (int i = 0; i <= 80; ++i)
            route.push_back({1.05 + .15 * i, 4.05 + .3 * std::sin(i * .5), .05});
        const auto result = optimize(grid, route);
        require(result.validation.valid, "Output failed geometry validation");
        require(result.path.front() == route.front() && result.path.back() == route.back(),
                "Endpoints changed");
        require(result.optimized.curvature_rms < .3 * result.baseline.curvature_rms,
                "Repeated bends remain");
        require(result.validation.max_deviation_m <= .800001, "Corridor exceeded");
    });
    test("elevated reference filters a bump while preserving ground", [] {
        auto keys = rectangle(110, 80);
        for (auto& key : keys)
            key[2] = key[0] == 42 ? 1 : 0;
        const Path route{{1.05, 4.05, .05}, {4.15, 4.05, .05}, {4.25, 4.05, .15},
                         {4.35, 4.05, .05}, {9.05, 4.05, .05}};
        const GridMap grid(keys);
        const auto result = optimize(grid, route);
        require(result.path.front() == route.front() && result.path.back() == route.back(),
                "Ground endpoints changed");
        require(result.validation.valid && result.navigation.valid, "Validation failed");
        require(result.navigation.smoothed_samples > 0, "Reference not smoothed");
        double minimum = 1, maximum = 0;
        for (std::size_t i = 0; i < result.path.size(); ++i) {
            require(result.path[i][0] == result.navigation_path[i][0] &&
                        result.path[i][1] == result.navigation_path[i][1], "Reference XY moved");
            minimum = std::min(minimum, result.navigation_path[i][2]);
            maximum = std::max(maximum, result.navigation_path[i][2]);
        }
        require(maximum - minimum < .02, "One-cell bump not filtered");
        require(result.navigation.min_ground_offset_m >= .2 - 1e-8, "Reference too low");
        require(result.navigation.max_ground_offset_m <= .5 + 1e-8, "Reference too high");
        Options options;
        options.navigation_height = .8;
        const auto higher = optimize(grid, route, options);
        require(higher.path == result.path, "Reference height changed ground route");
        require(std::abs(higher.navigation_path.front()[2] - route.front()[2] - .8) < 1e-12,
                "Elevated endpoint height incorrect");
    });
    test("invalid baseline is reported", [] {
        const auto result = optimize(GridMap({{0, 0, 0}, {2, 0, 0}, {3, 0, 0}}),
                                     {{.05, .05, .05}, {.35, .05, .05}});
        require(!result.validation.valid && result.validation.outside_count > 0,
                "Gap hidden by refinement");
    });
    test("invalid parameters and JSON", [] {
        bool rejected = false;
        try {
            optimize(GridMap(rectangle(10, 10)), {{.05, .05, .05}, {.05, .05, .05}});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Degenerate route accepted");
        rejected = false;
        try {
            run_request({{"resolution", .1},
                         {"voxels", {{0, 0, 2147483648LL}}},
                         {"route", {{0, 0, 0}, {1, 0, 0}}}});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Out-of-range voxel accepted");
    });
    return failures == 0 ? 0 : 1;
}
