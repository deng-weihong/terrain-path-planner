#include "internal.hpp"

#include <chrono>
#include <queue>
#include <tuple>

namespace terrain_planner {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
struct TimeLimit : std::runtime_error {
    TimeLimit() : std::runtime_error("Graph preparation reached its time limit") {}
};
struct Edge {
    int target;
    double distance;
    bool step;
};
constexpr int directions[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                  {0, 1},   {1, -1}, {1, 0},  {1, 1}};
} // namespace

struct GraphPlanner::Impl {
    GridMap grid;
    SearchOptions options;
    std::vector<Key> keys;
    Path points;
    std::map<Key, int> indices;
    std::map<std::pair<int, int>, std::vector<int>> columns;
    std::vector<std::vector<Edge>> adjacency;
    std::vector<double> boundary, clearance, penalty;
    std::vector<int> component;
    std::vector<std::size_t> component_size;
    double slope, preferred;
    int boundary_cells;
    Clock::time_point started = Clock::now();

    Impl(const GridMap& input, SearchOptions settings) : grid(input), options(settings) {
        const double r = grid.resolution();
        const double values[] = {
            options.ground_neighbor_radius,    options.ground_height_tolerance,
            options.maximum_slope_deg,         options.minimum_step_height,
            options.maximum_step_height,       options.maximum_step_horizontal_gap,
            options.boundary_height_tolerance, options.preferred_boundary_clearance_m,
            options.maximum_planning_time_ms};
        for (double value : values)
            if (!std::isfinite(value) || value < 0)
                throw std::invalid_argument("Invalid search parameter");
        if (options.maximum_slope_deg >= 89 || options.ground_neighbor_radius <= 0 ||
            options.maximum_planning_time_ms <= 0 || options.max_iterations == 0 ||
            options.maximum_step_height < options.minimum_step_height ||
            options.minimum_component_size == 0)
            throw std::invalid_argument("Invalid search limits");
        preferred = options.preferred_boundary_clearance_m > detail::epsilon
                        ? options.preferred_boundary_clearance_m
                        : 2 * r;
        const double radius =
            std::max(options.ground_neighbor_radius, options.maximum_step_horizontal_gap);
        if (std::ceil(std::max(radius, preferred + 0.5 * r) / r) > 1000)
            throw std::invalid_argument(
                "Search neighborhood exceeds 1000 cells; adjust resolution or search options");
        slope = std::tan(options.maximum_slope_deg * detail::pi / 180);
        boundary_cells =
            static_cast<int>((options.boundary_height_tolerance + detail::epsilon) / r);
        for (const auto& key : grid.keys()) {
            check(keys.size());
            if (!indices.emplace(key, static_cast<int>(keys.size())).second)
                continue;
            const int index = static_cast<int>(keys.size());
            keys.push_back(key);
            points.push_back({(key[0] + .5) * r, (key[1] + .5) * r, (key[2] + .5) * r});
            columns[{key[0], key[1]}].push_back(index);
        }
        boundary.resize(keys.size());
        clearance.resize(keys.size());
        penalty.resize(keys.size());
        std::vector<std::tuple<double, int, int>> offsets;
        const int cells = static_cast<int>(std::ceil((preferred + .5 * r) / r));
        for (int dx = -cells; dx <= cells; ++dx)
            for (int dy = -cells; dy <= cells; ++dy) {
                const double distance = r * std::hypot(dx, dy);
                if ((dx || dy) && distance <= preferred + .5 * r + detail::epsilon)
                    offsets.emplace_back(std::max(0.0, distance - .5 * r), dx, dy);
            }
        std::sort(offsets.begin(), offsets.end());
        for (std::size_t i = 0; i < keys.size(); ++i) {
            check(i);
            const auto& key = keys[i];
            int supported = 0;
            for (const auto& d : directions)
                if (surface(key[0] + d[0], key[1] + d[1], key[2]))
                    ++supported;
            boundary[i] = (8.0 - supported) / 8;
            clearance[i] = preferred;
            for (const auto& [distance, dx, dy] : offsets) {
                if (distance >= preferred)
                    break;
                if (!surface(key[0] + dx, key[1] + dy, key[2])) {
                    clearance[i] = distance;
                    break;
                }
            }
            penalty[i] = std::max(0.0, preferred - clearance[i]);
        }
        adjacency.resize(keys.size());
        std::vector<std::tuple<int, int, double, double>> neighbor_offsets;
        const int search_cells = static_cast<int>(std::ceil(radius / r));
        for (int dx = 0; dx <= search_cells; ++dx)
            for (int dy = -search_cells; dy <= search_cells; ++dy) {
                const double horizontal = r * std::hypot(dx, dy);
                if ((dx > 0 || dy > 0) && horizontal <= radius + detail::epsilon)
                    neighbor_offsets.emplace_back(
                        dx, dy, horizontal, options.ground_height_tolerance + slope * horizontal);
            }
        std::size_t ci = 0;
        for (const auto& [xy, sources] : columns) {
            check(ci++);
            for (const auto& [dx, dy, horizontal, normal_limit] : neighbor_offsets) {
                const auto found = columns.find({xy.first + dx, xy.second + dy});
                if (found == columns.end())
                    continue;
                for (int a : sources)
                    for (int b : found->second) {
                        const double height = std::abs(points[a][2] - points[b][2]);
                        const bool ordinary =
                            horizontal <= options.ground_neighbor_radius + detail::epsilon &&
                            height <= normal_limit + detail::epsilon;
                        const bool step =
                            horizontal <= options.maximum_step_horizontal_gap + detail::epsilon &&
                            height >= options.minimum_step_height - detail::epsilon &&
                            height <= options.maximum_step_height + detail::epsilon &&
                            boundary[a] <= .75 && boundary[b] <= .75;
                        if ((!ordinary && !step) || (ordinary && !diagonal_support(a, b)))
                            continue;
                        const double distance = std::hypot(horizontal, height);
                        adjacency[a].push_back({b, distance, !ordinary});
                        adjacency[b].push_back({a, distance, !ordinary});
                    }
            }
        }
        component.assign(keys.size(), -1);
        for (std::size_t seed = 0; seed < keys.size(); ++seed) {
            check(seed);
            if (component[seed] != -1)
                continue;
            const int id = static_cast<int>(component_size.size());
            component_size.push_back(0);
            std::vector<int> frontier{static_cast<int>(seed)};
            component[seed] = id;
            while (!frontier.empty()) {
                check(component_size[id]);
                const int current = frontier.back();
                frontier.pop_back();
                ++component_size[id];
                for (const auto& edge : adjacency[current])
                    if (component[edge.target] == -1) {
                        component[edge.target] = id;
                        frontier.push_back(edge.target);
                    }
            }
        }
        check(0);
    }

    void check(std::size_t index) const {
        if ((index & 255) == 0 && elapsed(started) >= options.maximum_planning_time_ms)
            throw TimeLimit();
    }
    bool surface(int x, int y, int z) const {
        const auto& heights = grid.heights(x, y);
        const double lo = (double(z) - boundary_cells + .5) * grid.resolution();
        const auto it = std::lower_bound(heights.begin(), heights.end(), lo - detail::epsilon);
        return it != heights.end() &&
               *it <= (double(z) + boundary_cells + .5) * grid.resolution() + detail::epsilon;
    }
    bool diagonal_support(int a, int b) const {
        const auto& first = keys[a];
        const auto& second = keys[b];
        if (first[0] == second[0] || first[1] == second[1])
            return true;
        const double r = grid.resolution();
        const double dx = std::abs(first[0] - second[0]) * r,
                     dy = std::abs(first[1] - second[1]) * r;
        if (std::max(dx, dy) > options.ground_neighbor_radius + detail::epsilon)
            return false;
        const int lx =
            static_cast<int>((options.ground_height_tolerance + slope * dx + detail::epsilon) / r);
        const int ly =
            static_cast<int>((options.ground_height_tolerance + slope * dy + detail::epsilon) / r);
        auto bridge = [&](int x, int y, int l1, int l2) {
            const double lo = (std::max(first[2] - l1, second[2] - l2) + .5) * r;
            const double hi = (std::min(first[2] + l1, second[2] + l2) + .5) * r;
            const auto& zs = grid.heights(x, y);
            const auto it = std::lower_bound(zs.begin(), zs.end(), lo - detail::epsilon);
            return it != zs.end() && *it <= hi + detail::epsilon;
        };
        return bridge(second[0], first[1], lx, ly) && bridge(first[0], second[1], ly, lx);
    }
    int endpoint(const Point& p) const {
        if (!detail::finite(p))
            return -1;
        const double r = grid.resolution();
        Key key{detail::cell(p[0], r), detail::cell(p[1], r), detail::cell(p[2], r)};
        const auto found = indices.find(key);
        if (found == indices.end())
            return -1;
        const int i = found->second;
        return detail::xy_distance(points[i], p) <= 1e-6 && std::abs(points[i][2] - p[2]) <= 1e-6 &&
                       component_size[component[i]] >= options.minimum_component_size
                   ? i
                   : -1;
    }

    SearchResult plan(const Point& start, const Point& goal, double budget_ms) const {
        const auto begin = Clock::now();
        SearchResult result;
        auto finish = [&](const std::string& status) {
            result.status = status;
            result.success = status == "success";
            result.time_ms = elapsed(begin);
            return result;
        };
        const double limit = budget_ms < 0 ? options.maximum_planning_time_ms
                                           : std::min(budget_ms, options.maximum_planning_time_ms);
        if (!std::isfinite(limit) || limit <= 0)
            return finish("time_limit");
        const int a = endpoint(start);
        if (a < 0)
            return finish("start_not_on_map");
        const int b = endpoint(goal);
        if (b < 0 || component[a] != component[b])
            return finish("goal_not_on_start_component");
        const double inf = std::numeric_limits<double>::infinity();
        std::vector<double> connectivity(points.size(), inf), physical(points.size(), inf);
        std::vector<int> parent(points.size(), -1);
        std::vector<bool> closed(points.size(), false);
        using Entry = std::tuple<double, double, double, double, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
        connectivity[a] = physical[a] = 0;
        parent[a] = a;
        frontier.emplace(0, detail::distance(points[a], points[b]), 0, 0, a);
        bool found = false;
        while (!frontier.empty()) {
            const auto [cf, pf, cost_c, cost_p, current] = frontier.top();
            frontier.pop();
            (void)cf;
            (void)pf;
            if (closed[current] || cost_c != connectivity[current] ||
                cost_p > physical[current] + detail::epsilon)
                continue;
            closed[current] = true;
            ++result.iterations;
            if (current == b) {
                found = true;
                break;
            }
            if (result.iterations >= options.max_iterations)
                return finish("expansion_limit");
            if ((result.iterations & 255) == 0 && elapsed(begin) >= limit)
                return finish("time_limit");
            for (const auto& edge : adjacency[current]) {
                const int next = edge.target;
                if (closed[next])
                    continue;
                double angle = 0;
                if (parent[current] != current) {
                    const auto& p = points[parent[current]];
                    const auto& q = points[current];
                    const auto& v = points[next];
                    const double l1 = detail::xy_distance(p, q), l2 = detail::xy_distance(q, v);
                    if (l1 > detail::epsilon && l2 > detail::epsilon)
                        angle = std::acos(detail::clamp(
                            ((q[0] - p[0]) * (v[0] - q[0]) + (q[1] - p[1]) * (v[1] - q[1])) /
                                (l1 * l2),
                            -1, 1));
                }
                const double cc = connectivity[current] + .5 * (penalty[current] + penalty[next]);
                const double pc = physical[current] + edge.distance +
                                  std::abs(points[next][2] - points[current][2]) +
                                  .125 * (boundary[current] + boundary[next]) + .05 * angle +
                                  (edge.step ? .2 : 0);
                if (cc + detail::epsilon < connectivity[next] ||
                    (std::abs(cc - connectivity[next]) <= detail::epsilon &&
                     pc + detail::epsilon < physical[next])) {
                    connectivity[next] = cc;
                    physical[next] = pc;
                    parent[next] = current;
                    frontier.emplace(cc, pc + detail::distance(points[next], points[b]), cc, pc,
                                     next);
                }
            }
        }
        if (!found)
            return finish("no_path");
        result.minimum_boundary_clearance_m = preferred;
        for (int i = b;; i = parent[i]) {
            result.path.push_back(points[i]);
            result.connectivity_cost += penalty[i];
            result.minimum_boundary_clearance_m =
                std::min(result.minimum_boundary_clearance_m, clearance[i]);
            if (i == a)
                break;
        }
        std::reverse(result.path.begin(), result.path.end());
        result.total_cost = physical[b];
        return finish(elapsed(begin) >= limit ? "time_limit" : "success");
    }
};

GraphPlanner::GraphPlanner(const GridMap& grid, SearchOptions options)
    : impl_(std::make_shared<Impl>(grid, options)) {}
SearchResult GraphPlanner::plan(const Point& start, const Point& goal, double budget_ms) const {
    return impl_->plan(start, goal, budget_ms);
}

SearchResult plan_path(const GridMap& grid, const Point& start, const Point& goal,
                       SearchOptions options) {
    const auto started = Clock::now();
    SearchResult result;
    try {
        const GraphPlanner planner(grid, options);
        result = planner.plan(start, goal,
                              std::max(0.0, options.maximum_planning_time_ms - elapsed(started)));
    } catch (const TimeLimit&) {
        result.status = "time_limit";
    }
    result.time_ms = elapsed(started);
    if (result.time_ms >= options.maximum_planning_time_ms) {
        result.success = false;
        result.status = "time_limit";
        result.path.clear();
    }
    return result;
}

} // namespace terrain_planner
