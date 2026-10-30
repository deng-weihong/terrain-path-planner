#include "terrain_planner/planner.hpp"

#include <cmath>
#include <iostream>

int main() {
    using namespace terrain_planner;
    std::vector<Key> keys;
    for (int y = 0; y < 80; ++y)
        for (int x = 0; x < 140; ++x)
            keys.push_back({x, y, 0});
    GridMap grid(std::move(keys), 0.1);
    Path route;
    for (int i = 0; i <= 80; ++i)
        route.push_back({1.05 + i * .15, 4.05 + .3 * std::sin(i * .5), .05});
    const auto result = optimize(grid, route);
    std::cout << "Samples: " << result.path.size() << ", valid: " << result.validation.valid
              << ", curvature RMS: " << result.baseline.curvature_rms << " -> "
              << result.optimized.curvature_rms << '\n'
              << "Navigation reference samples: " << result.navigation_path.size()
              << ", ground offset: " << result.navigation.min_ground_offset_m << " to "
              << result.navigation.max_ground_offset_m << " m"
              << ", height envelope valid: " << result.navigation.height_envelope_valid << '\n';
    return result.validation.valid && result.navigation.valid &&
                   result.optimized.curvature_rms < result.baseline.curvature_rms
               ? 0
               : 1;
}
