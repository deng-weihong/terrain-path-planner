#pragma once

#include "terrain_planner/planner.hpp"
#include <nlohmann/json.hpp>

namespace terrain_planner {

nlohmann::json to_json(const OptimizationResult& result);
// Requests contain resolution, voxels, either route or start/goal, and optional options.
nlohmann::json run_request(const nlohmann::json& request);

} // namespace terrain_planner
