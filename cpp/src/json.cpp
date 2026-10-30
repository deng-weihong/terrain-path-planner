#include "terrain_planner/json.hpp"
#include "internal.hpp"

namespace terrain_planner {
namespace {
using Json = nlohmann::json;

Json validation_json(const Validation& v) {
    return {{"valid", v.valid},
            {"outsideCount", v.outside_count},
            {"heightViolationCount", v.height_violation_count},
            {"diagonalViolationCount", v.diagonal_violation_count},
            {"transitionViolationCount", v.transition_violation_count},
            {"deviationViolationCount", v.deviation_violation_count},
            {"clearanceViolationCount", v.clearance_violation_count},
            {"clearanceFloorM", v.clearance_floor_m},
            {"clearanceTargetM", v.clearance_target_m},
            {"invalidIndices", v.invalid_indices},
            {"maxDeviationM", v.max_deviation_m},
            {"sampleCount", v.sample_count},
            {"sampleSpacingM", v.sample_spacing_m},
            {"inheritedViolationCount", v.inherited_violation_count},
            {"newViolationCount", v.new_violation_count},
            {"noNewViolations", v.new_violation_count == 0}};
}
Json metrics_json(const Metrics& m) {
    return {{"lengthM", m.length_m},
            {"length3dM", m.length3d_m},
            {"turningDeg", m.turning_deg},
            {"curvatureRms", m.curvature_rms},
            {"curvatureP95", m.curvature_p95},
            {"sharpTurns", m.sharp_turns},
            {"maxHeadingChangeDeg", m.max_heading_change_deg},
            {"denseSharpTurns", m.dense_sharp_turns},
            {"sharpTurnPoints", m.sharp_turn_points},
            {"minClearanceM", m.min_clearance_m},
            {"clearanceP05M", m.clearance_p05_m},
            {"belowClearancePercent", m.below_clearance_percent},
            {"profile", {{"s", m.profile_s}, {"curvature", m.profile_curvature}}}};
}
Point point(const Json& value) {
    if (!value.is_array() || value.size() != 3)
        throw std::invalid_argument("Each point must be an XYZ triple");
    Point p;
    for (int dim = 0; dim < 3; ++dim) {
        if (!value[dim].is_number())
            throw std::invalid_argument("Point coordinates must be numbers");
        p[dim] = value[dim].get<double>();
    }
    if (!detail::finite(p))
        throw std::invalid_argument("Point coordinates must be finite");
    return p;
}
Json planning_json(const SearchResult& s, const Point& start, const Point& goal) {
    return {{"engine", "traversable-graph-cpp-v1"},
            {"start", start},
            {"goal", goal},
            {"iterations", s.iterations},
            {"timeMs", s.time_ms},
            {"waypointCount", s.path.size()},
            {"status", s.status},
            {"totalCost", s.total_cost},
            {"connectivityCost", s.connectivity_cost},
            {"minimumBoundaryClearanceM", s.minimum_boundary_clearance_m}};
}
} // namespace

nlohmann::json to_json(const OptimizationResult& r) {
    Json spans = Json::array();
    for (const auto& span : r.protected_spans)
        spans.push_back({{"startM", span.first}, {"endM", span.second}});
    const auto& s = r.solver;
    return {{"algorithm", "corridor-shortcut-bspline-v3"},
            {"engine", "cpp"},
            {"options",
             {{"smoothness", r.options.smoothness},
              {"max_deviation", r.options.max_deviation},
              {"clearance", r.options.clearance},
              {"control_spacing", r.options.control_spacing},
              {"navigation_height", r.options.navigation_height},
              {"elevation_radius", r.options.elevation_radius}}},
            {"baseline", metrics_json(r.baseline)},
            {"optimized", metrics_json(r.optimized)},
            {"validation", validation_json(r.validation)},
            {"baselineValidation", validation_json(r.baseline_validation)},
            {"path", r.path},
            {"resultSchemaVersion", 2},
            {"pathRole", "ground-support"},
            {"navigationPath", r.navigation_path},
            {"navigation",
             {{"algorithm", "elevated-reference-v1"},
              {"heightM", r.navigation.height_m},
              {"smoothingRadiusM", r.navigation.smoothing_radius_m},
              {"minimumAllowedHeightM", r.navigation.minimum_allowed_height_m},
              {"maximumAllowedHeightM", r.navigation.maximum_allowed_height_m},
              {"minGroundOffsetM", r.navigation.min_ground_offset_m},
              {"maxGroundOffsetM", r.navigation.max_ground_offset_m},
              {"smoothedSamples", r.navigation.smoothed_samples},
              {"maxSmoothingAdjustmentM", r.navigation.max_smoothing_adjustment_m},
              {"maxPitchChangeDeg", r.navigation.max_pitch_change_deg},
              {"length3dM", r.navigation.length3d_m},
              {"validation", {{"valid", r.navigation.valid},
                              {"groundValid", r.navigation.ground_valid},
                              {"heightEnvelopeValid", r.navigation.height_envelope_valid},
                              {"ceilingClearanceVerified", false},
                              {"robotEnvelopeVerified", false}}}}},
            {"reference", r.reference},
            {"rawCandidate", r.raw_candidate},
            {"sourceRoute", r.source_route},
            {"shortcut",
             {{"segments", r.shortcut_segments},
              {"lengthSavedM", r.shortcut_length_saved_m},
              {"path", r.shortcut_path}}},
            {"retainedIndices", r.retained_indices},
            {"protectedSpans", spans},
            {"solver",
             {{"converged", s.converged},
              {"iterations", s.iterations},
              {"message", s.message},
              {"repairPasses", s.repair_passes},
              {"controlPoints", s.control_points},
              {"timeMs", s.time_ms},
              {"retainedPercent", s.retained_percent},
              {"continuity", "C2"},
              {"refinementPasses", s.refinement_passes},
              {"minimumSpeed", s.minimum_speed},
              {"constraintSamples", s.constraint_samples},
              {"heightRematchedSamples", s.height_rematched_samples}}},
            {"scope",
             {{"checks",
               {"XY occupancy", "diagonal support", "local terrain layer", "preserved steps",
                "deviation", "boundary floor", "endpoints"}},
              {"production3dGraphVerified", false},
              {"validationPath", "path"},
              {"navigationPathRole", "elevated-reference"},
              {"ceilingClearanceVerified", false},
              {"robotEnvelopeVerified", false},
              {"robotExecutionVerified", false}}}};
}

nlohmann::json run_request(const nlohmann::json& request) {
    if (!request.is_object())
        throw std::invalid_argument("Request must be a JSON object");
    const auto& voxels = request.at("voxels");
    if (!voxels.is_array() || voxels.empty() || voxels.size() > 2000000)
        throw std::invalid_argument("voxels must contain 1 to 2000000 int32 XYZ keys");
    std::vector<Key> keys;
    keys.reserve(voxels.size());
    for (const auto& row : voxels) {
        if (!row.is_array() || row.size() != 3)
            throw std::invalid_argument("Voxel keys must be int32 triples");
        Key key;
        for (int dim = 0; dim < 3; ++dim) {
            if (!row[dim].is_number_integer() ||
                row[dim].get<double>() < std::numeric_limits<std::int32_t>::min() ||
                row[dim].get<double>() > std::numeric_limits<std::int32_t>::max())
                throw std::invalid_argument("Voxel indices must be int32 integers");
            key[dim] = row[dim].get<std::int32_t>();
        }
        keys.push_back(key);
    }
    const double resolution = request.at("resolution").get<double>();
    GridMap grid(std::move(keys), resolution);
    Options options;
    if (request.contains("options")) {
        const auto& o = request.at("options");
        if (!o.is_object())
            throw std::invalid_argument("options must be an object");
        options.smoothness = o.value("smoothness", options.smoothness);
        options.max_deviation = o.value("max_deviation", options.max_deviation);
        options.clearance = o.value("clearance", options.clearance);
        options.control_spacing = o.value("control_spacing", options.control_spacing);
        options.navigation_height = o.value("navigation_height", options.navigation_height);
        options.elevation_radius = o.value("elevation_radius", options.elevation_radius);
    }
    const auto operation = request.value("operation", std::string("optimize"));
    if (operation != "search" && operation != "optimize")
        throw std::invalid_argument("operation must be search or optimize");
    const bool endpoints = request.contains("start") || request.contains("goal");
    if (endpoints &&
        (request.contains("route") || !request.contains("start") || !request.contains("goal")))
        throw std::invalid_argument("Provide either route or both start and goal");
    Path route;
    Json planning = nullptr;
    if (endpoints) {
        const Point start = point(request.at("start")), goal = point(request.at("goal"));
        SearchOptions search;
        search.preferred_boundary_clearance_m = options.clearance;
        if (request.contains("search")) {
            const auto& o = request.at("search");
            if (!o.is_object())
                throw std::invalid_argument("search must be an object");
            search.maximum_planning_time_ms =
                o.value("maximum_planning_time_ms", search.maximum_planning_time_ms);
            const double limit = o.value("max_iterations", double(search.max_iterations));
            if (!std::isfinite(limit) || limit < 1 || limit > 2500000 || std::floor(limit) != limit)
                throw std::invalid_argument("max_iterations must be an integer from 1 to 2500000");
            search.max_iterations = static_cast<std::size_t>(limit);
        }
        const auto result = plan_path(grid, start, goal, search);
        planning = planning_json(result, start, goal);
        if (operation == "search")
            return {{"engine", "cpp"},
                    {"success", result.success},
                    {"status", result.status},
                    {"path", result.path},
                    {"planning", planning}};
        if (!result.success)
            throw std::invalid_argument(std::string("Search failed: ") + result.status);
        route = result.path;
    } else {
        if (operation == "search")
            throw std::invalid_argument("Search requires start and goal");
        const auto& input = request.at("route");
        if (!input.is_array() || input.size() < 2 || input.size() > 20000)
            throw std::invalid_argument("route must contain 2 to 20000 XYZ points");
        for (const auto& p : input)
            route.push_back(point(p));
    }
    auto result = to_json(optimize(grid, route, options));
    result["planning"] = planning;
    result["dataset"] = request.value("dataset", std::string("custom"));
    result["mapSha256"] = request.value("mapSha256", std::string(""));
    result["jobId"] = request.value("jobId", std::string("native"));
    return result;
}

} // namespace terrain_planner
