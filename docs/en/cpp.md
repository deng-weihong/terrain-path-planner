# C++ Usage and Design

[简体中文](../cpp.md) | [English](cpp.md)

## Standalone Build

The core uses C++17, Eigen 3.3+, and LBFGSpp v0.3.0. The JSON adapter uses nlohmann/json. No Python, ROS, Node.js, or HTTP service is required.

```bash
sudo apt-get install -y build-essential cmake libeigen3-dev nlohmann-json3-dev
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build/cpp --parallel 2
ctest --test-dir build/cpp --output-on-failure
./build/cpp/terrain-plan-example
```

The first configuration downloads LBFGSpp v0.3.0 and verifies a pinned SHA-256 digest. Supply existing headers for an offline configuration:

```bash
cmake -S cpp -B build/cpp -DLBFGSPP_INCLUDE_DIR=/path/to/LBFGSpp/include
```

Use `Eigen3_DIR` for a custom Eigen installation and `NLOHMANN_JSON_INCLUDE_DIR` for custom JSON headers. In-source builds are rejected; dependencies and generated files stay under `build/` and are excluded from source archives.

Install and consume the library independently:

```bash
cmake --install build/cpp --prefix /tmp/terrain-planner-install
cmake -S cpp/examples/consumer -B build/consumer -DCMAKE_PREFIX_PATH=/tmp/terrain-planner-install
cmake --build build/consumer
./build/consumer/consumer
```

Downstream CMake uses `find_package(terrain_planner CONFIG REQUIRED)` and links `terrain_planner::terrain_planner`. Add `terrain_planner::terrain_planner_json` when using the JSON adapter.

## Native Library API

```cpp
#include <terrain_planner/planner.hpp>

using namespace terrain_planner;
std::vector<Key> keys{{0,0,0}, {1,0,0}, {2,0,0}, {3,0,0}, {4,0,0}};
GridMap map(keys, 0.1);
GraphPlanner graph(map);
auto search = graph.plan({0.05,0.05,0.05}, {0.45,0.05,0.05});
if (search.success) {
    auto result = optimize(map, search.path);
    // Read result.validation and result.solver before using result.path.
}
```

`Point` is `std::array<double,3>`, `Key` is `std::array<int32_t,3>`, and `Path` is a list of XYZ points. Public headers do not require Eigen includes. `GridMap` retains independent height columns and an XY distance field. `GraphPlanner` prepares 3D adjacency, boundary features, and components; reuse it for different endpoints and reconstruct it after changing the map.

`SearchOptions` defines slope, step, clearance, component size, expansion, and time limits. Endpoints must match exact voxel centers within 1e-6 m. `plan_path()` times preparation and search together. `GraphPlanner::plan()` times the current query and accepts an optional smaller budget. Results include `success`, `status`, the raw graph route, costs, and elapsed time. Defaults are 250,000 expansions and 60 seconds.

`Options` matches the four Python parameters. `OptimizationResult` contains the curve, reference route, validation, curvature and clearance metrics, solver state, and protected spans. Invalid inputs throw standard C++ exceptions; failed geometry checks remain visible in the returned result and counters.

## JSON CLI

```bash
./build/cpp/terrain-plan --input examples/native-request.json --output result.json
```

Without file arguments, the program reads one JSON object from stdin and writes the result to stdout. Errors are JSON objects with an `error` field on stderr and exit code 2. A completed calculation exits with 0; callers must still inspect result fields.

Requests contain `resolution` and an integer XYZ array, `voxels`. These represent traversable ground, not obstacles. Other fields are:

| Field | Usage |
| --- | --- |
| `route` | Metric XYZ route to smooth; excludes endpoints |
| `start` / `goal` | Supply both for native XYZ graph search first |
| `options` | `smoothness`, `max_deviation`, `clearance`, `control_spacing` |
| `operation` | Defaults to `optimize`; `search` returns graph search only |
| `search` | Overrides `max_iterations` and `maximum_planning_time_ms` |
| `dataset` / `mapSha256` / `jobId` | Optional provenance copied to optimization output |

`operation=search` returns normally even for unreachable endpoints; inspect `success` and `status`. The default complete pipeline reports an error when search fails. Search statuses include `success`, `start_not_on_map`, `goal_not_on_start_component`, `no_path`, `expansion_limit`, and `time_limit`.

Maps accept at most 2,000,000 input voxels and 20,000,000 padded XY distance-field cells. Routes accept at most 20,000 input points and 0.1-1000 m of XY length. Oversized inputs are rejected. Complete optimization output uses the viewer's fields and adds `engine: "cpp"`.

## Numerical Methods

Graph search preserves XYZ connectivity, diagonal bridges, slopes, and explicit steps. It compares boundary-related connectivity costs lexicographically before costs for distance, vertical movement, boundaries, turns, and steps. As in the Python viewer entry point, raw graph vertices feed whole-path optimization.

The distance field uses a separable squared Euclidean transform, evaluating lower envelopes of parabolas before taking square roots and subtracting half a cell. Queries use bilinear interpolation and analytic gradients. Height columns are checked separately so overhead ground in the XY projection does not support the current floor.

The initial cubic B-spline uses sparse basis and second-derivative matrices. LBFGSpp jointly optimizes interior controls using L-BFGS-B. Objective terms, control bounds, narrow-endpoint tangents, route corridors, protected terrain, and adaptive clearance correspond to Python. Dense checks and repairs follow the soft-penalty solve.

The final natural smoothing spline solves:

```text
min_f sum_i w_i * (f(s_i) - target_i)^2 + lambda * integral (f''(s))^2 ds
lambda = 0.12 * sqrt(smoothness / 6)

h_i = s_(i+1) - s_i
Q: each column has [1/h_left, -1/h_left - 1/h_right, 1/h_right]
R: diagonal (h_left+h_right)/3, off-diagonal h_right/6
(R + lambda * Q^T * W^-1 * Q) * m = Q^T * target
f = target - lambda * W^-1 * Q * m
```

Eigen sparse LDLT solves for interior second derivatives `m`; natural boundary second derivatives are zero. Endpoints are fixed exactly, while other constrained positions receive larger fitting weights. Clearance projection, terrain matching, and dense checks are repeated. Output comes from a single C2 planar spline; Z still follows ground layers. This step does not invoke SciPy.

## Cross-Language Validation

```bash
npm run build:cpp
npm run test:cpp
PATH_PLANNER_REQUIRE_CPP=1 npm test
PATH_PLANNER_TEST_CPP=1 npm run test:web
```

CTest covers the native library, negative coordinates, distance fields, exact endpoints, gaps, diagonal support, floors, steps, expansion limits, whole-path smoothing, and invalid inputs. `tests/test_native.py` invokes both implementations on shared maps and parameters: three presets on two public scenes, small-height rematching, overhead-layer isolation, and an optional recorded map.

Endpoint tolerance is 1e-9 m. Public-scene baseline length and minimum clearance are compared to 1e-8; connectivity and failure classification must agree. Optimized paths must add no violations and satisfy corridor and sampling constraints. Across six presets, length differences must remain within 5% of Python and curvature RMS below 60% of the reference. Pointwise equality is not promised: LBFGSpp and SciPy can converge to different feasible curves. Metrics are saved to `artifacts/cpp-parity.json`.

## 3D Viewer

After building C++, start the existing viewer and choose C++ in the engine menu. `GET /api/engines` reports availability. C++ is disabled when absent; direct requests for an unavailable executable return 503. `PATH_PLANNER_CPP` selects another binary. The adapter transfers JSON and map data; it does not solve native algorithms. Explicit C++ requests never fall back to Python.

Both engines use the same endpoint, parameter, curvature, validation, history, and export workflows. The native library and CLI also work without this adapter. Complete 3D edge validation of final curves, robot footprints, dynamic avoidance, speed planning, and robot execution remain outside the current scope; see [Algorithm Design](algorithm.md).
