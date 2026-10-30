# Data Formats and API

[简体中文](../data-format.md) | [English](data-format.md)

## Maps

Synthetic datasets are `obstacles` and `zigzag`. An optional recorded map may be placed in `data/local/site-map/`, or `PATH_PLANNER_DATA_DIR` may point to a directory containing `manifest.json`. Local maps are excluded from source archives.

Example layout:

```text
my-map/
  manifest.json
  traversable.bin
  baseline-route.json
```

```json
{
  "id": "my-map",
  "name": "Example map",
  "resolution": 0.1,
  "mapFile": "traversable.bin",
  "routeFile": "baseline-route.json"
}
```

`id` contains lowercase letters, digits, and hyphens, with at most 64 characters, and must not conflict with a built-in scene. Resolution is a positive finite value in meters. Referenced files must stay inside the dataset directory. Maps load at API startup; restart the service after changing them.

`traversable.bin` consists of little-endian signed int32 triples, 12 bytes per voxel: `ix, iy, iz`. Indices describe traversable ground, not obstacles. Centers are `(index + 0.5) * resolution`. Area counts unique XY positions, without counting separate Z layers at the same XY twice.

`mapSha256` is the SHA-256 of the int32 binary. Resolution is not included, so map matching must also consider resolution and provenance.

## Routes

Coordinates are local metric XYZ in the map frame, not latitude and longitude. The minimal format is:

```json
[[1.05, 2.05, 0.05], [1.15, 2.05, 0.05]]
```

Dataset route files also accept `{"pathPoints": [...]}` with either arrays or `{"x": 1.05, "y": 2.05, "z": 0.05}` objects. Browser imports prefer `sourceRoute` in exported results and also accept `pathPoints`, `path`, or `route` arrays; `navigationPath` is never imported as ground. The HTTP `route` parameter uses XYZ arrays.

HTTP input accepts 2-20,000 points and an XY length of 0.1-1,000 m, with finite coordinates. Imported XY points must lie in the map projection; subsequent validation checks the full set of geometric constraints.

## Optimization Parameters

| Field | Default | Range | Meaning |
| --- | --- | --- | --- |
| `smoothness` | 6 | 0.1-60 | Second-derivative energy weight |
| `max_deviation` | 0.8 | 0.05-3 m | Maximum XY displacement at the same reference arc-length parameter |
| `clearance` | 0.39 | 0-1 m | Target projected clearance, with an adaptive floor in narrow regions |
| `control_spacing` | 1.5 | 0.4-4 m | Initial B-spline control-point spacing |
| `navigation_height` | 0.35 | 0.1-1.5 m | Target navigation reference height above ground |
| `elevation_radius` | 1.5 | 0.3-3 m | Elevation smoothing radius along the reference |

## HTTP API

Interactive API documentation: <http://localhost:5181/api/docs>. The API is part of this project, not an external service dependency.

| Method and Path | Purpose |
| --- | --- |
| `GET /api/health` | Health and project identifier |
| `GET /api/engines` | Python and C++ engines with availability |
| `GET /api/datasets` | Available scenes |
| `GET /api/datasets/{id}` | Resolution, bounds, voxel count, sample route, and metrics |
| `GET /api/datasets/{id}/voxels.bin` | Complete XYZ int32 binary |
| `GET /api/datasets/{id}/map.png` | XY projection PNG |
| `POST /api/datasets/{id}/pick` | Endpoint voxel and available elevation layers |
| `POST /api/optimize` | Create an asynchronous calculation job |
| `GET /api/jobs/{id}` | Job phase, status, result, or error |
| `GET /api/jobs/{id}/export` | Export a completed result |

3D picking request:

```json
{"x": 2.05, "y": 3.05, "z": 0.05}
```

The response contains `point` (voxel-center XYZ) and `heights` (all ground elevations at the same XY). An explicit `z` must match an existing layer. If omitted, the nearest sample-route elevation determines the layer. Browser raycasting always sends the actual hit elevation.

Endpoint planning request:

```json
{
  "dataset": "zigzag",
  "engine": "cpp",
  "start": [2.05, 3.05, 0.05],
  "goal": [20.05, 9.05, 0.05],
  "smoothness": 6,
  "max_deviation": 0.8,
  "clearance": 0.39,
  "control_spacing": 1.5,
  "navigation_height": 0.35,
  "elevation_radius": 1.5
}
```

Without `start` and `goal`, the request uses `route` or the scene's sample route. Endpoints must be supplied together and cannot be combined with `route`. Job creation returns 202 and an `id`; an already running calculation returns 409, invalid parameters return 422, and an unknown dataset returns 404.

`engine` defaults to `python` and also accepts `cpp`. A missing or nonexecutable native binary returns 503 without a fallback. The standalone C++ CLI accepts `voxels`, `resolution`, a `route` or endpoints, and nested `options`; see the [C++ JSON protocol](cpp.md).

Polled statuses are `running`, `completed`, or `failed`; execution phases are `planning` or `optimizing`. A failed job releases the worker so another calculation can be submitted.

## Result Fields

| Field | Contents |
| --- | --- |
| `algorithm` | Optimization algorithm version |
| `engine` | Actual `python` or `cpp` execution, added by the API or native JSON adapter |
| `options` | Actual parameters used |
| `dataset` / `mapSha256` / `jobId` | API job provenance |
| `sourceRoute` | Original XYZ vertices from search or import |
| `planning` | Search engine, exact endpoints, time, and expanded nodes; null if search was skipped |
| `reference` | Reference route sampled at the output parameters |
| `resultSchemaVersion` / `pathRole` | `2` / `ground-support` |
| `path` | Ground support route checked by `validation` |
| `navigationPath` | Smoothed elevated reference with the same XY as `path` |
| `navigation` | Target and actual heights, smoothing, pitch change and separate validation |
| `rawCandidate` | Initial B-spline candidate for comparison |
| `baseline` / `optimized` | Metrics before and after optimization |
| `baselineValidation` / `validation` | Reference and final dense checks |
| `solver` | Convergence, iterations, timing, continuity, and correction details |
| `scope` | Implemented checks and unverified scope |

`solver.continuity` describes the ground XY spline; `heightRematchedSamples` counts ground-height rematching. `navigation.heightM` and `smoothingRadiusM` report actual options; `minGroundOffsetM` / `maxGroundOffsetM` report actual reference height relative to the ground route. `navigation.validation.valid` requires both `groundValid` and `heightEnvelopeValid`. Overhead and robot envelope checks remain false. `optimized` still describes the ground route; elevated 3D length and pitch change are reported under `navigation`.

A direct `optimize()` call returns the core algorithm fields. The API adds `sourceRoute`, `planning`, and job provenance.

Results contain more than a point list. Inspect `validation.valid`, new and inherited violations, solver state, dense heading changes, and the verification limits in `scope` before using an output route.
