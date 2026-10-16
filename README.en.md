# Terrain Path Planner

[简体中文](README.md) | [English](README.en.md)

Path search on traversable voxel graphs and whole-path smoothing, with a standalone Python package and an interactive Three.js viewer.

The planner searches a graph that preserves terrain elevation, removes unnecessary detours within the original route corridor, and jointly optimizes the entire curve. The viewer supports manual start and goal selection, elevation layers, parameter adjustment, curvature and clearance comparisons, and result export. It runs independently of robots, ROS, and external application servers.

The current implementation is Python. The C++ version is planned and has not been implemented yet; see [Architecture](docs/en/architecture.md) for its scope and shared data contracts.

![3D map, route comparison, and validation results](docs/images/desktop.png)

## Quick Start

Requirements: Python 3.10+, Node.js 20.19+ (22.12+ recommended), and a browser with WebGL 2 support.

```bash
git clone https://github.com/deng-weihong/terrain-path-planner.git
cd terrain-path-planner
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[web,test]'
npm ci
npm run build
npm start
```

Open <http://localhost:5181>. The default obstacle-detour scene is generated reproducibly and does not require a recorded site map. Chinese is the default documentation language; this file and `docs/en/` provide the English version.

To use the algorithm without the viewer:

```bash
.venv/bin/python -m pip install -e .
.venv/bin/python examples/plan_route.py
```

For frontend development, run `npm start` and `npm run dev` in separate terminals, then open <http://localhost:5182>. The npm commands prefer the project `.venv`; set `PATH_PLANNER_PYTHON` to choose another interpreter.

## Pipeline

```text
Traversable XYZ voxels + start and goal
  -> 3D adjacency / slope and step rules / connected components
  -> Graph search with clearance preferences
  -> Original route and allowed displacement corridor
  -> Farthest-visible-point shortcuts
  -> Global cubic B-spline control-point optimization
  -> Ground elevation matching and clearance correction
  -> Constraint refinement using one C2 planar spline
  -> Dense validation / metrics / export
```

Graph search and final curve validation cover different constraints. Search preserves XYZ adjacency; smoothing primarily optimizes XY and checks ground layers separately. The final curve has not been revalidated edge by edge against the complete 3D graph. Clearance is estimated from the XY projection of traversable ground and does not include the robot footprint. Existing problems in the reference route are counted separately, so `newViolationCount = 0` does not mean that every check passed. See [Algorithm Design](docs/en/algorithm.md).

## Validation

```bash
npm test
npm run benchmark
npm run build
npx playwright install chromium
npm run test:web
```

Browser checks start a temporary local API and verify WebGL pixels, 3D interactions, endpoint planning, import and export, and failure recovery at desktop, laptop, and mobile viewport sizes. Screenshots and metrics are written to `artifacts/`.

## Layout

```text
src/terrain_planner/  Python algorithms, models, and optional viewer API
web/                  Vue + Three.js viewer
tests/                Algorithm, API, and browser checks
examples/             Minimal runnable examples
docs/                 Chinese documentation and English versions in en/
scripts/              Development commands and source packaging
data/                 Data conventions; local maps belong in local/
```

## Documentation

- [Algorithm Design](docs/en/algorithm.md): graph search, corridor constraints, spline objectives, terrain, and validation.
- [Architecture](docs/en/architecture.md): module boundaries, execution paths, and the planned C++ port.
- [Data Formats and API](docs/en/data-format.md): maps, routes, parameters, jobs, and exports.
- [Development and Validation](docs/en/development.md): setup, tests, benchmarks, and 3D rendering.
- [Source Snapshots and Releases](docs/en/releasing.md): archive contents, licensing status, and release steps.
- [Contributing](CONTRIBUTING.en.md) and [Provenance and Dependencies](NOTICE.en.md).

A project license has not been selected. No formally licensed release has been published yet.
