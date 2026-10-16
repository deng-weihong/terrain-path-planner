# Development and Validation

[简体中文](../development.md) | [English](development.md)

## Environment

Create `.venv` and install `.[web,test]` as described in the README, then run `npm ci`. NumPy and SciPy are core dependencies. HTTP and PNG previews are optional extras.

The npm Python launcher chooses `PATH_PLANNER_PYTHON`, the project `.venv`, or system `python3`, in that order. It sets this project's `src` path for child processes. Importing the algorithm does not modify `sys.path`.

```bash
npm start -- --host 0.0.0.0 --port 5181
npm run dev
```

Vite serves development pages on port 5182 and proxies API calls to port 5181. Update `vite.config.ts` when changing the API development port. The Python API directly serves the built frontend.

## Python Tests

```bash
npm test
```

With the package installed, `python -m unittest discover -s tests -v` is also supported. Coverage includes:

- Search: map gaps, diagonal support, explicit steps, exact endpoints, floor isolation, graph caching, and time limits.
- Smoothing: repeated heading changes, obstacle detours, ground rematching, unrelated layers, support, and visible validation failures.
- API: binary maps, elevation picking, endpoint planning, rerunning the original route, export, invalid input, and concurrent rejection.
- Independence: importing and searching in a separate process with Web dependencies blocked.

Recorded-map regressions are optional. Four tests are explicitly skipped when the local `site-map` fixture is unavailable; all other tests still run. To force public examples only:

```bash
PATH_PLANNER_NO_LOCAL_DATA=1 npm test
```

## Benchmarks

```bash
npm run benchmark
npm run benchmark -- --include-local
npm run benchmark -- --output artifacts/my-benchmark.json
```

The default benchmark runs conservative, balanced, and strong-smoothing presets on both synthetic scenes. It records map hashes, parameters, timing, metrics, and validation. Compare constraint results together with performance metrics; stronger smoothing does not improve every metric automatically.

Benchmark output supports reproduction and comparison, not a performance guarantee. Report hardware, Python/SciPy versions, map size, cold or cached search, and whether preprocessing is included when comparing timing.

## Browser Checks

```bash
npm run build
npx playwright install chromium
npm run test:web
```

The checker starts a temporary local API by default and closes it afterward. To use an existing server:

```bash
PATH_PLANNER_URL=http://127.0.0.1:5181 npm run test:web
```

Optional recorded-map checks:

```bash
PATH_PLANNER_TEST_LOCAL=1 npm run test:web
```

Default checks do not need recorded data. They cover 1440 x 1000, 1280 x 800, and 390 x 844 viewports, checking actual WebGL pixels, orbit controls, point/voxel modes, display elevation scaling, picking, drag and touch cancellation, parameter reruns, endpoint swaps, history, import/export, calculation failures, and map download retries.

Screenshots, exported results, and check records are saved under `artifacts/`. Failure handling also saves a screenshot and page text. For a first browser installation in CI, use `npx playwright install --with-deps chromium`.

Verified [desktop](../images/desktop.png) and [mobile](../images/mobile.png) previews are included in the documentation. Both use public synthetic scenes.

## 3D Rendering Conventions

Z points upward. Every voxel center is decoded from the original little-endian int32 XYZ data. Voxels use spatially tiled `InstancedMesh` geometry. When a distant voxel is smaller than 1.25 pixels, rendering switches to the same complete point set; zooming in restores voxel faces.

Route widths are fixed in screen pixels for readability on large maps. A small display elevation offset reduces overlap with ground; exported coordinates exclude it. Height scaling changes only the displayed scene, not algorithm inputs, the original picked coordinates, or numerical metrics.

Raycasting returns the actual voxel center and elevation. A miss reports an error instead of substituting an intersection with a plane. Map switches, retries, and component disposal release geometry, materials, textures, and WebGL contexts.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Missing SciPy/FastAPI | Use the project `.venv` and install the required extras |
| Creating `.venv` fails because ensurepip is missing | On Debian/Ubuntu, install the matching `python3-venv` package or use an existing interpreter |
| System Python comes from a cross-compilation toolchain | Point `PATH_PLANNER_PYTHON` to a working native interpreter |
| Homepage returns 503 | Run `npm run build` first |
| Custom map is absent | Check the manifest and data-directory environment variable, then restart the API |
| A visible curve fails validation | Inspect new/inherited violations, clearance, ground layers, and sharp turns rather than judging appearance alone |
| Blank WebGL scene | Check the console, WebGL 2 support, and map download state |
