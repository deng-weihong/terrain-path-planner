import json
import os
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

import numpy as np

from terrain_planner.datasets import load_datasets
from terrain_planner.global_planner import plan_path
from terrain_planner.native import native_binary, run_native
from terrain_planner.optimizer import GridMap, Options, optimize, resample
from terrain_planner.planning_types import NavigationGrid


class NativePlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if native_binary() is None:
            if os.environ.get("PATH_PLANNER_REQUIRE_CPP") == "1":
                raise RuntimeError("C++ tests are required; run npm run build:cpp")
            raise unittest.SkipTest("Optional C++ executable is not built; run npm run build:cpp")
        cls.datasets = load_datasets()

    def native_optimize(self, grid, route, options=Options()):
        return run_native({"resolution": grid.resolution, "voxels": grid.keys.tolist(),
                           "route": np.asarray(route).tolist(), "options": vars(options)})

    def test_search_agrees_on_connectivity_steps_and_floor_separation(self):
        rectangle = lambda x0, z=0: [(x, y, z) for y in range(5) for x in range(x0, x0 + 5)]
        cases = [
            (rectangle(0), [0, 0, 0], [4, 4, 0]),
            (rectangle(-5), [-5, 0, 0], [-1, 4, 0]),
            (rectangle(0) + rectangle(7), [0, 0, 0], [7, 0, 0]),
            (rectangle(0) + rectangle(0, 8), [0, 0, 0], [4, 4, 8]),
            (rectangle(0) + rectangle(6, 2), [0, 0, 0], [10, 4, 2]),
            ([[-2, 0, 0], [-1, 0, 0], [0, 0, 0], [1, 1, 0], [2, 1, 0], [3, 1, 0]], [0, 0, 0], [1, 1, 0]),
        ]
        for keys, a, b in cases:
            with self.subTest(start=a, goal=b):
                points = (np.asarray(keys) + .5) * .1
                start, goal = (np.asarray(a) + .5) * .1, (np.asarray(b) + .5) * .1
                python = plan_path(NavigationGrid(.1, list(map(tuple, points))), start=tuple(start), goal=tuple(goal),
                                   max_iterations=250000, snap_search_radius_cells=0, exact_endpoints=True,
                                   enable_path_smoothing=False, maximum_planning_time_ms=60000)
                native = run_native({"resolution": .1, "voxels": keys, "start": start.tolist(),
                                     "goal": goal.tolist(), "operation": "search"})
                self.assertEqual(native["success"], python.success, native)
                self.assertEqual(native["status"], python.status)
                if python.success:
                    np.testing.assert_allclose(np.asarray(native["path"])[[0, -1]], [start, goal], atol=1e-9, rtol=0)
                    self.assertAlmostEqual(native["planning"]["totalCost"], python.total_cost, places=8)
                    self.assertAlmostEqual(native["planning"]["minimumBoundaryClearanceM"], python.minimum_boundary_clearance_m, places=8)

    def test_synthetic_presets_agree_on_geometry_and_route_quality(self):
        presets = [Options(2, .4, .39, 1), Options(), Options(30, 1.5, .39, 2.5)]
        summary = []
        for name in ("zigzag", "obstacles"):
            dataset = self.datasets[name]
            for options in presets:
                with self.subTest(dataset=name, options=options):
                    python = optimize(dataset.grid, dataset.route, options)
                    native = self.native_optimize(dataset.grid, dataset.route, options)
                    self.assertTrue(native["validation"]["valid"], native["validation"])
                    self.assertEqual(native["validation"]["newViolationCount"], 0)
                    self.assertLessEqual(native["validation"]["maxDeviationM"], options.max_deviation + 1e-6)
                    self.assertLessEqual(native["validation"]["sampleSpacingM"], .05)
                    np.testing.assert_array_equal(np.asarray(native["path"])[[0, -1]], dataset.route[[0, -1]])
                    self.assertLess(abs(native["optimized"]["lengthM"] - python["optimized"]["lengthM"]),
                                    .05 * python["optimized"]["lengthM"])
                    self.assertLess(native["optimized"]["curvatureRms"], native["baseline"]["curvatureRms"] * .6)
                    self.assertAlmostEqual(native["baseline"]["lengthM"], python["baseline"]["lengthM"], places=8)
                    self.assertAlmostEqual(native["baseline"]["minClearanceM"], python["baseline"]["minClearanceM"], places=8)
                    summary.append({"dataset": name, "options": vars(options), "python": python["optimized"],
                                    "cpp": native["optimized"], "validation": native["validation"]})
        output = Path(__file__).resolve().parents[1] / "artifacts" / "cpp-parity.json"
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    def test_small_ground_transition_is_rematched(self):
        x, y = np.meshgrid(np.arange(150), np.arange(100))
        grid = GridMap(np.column_stack([x.ravel(), y.ravel(), (x.ravel() >= 70).astype(int)]))
        xs = np.linspace(1.05, 13.05, 121)
        ys = 5.05 + np.where(xs < 7, .5 * np.sin((xs - 1.05) * np.pi / 2) ** 2, 0)
        route = np.column_stack([xs, ys, .05 + .1 * (xs >= 7)])
        result = self.native_optimize(grid, route)
        self.assertTrue(result["validation"]["valid"], result["validation"])
        self.assertGreater(result["solver"]["heightRematchedSamples"], 0)
        self.assertLess(np.abs(np.asarray(result["path"])[:, 1] - 5.05).max(), 1e-5)

    def test_overhead_layer_is_not_used_as_ground(self):
        x, y = np.meshgrid(np.arange(160), np.arange(100))
        upper = (x >= 65) & (x <= 90) & (y < 65)
        grid = GridMap(np.column_stack([x.ravel(), y.ravel(), np.where(upper, 20, 0).ravel()]))
        route, _ = resample(np.array([[1.05, 5.05, .05], [5.05, 5.05, .05], [5.05, 7.55, .05],
                                      [10.55, 7.55, .05], [10.55, 5.05, .05], [14.05, 5.05, .05]]))
        result = self.native_optimize(grid, route, Options(max_deviation=2))
        path = np.asarray(result["path"])
        self.assertTrue(result["validation"]["valid"], result["validation"])
        self.assertLess(np.abs(path[:, 2] - .05).max(), 1e-8)
        self.assertGreaterEqual(path[(path[:, 0] >= 6.5) & (path[:, 0] < 9.1), 1].min(), 6.5)

    def test_invalid_baseline_and_malformed_requests_are_not_hidden(self):
        result = self.native_optimize(GridMap(np.array([[0, 0, 0], [2, 0, 0], [3, 0, 0]])),
                                      [[.05, .05, .05], [.35, .05, .05]])
        self.assertFalse(result["validation"]["valid"])
        self.assertGreater(result["validation"]["outsideCount"], 0)
        for request in ({}, {"resolution": .1, "voxels": [[0.5, 0, 0]]},
                        {"resolution": .1, "voxels": [[2147483648, 0, 0]]}):
            with self.subTest(request=request), self.assertRaises(ValueError):
                run_native(request)
        completed = subprocess.run([str(native_binary())], input="{broken", capture_output=True, text=True, timeout=5)
        self.assertEqual(completed.returncode, 2)
        self.assertIn("error", json.loads(completed.stderr))

    def test_optional_recorded_map_retains_supported_terrain_and_clearance(self):
        if "site-map" not in self.datasets:
            self.skipTest("Optional local site-map fixture is not installed")
        dataset = self.datasets["site-map"]
        result = self.native_optimize(dataset.grid, dataset.route)
        self.assertEqual(result["validation"]["newViolationCount"], 0, result["validation"])
        self.assertEqual(result["validation"]["clearanceViolationCount"], 0)
        self.assertGreater(result["solver"]["heightRematchedSamples"], 0)
        self.assertLess(result["optimized"]["curvatureRms"], result["baseline"]["curvatureRms"])
        self.assertLess(result["optimized"]["maxHeadingChangeDeg"], 5)
        self.assertGreater(result["solver"]["minimumSpeed"], .05)


class NativeAvailabilityTests(unittest.TestCase):
    def test_explicit_missing_binary_does_not_fall_back_to_python(self):
        with patch.dict(os.environ, {"PATH_PLANNER_CPP": str(Path("build/missing-native-binary").resolve())}):
            self.assertIsNone(native_binary())
            with self.assertRaisesRegex(ValueError, "unavailable"):
                run_native({})
