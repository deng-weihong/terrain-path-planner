import unittest

import numpy as np

from terrain_planner.datasets import load_datasets
from terrain_planner.optimizer import GridMap, Options, TerrainProfile, arc_length, check_path, optimize, resample


class WholePathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.datasets = load_datasets()

    def site_dataset(self):
        if "site-map" not in self.datasets:
            self.skipTest("Optional local site-map fixture is not installed")
        return self.datasets["site-map"]

    def test_global_smoothing_reduces_repeated_heading_changes(self):
        dataset = self.datasets["zigzag"]
        result = optimize(dataset.grid, dataset.route)
        self.assertTrue(result["validation"]["valid"])
        self.assertLess(result["optimized"]["curvatureRms"], result["baseline"]["curvatureRms"] * 0.3)
        self.assertLess(result["optimized"]["turningDeg"], result["baseline"]["turningDeg"] * 0.3)
        np.testing.assert_array_equal(np.array(result["path"])[[0, -1]], dataset.route[[0, -1]])
        self.assertLessEqual(result["validation"]["maxDeviationM"], Options().max_deviation + 1e-6)

    def test_obstacle_route_does_not_cut_unknown_space(self):
        dataset = self.datasets["obstacles"]
        result = optimize(dataset.grid, dataset.route, Options(smoothness=30, max_deviation=1.5))
        self.assertTrue(result["validation"]["valid"], result["validation"])
        self.assertEqual(result["validation"]["outsideCount"], 0)
        self.assertEqual(result["validation"]["diagonalViolationCount"], 0)

    def test_diagonally_touching_cells_are_not_a_corridor(self):
        grid = GridMap(np.array([[0, 0, 0], [1, 1, 0]]))
        path, s = resample(np.array([[.05, .05, .05], [.15, .15, .05]]), .01)
        result = check_path(grid, path, path, s, .8)
        self.assertFalse(result["valid"])
        self.assertGreater(result["diagonalViolationCount"], 0)

    def test_height_layers_are_not_merged_into_xy_support(self):
        grid = GridMap(np.array([[0, 0, 0], [1, 0, 20], [2, 0, 0]]))
        route = np.array([[.05, .05, .05], [.25, .05, .05]])
        path, s = resample(route, .01)
        result = check_path(grid, path, route, s, .8)
        self.assertEqual(result["outsideCount"], 0)
        self.assertGreater(result["heightViolationCount"], 0)
        self.assertFalse(result["valid"])

    def test_dense_validation_detects_gap_between_valid_endpoints(self):
        grid = GridMap(np.array([[0, 0, 0], [2, 0, 0]]))
        route = np.array([[.05, .05, .05], [.25, .05, .05]])
        path, s = resample(route, .01)
        result = check_path(grid, path, route, s, .8)
        self.assertGreater(result["outsideCount"], 0)
        self.assertFalse(result["valid"])

    def test_actual_map_preserves_endpoints_and_rematches_supported_ground(self):
        dataset = self.site_dataset()
        result = optimize(dataset.grid, dataset.route)
        points = np.array(result["path"])
        reference = np.array(result["reference"])
        np.testing.assert_array_equal(points[[0, -1]], dataset.route[[0, -1]])
        rematched = np.abs(points[:, 2] - reference[:, 2]) > 1e-8
        self.assertTrue(rematched.any())
        _, supported = dataset.grid.heights(points[rematched, :2], points[rematched, 2])
        self.assertTrue(supported.all())
        self.assertLessEqual(np.abs(points[:, 2] - reference[:, 2]).max(), .100001)
        self.assertEqual(result["validation"]["newViolationCount"], 0)
        self.assertEqual(result["validation"]["outsideCount"], 0)
        self.assertLessEqual(result["validation"]["sampleSpacingM"], .05)
        self.assertLess(result["optimized"]["curvatureRms"], result["baseline"]["curvatureRms"])
        self.assertLess(result["optimized"]["turningDeg"], result["baseline"]["turningDeg"])
        self.assertEqual(result["validation"]["clearanceViolationCount"], 0)
        self.assertGreaterEqual(result["optimized"]["minClearanceM"], result["baseline"]["minClearanceM"] - 1e-5)
        self.assertEqual(result["validation"]["inheritedViolationCount"],
                         len(result["baselineValidation"]["invalidIndices"]))
        s = np.linspace(0, arc_length(dataset.route)[-1], len(points))
        for span in result["protectedSpans"]:
            core = (s >= span["startM"]) & (s <= span["endM"])
            self.assertLess(np.linalg.norm(points[core] - reference[core], axis=1).max(), 1e-5)
        self.assertFalse(result["scope"]["production3dGraphVerified"])

    def test_reported_unnecessary_double_bend_is_straightened(self):
        dataset = self.site_dataset()
        result = optimize(dataset.grid, dataset.route)
        points = np.asarray(result["path"])
        s = np.linspace(0, arc_length(dataset.route)[-1], len(points))
        local = points[(s >= 34.4) & (s <= 39.4), :2]
        direction = local[-1] - local[0]
        offsets = local - local[0]
        distance = np.abs(offsets[:, 0] * direction[1] - offsets[:, 1] * direction[0]) / np.linalg.norm(direction)
        self.assertLess(distance.max(), .06)
        self.assertGreater(result["shortcut"]["segments"], 0)
        self.assertGreater(result["solver"]["heightRematchedSamples"], 0)
        self.assertLess(result["optimized"]["lengthM"], 176)
        self.assertEqual(result["validation"]["newViolationCount"], 0)

    def test_reported_inside_corner_keeps_the_requested_boundary_distance(self):
        dataset = self.site_dataset()
        result = optimize(dataset.grid, dataset.route)
        points, _ = resample(np.asarray(result["path"]), .01)
        near = np.linalg.norm(points[:, :2] - [-50.6, -80.1], axis=1) < 1.5
        self.assertTrue(near.any())
        clearance, _ = dataset.grid.field(points[near, :2])
        self.assertGreaterEqual(clearance.min(), Options().clearance - 1e-4)
        self.assertEqual(result["validation"]["clearanceViolationCount"], 0)
        self.assertEqual(result["validation"]["newViolationCount"], 0)

    def test_open_ground_shortcut_moves_a_small_height_transition(self):
        x, y = np.meshgrid(np.arange(150), np.arange(100))
        grid = GridMap(np.column_stack([x.ravel(), y.ravel(), (x.ravel() >= 70).astype(int)]))
        xs = np.linspace(1.05, 13.05, 121)
        ys = 5.05 + np.where(xs < 7, .5 * np.sin((xs - 1.05) * np.pi / 2) ** 2, 0)
        route = np.column_stack([xs, ys, .05 + .1 * (xs >= 7)])
        result = optimize(grid, route)
        path = np.asarray(result["path"])
        self.assertTrue(result["validation"]["valid"], result["validation"])
        self.assertLess(np.abs(path[:, 1] - 5.05).max(), 1e-5)
        self.assertGreater(result["solver"]["heightRematchedSamples"], 0)
        self.assertLess(result["optimized"]["turningDeg"], .01)
        np.testing.assert_array_equal(path[[0, -1]], route[[0, -1]])

    def test_shortcut_cannot_use_an_overhead_layer_as_ground(self):
        x, y = np.meshgrid(np.arange(160), np.arange(100))
        upper = (x >= 65) & (x <= 90) & (y < 65)
        grid = GridMap(np.column_stack([x.ravel(), y.ravel(), np.where(upper, 20, 0).ravel()]))
        route, _ = resample(np.array([
            [1.05, 5.05, .05], [5.05, 5.05, .05], [5.05, 7.55, .05],
            [10.55, 7.55, .05], [10.55, 5.05, .05], [14.05, 5.05, .05],
        ]))
        result = optimize(grid, route, Options(max_deviation=2))
        path = np.asarray(result["path"])
        self.assertTrue(result["validation"]["valid"], result["validation"])
        self.assertLess(np.abs(path[:, 2] - .05).max(), 1e-8)
        self.assertGreaterEqual(path[(path[:, 0] >= 6.5) & (path[:, 0] < 9.1), 1].min(), 6.5)

    def test_diagonal_bridge_at_an_unrelated_height_is_rejected(self):
        grid = GridMap(np.array([[0, 0, 0], [1, 1, 0], [0, 1, 1], [1, 0, 1]]))
        route = np.array([[.05, .05, .05], [.15, .15, .05]])
        path, s = resample(route, .01)
        terrain = TerrainProfile(route, .8)
        result = check_path(grid, path, route, s, .8, terrain)
        self.assertFalse(result["valid"])
        self.assertGreater(result["diagonalViolationCount"], 0)

    def test_degenerate_and_invalid_input_is_rejected(self):
        grid = self.datasets["zigzag"].grid
        with self.assertRaises(ValueError):
            optimize(grid, np.array([[0, 0, 0], [0, 0, 0]]))
        with self.assertRaises(ValueError):
            optimize(grid, np.array([[0, 0, 0], [1, float("nan"), 0]]))
        with self.assertRaises(ValueError):
            optimize(grid, self.datasets["zigzag"].route, Options(max_deviation=-1))

    def test_reported_bend_does_not_inherit_reference_tangent_jump(self):
        dataset = self.site_dataset()
        result = optimize(dataset.grid, dataset.route)
        target = np.array([-22.35, -24.05])

        def local_max_turn(points):
            sampled, _ = resample(np.asarray(points), .05)
            direction = np.diff(sampled[:, :2], axis=0)
            heading = np.unwrap(np.arctan2(direction[:, 1], direction[:, 0]))
            turns = np.abs(np.rad2deg(np.diff(heading)))
            near = np.linalg.norm(sampled[1:-1, :2] - target, axis=1) < 1
            return turns[near].max()

        self.assertGreater(local_max_turn(dataset.route), 20)
        self.assertLess(local_max_turn(result["path"]), local_max_turn(dataset.route) * .2)
        self.assertEqual(result["optimized"]["denseSharpTurns"], 0)
        self.assertLess(result["optimized"]["maxHeadingChangeDeg"], 5)
        self.assertGreater(result["solver"]["minimumSpeed"], .05)

    def test_refinement_constraints_cannot_hide_failed_geometry(self):
        grid = GridMap(np.array([[0, 0, 0], [2, 0, 0], [3, 0, 0]]))
        route = np.array([[.05, .05, .05], [.35, .05, .05]])
        result = optimize(grid, route)
        self.assertFalse(result["validation"]["valid"])
        self.assertGreater(result["validation"]["outsideCount"], 0)


if __name__ == "__main__":
    unittest.main()
