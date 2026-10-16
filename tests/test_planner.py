import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from terrain_planner import global_planner as planner
from terrain_planner.datasets import Dataset
from terrain_planner.optimizer import GridMap
from terrain_planner.planner import pick_point, plan_reference
from terrain_planner.planning_types import NavigationGrid


def rectangle(width=14, height=10, z=0.05, x0=0):
    return [((x + x0 + .5) * .1, (y + .5) * .1, z) for x in range(width) for y in range(height)]


def plan(points, start=None, goal=None, **kwargs):
    return planner.plan_path(
        NavigationGrid(.1, points), start=start or points[0], goal=goal or points[-1],
        max_iterations=10000, snap_search_radius_cells=0, **kwargs,
    )


class StandalonePlannerTests(unittest.TestCase):
    def tearDown(self):
        planner._prepared_planner.cache_clear()

    def test_open_grid_retains_ported_shortcut_and_optional_smoothing(self):
        points = rectangle()
        result = plan(points, start=(.15, .15, .05), goal=(1.15, .75, .05), preferred_boundary_clearance_m=0)
        self.assertTrue(result.success, result)
        self.assertGreater(len(result.raw_waypoints), 2)
        self.assertEqual(result.smoothing["shortcut_points"], 2)
        raw = plan(points, enable_path_smoothing=False, exact_endpoints=True)
        self.assertTrue(raw.success)
        self.assertEqual(raw.waypoints, raw.raw_waypoints)
        self.assertFalse(raw.smoothing["applied"])
        np.testing.assert_allclose(raw.waypoints[0], points[0], atol=1e-9)
        np.testing.assert_allclose(raw.waypoints[-1], points[-1], atol=1e-9)

    def test_unknown_gap_and_unsupported_diagonal_are_disconnected(self):
        left, right = rectangle(3, 3), rectangle(3, 3, x0=5)
        result = plan(left + right, start=left[0], goal=right[0], exact_endpoints=True)
        self.assertFalse(result.success)
        self.assertEqual(result.status, "goal_not_on_start_component")
        diagonal = [(.05, .05, .05), (-.05, .05, .05), (-.15, .05, .05),
                    (.15, .15, .05), (.25, .15, .05), (.35, .15, .05)]
        self.assertFalse(plan(diagonal, start=diagonal[0], goal=diagonal[3], exact_endpoints=True).success)

    def test_supported_step_stays_explicit(self):
        points = rectangle(2, 3) + rectangle(2, 3, z=.25, x0=3)
        result = plan(points, exact_endpoints=True, enable_path_smoothing=False)
        self.assertTrue(result.success, result)
        self.assertTrue(any(abs(b[2] - a[2]) > .19 for a, b in zip(result.waypoints, result.waypoints[1:])))

    def test_exact_endpoints_do_not_jump_floors_or_move_boundary_cells(self):
        lower, upper = rectangle(5, 5), rectangle(5, 5, z=.85)
        result = plan(lower + upper, start=lower[0], goal=upper[-1], exact_endpoints=True)
        self.assertFalse(result.success)
        self.assertEqual(result.status, "goal_not_on_start_component")
        wrong = plan(lower, start=(.05, .05, .85), goal=lower[-1], exact_endpoints=True)
        self.assertEqual(wrong.status, "start_not_on_map")
        edge = plan(lower, start=lower[0], goal=lower[-1], exact_endpoints=True)
        self.assertTrue(edge.success)
        np.testing.assert_allclose(edge.snapped_start, lower[0], atol=1e-9)

    def test_physical_boundary_preference_survives_port(self):
        result = plan(rectangle(9, 9), start=(.45, .45, .05), goal=(.55, .45, .05),
                      enable_path_smoothing=False, preferred_boundary_clearance_m=.35)
        self.assertTrue(result.success)
        self.assertEqual(result.minimum_boundary_clearance_m, .35)
        self.assertEqual(result.minimum_boundary_clearance_cells, 3)
        self.assertEqual(result.connectivity_cost, 0)

    def test_graph_reused_for_endpoints_and_invalidated_on_map_edit(self):
        planner._prepared_planner.cache_clear()
        points = rectangle(9, 1)
        first = plan(points, exact_endpoints=True)
        second = plan(points, start=points[-1], goal=points[0], exact_endpoints=True)
        self.assertTrue(first.success and second.success)
        self.assertEqual(planner._prepared_planner.cache_info().hits, 1)
        edited = plan(points[:4] + points[5:], exact_endpoints=True)
        self.assertFalse(edited.success)
        self.assertEqual(planner._prepared_planner.cache_info().misses, 2)

    def test_preparation_is_bounded_by_planning_deadline(self):
        planner._prepared_planner.cache_clear()
        now = [0.0]
        original = planner._TraversablePlanner._build_adjacency

        def delayed(instance):
            now[0] = 2.0
            original(instance)

        with patch.object(planner.time, "monotonic", side_effect=lambda: now[0]), \
             patch.object(planner._TraversablePlanner, "_build_adjacency", delayed):
            result = plan(rectangle(9, 1), maximum_planning_time_ms=1000)
        self.assertEqual(result.status, "time_limit")
        self.assertEqual(result.planning_time_ms, 2000)
        self.assertEqual(planner._prepared_planner.cache_info().currsize, 0)

    def test_pick_reports_height_layers_and_rejects_unknown_cells(self):
        keys = np.array([(x, y, z) for x in range(-2, 6) for y in range(6) for z in (0, 8)])
        dataset = Dataset("test", "test", GridMap(keys), np.array([[-.05, .05, .85], [.45, .45, .85]]), "test")
        picked = pick_point(dataset, -.01, .01)
        np.testing.assert_allclose(picked["point"], [-.05, .05, .85])
        np.testing.assert_allclose(picked["heights"], [.05, .85])
        with self.assertRaisesRegex(ValueError, "没有可通行"):
            pick_point(dataset, 99, 99)
        with self.assertRaises(ValueError):
            pick_point(dataset, float("nan"), 0)
        with self.assertRaisesRegex(ValueError, "距离过近"):
            plan_reference(dataset, (-.05, .05, .85), (-.05, .05, .85), .39)
        with self.assertRaises(ValueError):
            plan_reference(dataset, (-.05, .05, .05), (.45, .45, .85), .39)

    def test_import_and_search_without_web_packages_or_path_injection(self):
        root = Path(__file__).resolve().parents[1] / "src"
        script = '''
import sys
sys.path.insert(0, sys.argv[1])
class BlockWeb:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"fastapi", "uvicorn", "PIL"}:
            raise ImportError("Web dependency is forbidden")
sys.meta_path.insert(0, BlockWeb())
before = list(sys.path)
from terrain_planner.planner import plan_reference
from terrain_planner.global_planner import plan_path
from terrain_planner.planning_types import NavigationGrid
points = [((i+.5)*.1, .05, .05) for i in range(8)]
result = plan_path(NavigationGrid(.1, points), start=points[0], goal=points[-1],
                   max_iterations=10000, snap_search_radius_cells=0, exact_endpoints=True)
assert result.success, result
assert before == sys.path, "Import modified sys.path"
assert not any(name.split(".")[0] in {"fastapi", "uvicorn", "PIL"} for name in sys.modules)
print("standalone planning passed")
'''
        result = subprocess.run([sys.executable, "-E", "-c", script, str(root)], cwd="/tmp",
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("standalone planning passed", result.stdout)
