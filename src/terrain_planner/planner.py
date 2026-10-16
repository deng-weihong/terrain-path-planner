from __future__ import annotations

import numpy as np

from .datasets import Dataset
from .global_planner import plan_path
from .planning_types import NavigationGrid


def pick_point(dataset: Dataset, x: float, y: float, z: float | None = None) -> dict:
    if not np.isfinite([x, y]).all():
        raise ValueError("坐标必须是有限数值")
    key = tuple(np.floor(np.array([x, y]) / dataset.grid.resolution).astype(int))
    heights = dataset.grid.columns.get(key)
    if not heights:
        raise ValueError("所选位置没有可通行体素，请重新选择")
    xy = (np.array(key) + .5) * dataset.grid.resolution
    if z is None:
        nearest = np.argmin(np.linalg.norm(dataset.route[:, :2] - xy, axis=1))
        z = min(heights, key=lambda height: abs(height - dataset.route[nearest, 2]))
    elif not np.isfinite(z) or not any(abs(height - z) < 1e-6 for height in heights):
        raise ValueError("所选体素高程不存在，请重新选择")
    else:
        z = min(heights, key=lambda height: abs(height - z))
    return {"point": [float(xy[0]), float(xy[1]), float(z)], "heights": heights}


def plan_reference(dataset: Dataset, start: tuple[float, float, float],
                   goal: tuple[float, float, float], clearance: float) -> tuple[np.ndarray, dict]:
    for name, point in (("起点", start), ("终点", goal)):
        if not np.isfinite(point).all():
            raise ValueError(f"{name}坐标必须是有限数值")
        heights = dataset.grid.columns.get(tuple(np.floor(np.asarray(point[:2]) / dataset.grid.resolution).astype(int)), ())
        if not any(abs(z - point[2]) < 1e-6 for z in heights):
            raise ValueError(f"{name}没有对应的地面高程，请重新选择")
    if np.linalg.norm(np.asarray(start[:2]) - goal[:2]) < dataset.grid.resolution - 1e-6:
        raise ValueError("起点和终点距离过近，请重新选择")
    points = ((dataset.grid.keys.astype(float) + .5) * dataset.grid.resolution).tolist()
    navigation = NavigationGrid(dataset.grid.resolution, [tuple(point) for point in points], {})
    result = plan_path(
        navigation, start=start, goal=goal, max_iterations=250000,
        snap_search_radius_cells=0, enable_path_smoothing=False,
        maximum_planning_time_ms=60000, preferred_boundary_clearance_m=clearance,
        exact_endpoints=True,
    )
    if not result.success:
        raise ValueError(result.message)
    if not np.allclose(result.snapped_start, start, atol=1e-6, rtol=0) or not np.allclose(result.snapped_goal, goal, atol=1e-6, rtol=0):
        raise ValueError("所选高程或位置不在同一可通行连通域，请重新选择")
    return np.asarray(result.waypoints), {
        "engine": "traversable-graph-v1",
        "start": list(start), "goal": list(goal),
        "iterations": result.iterations, "timeMs": round(result.planning_time_ms, 1),
        "waypointCount": result.waypoint_count,
    }
