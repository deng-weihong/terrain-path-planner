"""Local voxel coordinates and planning results for the standalone planner."""

from dataclasses import dataclass, field
from math import dist, floor

Point3D = tuple[float, float, float]
GridKey = tuple[int, int, int]


def point_to_grid_key(point: Point3D, resolution: float) -> GridKey:
    size = max(float(resolution), 1e-6)
    return (floor(point[0] / size), floor(point[1] / size), floor(point[2] / size))


def grid_key_to_point(key: GridKey, resolution: float) -> Point3D:
    return ((key[0] + .5) * resolution, (key[1] + .5) * resolution, (key[2] + .5) * resolution)


@dataclass(frozen=True)
class NavigationGrid:
    resolution: float
    traversable_points: list[Point3D]
    metadata: dict = field(default_factory=dict)

    @property
    def traversable_count(self) -> int:
        return len(self.traversable_points)


@dataclass(frozen=True)
class PathResult:
    success: bool
    message: str
    waypoints: list[Point3D]
    snapped_start: Point3D | None = None
    snapped_goal: Point3D | None = None
    iterations: int = 0
    raw_waypoints: list[Point3D] = field(default_factory=list)
    smoothing: dict = field(default_factory=dict)
    total_cost: float = 0.0
    status: str = ""
    planning_time_ms: float = 0.0
    connectivity_cost: float = 0.0
    minimum_boundary_clearance_cells: int = 0
    minimum_boundary_clearance_m: float = 0.0

    @property
    def waypoint_count(self) -> int:
        return len(self.waypoints)

    @property
    def length(self) -> float:
        return sum(dist(left, right) for left, right in zip(self.waypoints, self.waypoints[1:]))
