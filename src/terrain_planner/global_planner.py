"""Standalone global planning on the traversable XYZ voxel graph.

Ported from algorithms/path_planner.py in the workspace Server on 2026-10-16.
The graph and search implementation is preserved; all types and grid helpers
are local to this package. No external service or workspace path is needed.
"""

from __future__ import annotations

import heapq
import time
from bisect import bisect_left
from collections import defaultdict, deque
from dataclasses import dataclass
from functools import lru_cache
from math import acos, ceil, hypot, inf, isfinite, pi, sqrt, tan

from .planning_types import GridKey, NavigationGrid, PathResult, Point3D, grid_key_to_point, point_to_grid_key

_EPSILON = 1.0e-9
_INVALID_INDEX = -1
_XY_DIRECTIONS: tuple[tuple[int, int], ...] = (
    (-1, -1), (-1, 0), (-1, 1), (0, -1),
    (0, 1), (1, -1), (1, 0), (1, 1),
)


@dataclass(frozen=True)
class _PlannerOptions:
    resolution: float
    ground_neighbor_radius: float = 0.16
    ground_height_tolerance: float = 0.04
    maximum_slope_deg: float = 25.0
    minimum_step_height: float = 0.05
    maximum_step_height: float = 0.20
    maximum_step_horizontal_gap: float = 0.30
    step_intensity_threshold: float = 0.25
    boundary_height_tolerance: float = 0.20
    # Match the SDK planner: boundary clearance is a soft lexicographic
    # preference.  A zero physical value falls back to two map cells.
    preferred_boundary_clearance_m: float = 0.0
    preferred_boundary_clearance_cells: int = 2
    prioritize_connectivity: bool = True
    distance_cost_weight: float = 1.0
    terrain_cost_weight: float = 0.50
    vertical_cost_weight: float = 1.0
    turn_cost_weight: float = 0.05
    boundary_cost_weight: float = 0.25
    step_transition_cost: float = 0.20
    enable_path_smoothing: bool = True
    smoothing_sample_spacing: float = 0.05
    smoothing_support_radius: float = 0.08
    smoothing_vertical_tolerance: float = 0.08
    shortcut_max_distance: float = 5.0
    corner_radius: float = 0.30
    minimum_corner_angle_deg: float = 5.0
    projection_radius: float = 0.75
    projection_max_height: float = 1.0
    projection_vertical_weight: float = 0.25
    projection_boundary_weight: float = 0.10
    minimum_component_size: int = 3
    maximum_expanded_nodes: int = 250000
    maximum_planning_time_ms: float = 1000.0
    infer_geometric_steps: bool = True


@dataclass(frozen=True)
class _Point:
    x: float
    y: float
    z: float
    terrain_cost: float = 0.0


@dataclass(frozen=True, slots=True)
class _Edge:
    target: int
    distance: float
    step_transition: bool


@dataclass(frozen=True)
class _ConnectivityProfile:
    penalty: float = 0.0
    worst_penalty: float = 0.0
    minimum_clearance_cells: int = 0
    minimum_clearance_m: float = 0.0


class _PlanningTimeLimit(Exception):
    pass


def _squared_distance(first: _Point, second: _Point) -> float:
    return (
        (first.x - second.x) ** 2
        + (first.y - second.y) ** 2
        + (first.z - second.z) ** 2
    )


def _point_distance(first: _Point, second: _Point) -> float:
    return sqrt(_squared_distance(first, second))


def _lerp(first: _Point, second: _Point, ratio: float) -> _Point:
    return _Point(
        first.x + ratio * (second.x - first.x),
        first.y + ratio * (second.y - first.y),
        first.z + ratio * (second.z - first.z),
        first.terrain_cost + ratio * (second.terrain_cost - first.terrain_cost),
    )


class _TraversablePlanner:
    def __init__(self, options: _PlannerOptions, points: list[_Point]) -> None:
        self.options = options
        self.maximum_slope_tangent = tan(options.maximum_slope_deg * pi / 180.0)
        self.boundary_height_cells = int((options.boundary_height_tolerance + _EPSILON) / options.resolution)
        self.preparation_deadline = time.monotonic() + options.maximum_planning_time_ms / 1000.0
        self.points: list[_Point] = []
        self.keys: list[GridKey] = []
        self.columns: dict[tuple[int, int], list[int]] = defaultdict(list)
        self.column_heights: dict[tuple[int, int], list[int]] = {}
        self.adjacency: list[list[_Edge]] = []
        self.boundary_penalties: list[float] = []
        self.boundary_clearance_m: list[float] = []
        self.boundary_clearance_cells: list[int] = []
        self.connectivity_penalties: list[float] = []
        self.component_ids: list[int] = []
        self.component_sizes: list[int] = []
        self._set_points(points)

    def _check_preparation_time(self, index: int = 0) -> None:
        if index & 0xFF == 0 and time.monotonic() >= self.preparation_deadline:
            raise _PlanningTimeLimit

    def _grid_key(self, point: _Point) -> GridKey:
        return point_to_grid_key((point.x, point.y, point.z), self.options.resolution)

    def _set_points(self, input_points: list[_Point]) -> None:
        unique: dict[GridKey, int] = {}
        for input_index, point in enumerate(input_points):
            self._check_preparation_time(input_index)
            if not all(isfinite(value) for value in (point.x, point.y, point.z, point.terrain_cost)):
                continue
            key = self._grid_key(point)
            existing = unique.get(key)
            if existing is None:
                index = len(self.points)
                unique[key] = index
                # Keep the exact voxel centre.  This is also what the binary
                # map format represents and avoids accumulated floating error.
                center = grid_key_to_point(key, self.options.resolution)
                self.points.append(_Point(*center, max(0.0, point.terrain_cost)))
                self.keys.append(key)
            else:
                stored = self.points[existing]
                self.points[existing] = _Point(
                    stored.x, stored.y, stored.z,
                    max(stored.terrain_cost, max(0.0, point.terrain_cost)),
                )

        for index, key in enumerate(self.keys):
            self._check_preparation_time(index)
            self.columns[(key[0], key[1])].append(index)
        self.column_heights = {
            key: sorted(self.keys[index][2] for index in indices)
            for key, indices in self.columns.items()
        }
        self._build_boundary_penalties()
        self._build_connectivity_penalties()
        self._build_adjacency()
        self._build_components()

    def _build_boundary_penalties(self) -> None:
        self.boundary_penalties = [1.0] * len(self.points)
        for index, key in enumerate(self.keys):
            self._check_preparation_time(index)
            supported = 0
            for dx, dy in _XY_DIRECTIONS:
                if self._has_surface(key[0] + dx, key[1] + dy, key[2]):
                    supported += 1
            self.boundary_penalties[index] = (8.0 - supported) / 8.0

    def _has_surface(self, x: int, y: int, z: int) -> bool:
        heights = self.column_heights.get((x, y))
        if not heights:
            return False
        position = bisect_left(heights, z - self.boundary_height_cells)
        return position < len(heights) and heights[position] <= z + self.boundary_height_cells

    def _preferred_boundary_clearance_m(self) -> float:
        if self.options.preferred_boundary_clearance_m > _EPSILON:
            return self.options.preferred_boundary_clearance_m
        return max(0.0, float(self.options.preferred_boundary_clearance_cells)) * self.options.resolution

    def _build_connectivity_penalties(self) -> None:
        preferred = self._preferred_boundary_clearance_m()
        self.boundary_clearance_cells = [0] * len(self.points)
        self.boundary_clearance_m = [0.0] * len(self.points)
        self.connectivity_penalties = [0.0] * len(self.points)
        if not self.points or preferred <= _EPSILON:
            return

        # Search only as far as the requested clearance.  A missing column is
        # a boundary when it has no surface at a similar height.
        search_radius = preferred + 0.5 * self.options.resolution
        search_cells = int(ceil(search_radius / self.options.resolution))
        offsets = sorted(
            (max(0.0, distance - 0.5 * self.options.resolution), dx, dy)
            for dx in range(-search_cells, search_cells + 1)
            for dy in range(-search_cells, search_cells + 1)
            if (dx or dy)
            and (distance := self.options.resolution * hypot(dx, dy)) <= search_radius + _EPSILON
        )
        for index, key in enumerate(self.keys):
            self._check_preparation_time(index)
            clearance = preferred
            # Offsets are nearest first, so the first unsupported column is
            # the exact minimum; interior cells need no trigonometry here.
            for boundary_distance, dx, dy in offsets:
                if boundary_distance >= preferred:
                    break
                if not self._has_surface(key[0] + dx, key[1] + dy, key[2]):
                    clearance = boundary_distance
                    break

            self.boundary_clearance_m[index] = min(preferred, clearance)
            # Keep the legacy cell diagnostic while making metres authoritative.
            cell_clearance = max(0.0, self.boundary_clearance_m[index] - _EPSILON)
            self.boundary_clearance_cells[index] = int(
                cell_clearance / self.options.resolution
            )
            self.connectivity_penalties[index] = max(
                0.0, preferred - self.boundary_clearance_m[index]
            )

    def _has_lateral_support(self, index: int) -> bool:
        if index < 0 or index >= len(self.keys):
            return False
        return self.boundary_penalties[index] <= 0.75

    def _ordinary_diagonal_has_support(self, source: int, target: int) -> bool:
        source_key = self.keys[source]
        target_key = self.keys[target]
        if source_key[0] == target_key[0] or source_key[1] == target_key[1]:
            return True

        x_distance = abs(target_key[0] - source_key[0]) * self.options.resolution
        y_distance = abs(target_key[1] - source_key[1]) * self.options.resolution
        if max(x_distance, y_distance) > self.options.ground_neighbor_radius + _EPSILON:
            return False
        x_limit = int(
            (self.options.ground_height_tolerance + self.maximum_slope_tangent * x_distance + _EPSILON)
            / self.options.resolution
        )
        y_limit = int(
            (self.options.ground_height_tolerance + self.maximum_slope_tangent * y_distance + _EPSILON)
            / self.options.resolution
        )

        def has_bridge(column_key: tuple[int, int], first_limit: int, second_limit: int) -> bool:
            heights = self.column_heights.get(column_key, ())
            lower = max(source_key[2] - first_limit, target_key[2] - second_limit)
            upper = min(source_key[2] + first_limit, target_key[2] + second_limit)
            position = bisect_left(heights, lower)
            return position < len(heights) and heights[position] <= upper

        return has_bridge((target_key[0], source_key[1]), x_limit, y_limit) and has_bridge(
            (source_key[0], target_key[1]), y_limit, x_limit
        )

    def _build_adjacency(self) -> None:
        self.adjacency = [[] for _ in self.points]
        search_cells = int(ceil(self.options.maximum_step_horizontal_gap / self.options.resolution))
        maximum_distance = max(self.options.ground_neighbor_radius, self.options.maximum_step_horizontal_gap)
        offsets = [
            (dx, dy, horizontal, self.options.ground_height_tolerance + self.maximum_slope_tangent * horizontal)
            for dx in range(search_cells + 1)
            for dy in range(-search_cells, search_cells + 1)
            if (dx > 0 or dy > 0)
            and (horizontal := self.options.resolution * hypot(dx, dy)) <= maximum_distance + _EPSILON
        ]
        # Visit each pair of occupied columns once. Distances depend only on
        # grid offsets, while height and support still decide every edge.
        for column_index, ((x, y), sources) in enumerate(self.columns.items()):
            self._check_preparation_time(column_index)
            for dx, dy, horizontal, normal_limit in offsets:
                targets = self.columns.get((x + dx, y + dy), ())
                for source in sources:
                    first = self.points[source]
                    for target in targets:
                        second = self.points[target]
                        height = abs(second.z - first.z)
                        ordinary = (
                            horizontal <= self.options.ground_neighbor_radius + _EPSILON
                            and height <= normal_limit + _EPSILON
                        )
                        if not ordinary and not (
                            horizontal <= self.options.maximum_step_horizontal_gap + _EPSILON
                            and self.options.minimum_step_height - _EPSILON
                            <= height <= self.options.maximum_step_height + _EPSILON
                            and (
                                first.terrain_cost >= self.options.step_intensity_threshold
                                or second.terrain_cost >= self.options.step_intensity_threshold
                                or (
                                    self.options.infer_geometric_steps
                                    and self._has_lateral_support(source)
                                    and self._has_lateral_support(target)
                                )
                            )
                        ):
                            continue
                        if ordinary and not self._ordinary_diagonal_has_support(source, target):
                            continue
                        distance = hypot(horizontal, height)
                        self.adjacency[source].append(_Edge(target, distance, not ordinary))
                        self.adjacency[target].append(_Edge(source, distance, not ordinary))

    def _build_components(self) -> None:
        self.component_ids = [_INVALID_INDEX] * len(self.points)
        self.component_sizes = []
        for seed in range(len(self.points)):
            self._check_preparation_time(seed)
            if self.component_ids[seed] != _INVALID_INDEX:
                continue
            component = len(self.component_sizes)
            self.component_sizes.append(0)
            frontier: deque[int] = deque([seed])
            self.component_ids[seed] = component
            while frontier:
                self._check_preparation_time(self.component_sizes[component])
                current = frontier.pop()
                self.component_sizes[component] += 1
                for edge in self.adjacency[current]:
                    if self.component_ids[edge.target] == _INVALID_INDEX:
                        self.component_ids[edge.target] = component
                        frontier.append(edge.target)

    def _edge_between(self, source: int, target: int) -> _Edge | None:
        return next((edge for edge in self.adjacency[source] if edge.target == target), None)

    def _diagonal_transition_supported(self, source: int, target: int, component: int) -> bool:
        source_key = self.keys[source]
        target_key = self.keys[target]
        if abs(target_key[0] - source_key[0]) != 1 or abs(target_key[1] - source_key[1]) != 1:
            return True

        def has_bridge(column_key: tuple[int, int]) -> bool:
            for candidate in self.columns.get(column_key, ()):
                if self.component_ids[candidate] != component:
                    continue
                first = self._edge_between(source, candidate)
                second = self._edge_between(candidate, target)
                if (
                    first is not None
                    and second is not None
                    and not first.step_transition
                    and not second.step_transition
                ):
                    return True
            return False

        return has_bridge((target_key[0], source_key[1])) and has_bridge((source_key[0], target_key[1]))

    def _support_for_sample(
        self,
        sample: _Point,
        component: int,
        previous_support: int,
        allow_step: bool,
    ) -> int | None:
        center = self._grid_key(sample)
        best: int | None = None
        best_score = inf
        for candidate in self.columns.get((center[0], center[1]), ()):
            if self.component_ids[candidate] != component:
                continue
            point = self.points[candidate]
            horizontal = hypot(point.x - sample.x, point.y - sample.y)
            vertical = abs(point.z - sample.z)
            if (
                horizontal > self.options.smoothing_support_radius + _EPSILON
                or vertical > self.options.smoothing_vertical_tolerance + _EPSILON
            ):
                continue
            if candidate != previous_support:
                edge = self._edge_between(previous_support, candidate)
                if edge is None or (edge.step_transition and not allow_step):
                    continue
                if (
                    not edge.step_transition
                    and not self._diagonal_transition_supported(
                        previous_support, candidate, component
                    )
                ):
                    continue
            score = horizontal * horizontal + 0.25 * vertical * vertical
            if score < best_score:
                best_score = score
                best = candidate
        return best

    def _project_geometry(
        self,
        geometry: list[_Point],
        component: int,
        start_support: int,
        end_support: int,
        allow_step: bool,
        collect: bool = False,
        support_indices: list[int] | None = None,
    ) -> tuple[bool, list[_Point]]:
        if (
            not geometry
            or start_support < 0
            or end_support < 0
            or self.component_ids[start_support] != component
            or self.component_ids[end_support] != component
        ):
            return False, []
        projected = [self.points[start_support]] if collect else []
        if support_indices is not None:
            support_indices.clear()
            support_indices.append(start_support)
        previous_support = start_support
        for sample in geometry[1:]:
            support = self._support_for_sample(sample, component, previous_support, allow_step)
            if support is None:
                return False, []
            previous_support = support
            if support_indices is not None:
                support_indices.append(previous_support)
            if collect:
                point = self.points[support]
                projected.append(_Point(sample.x, sample.y, point.z, point.terrain_cost))
        if previous_support != end_support:
            return False, []
        if collect:
            projected[-1] = self.points[end_support]
        return True, projected

    def _linear_geometry(self, start: _Point, end: _Point) -> list[_Point]:
        distance = _point_distance(start, end)
        if distance < _EPSILON:
            return [start]
        intervals = max(1, int(ceil(distance / self.options.smoothing_sample_spacing)))
        return [_lerp(start, end, index / intervals) for index in range(intervals + 1)]

    def _polyline_geometry(self, indices: list[int]) -> list[_Point]:
        if not indices:
            return []
        geometry = [self.points[indices[0]]]
        for index in range(1, len(indices)):
            segment = self._linear_geometry(self.points[indices[index - 1]], self.points[indices[index]])
            geometry.extend(segment[1:])
        return geometry

    def _rounded_geometry(self, indices: list[int]) -> tuple[list[_Point], int]:
        if len(indices) < 3 or self.options.corner_radius <= _EPSILON:
            return self._polyline_geometry(indices), 0
        geometry = [self.points[indices[0]]]
        rounded_corners = 0

        def append_line(end: _Point) -> None:
            geometry.extend(self._linear_geometry(geometry[-1], end)[1:])

        minimum_angle = self.options.minimum_corner_angle_deg * pi / 180.0
        for index in range(1, len(indices) - 1):
            previous = self.points[indices[index - 1]]
            corner = self.points[indices[index]]
            next_point = self.points[indices[index + 1]]
            incoming_x = corner.x - previous.x
            incoming_y = corner.y - previous.y
            outgoing_x = next_point.x - corner.x
            outgoing_y = next_point.y - corner.y
            incoming_length = hypot(incoming_x, incoming_y)
            outgoing_length = hypot(outgoing_x, outgoing_y)
            if incoming_length < _EPSILON or outgoing_length < _EPSILON:
                append_line(corner)
                continue
            cosine = max(
                -1.0,
                min(
                    1.0,
                    (incoming_x * outgoing_x + incoming_y * outgoing_y)
                    / (incoming_length * outgoing_length),
                ),
            )
            angle = acos(cosine)
            if angle < minimum_angle or angle > pi - minimum_angle:
                append_line(corner)
                continue
            trim = min(self.options.corner_radius, 0.35 * incoming_length, 0.35 * outgoing_length)
            if trim < self.options.smoothing_sample_spacing:
                append_line(corner)
                continue
            incoming_ratio = trim / incoming_length
            outgoing_ratio = trim / outgoing_length
            entry = _Point(
                corner.x - incoming_ratio * incoming_x,
                corner.y - incoming_ratio * incoming_y,
                corner.z - incoming_ratio * (corner.z - previous.z),
                corner.terrain_cost,
            )
            exit_point = _Point(
                corner.x + outgoing_ratio * outgoing_x,
                corner.y + outgoing_ratio * outgoing_y,
                corner.z + outgoing_ratio * (next_point.z - corner.z),
                corner.terrain_cost,
            )
            append_line(entry)
            curve_length = _point_distance(entry, corner) + _point_distance(corner, exit_point)
            intervals = max(2, int(ceil(curve_length / self.options.smoothing_sample_spacing)))
            for sample in range(1, intervals + 1):
                ratio = sample / intervals
                inverse = 1.0 - ratio
                geometry.append(_Point(
                    inverse * inverse * entry.x + 2.0 * inverse * ratio * corner.x + ratio * ratio * exit_point.x,
                    inverse * inverse * entry.y + 2.0 * inverse * ratio * corner.y + ratio * ratio * exit_point.y,
                    inverse * inverse * entry.z + 2.0 * inverse * ratio * corner.z + ratio * ratio * exit_point.z,
                    corner.terrain_cost,
                ))
            rounded_corners += 1
        append_line(self.points[indices[-1]])
        return geometry, rounded_corners

    def _shortcut_path(self, raw_indices: list[int], component: int) -> list[int]:
        if len(raw_indices) < 3:
            return list(range(len(raw_indices)))
        step_prefix = [0] * len(raw_indices)
        for index in range(1, len(raw_indices)):
            edge = self._edge_between(raw_indices[index - 1], raw_indices[index])
            step_prefix[index] = step_prefix[index - 1] + int(edge is not None and edge.step_transition)
        positions = [0]
        anchor = 0
        while anchor + 1 < len(raw_indices):
            best = anchor + 1
            for candidate in range(len(raw_indices) - 1, anchor + 1, -1):
                if step_prefix[candidate] != step_prefix[anchor]:
                    continue
                if (
                    _point_distance(
                        self.points[raw_indices[anchor]],
                        self.points[raw_indices[candidate]],
                    )
                    > self.options.shortcut_max_distance + _EPSILON
                ):
                    continue
                valid, _ = self._project_geometry(
                    self._linear_geometry(self.points[raw_indices[anchor]], self.points[raw_indices[candidate]]),
                    component,
                    raw_indices[anchor],
                    raw_indices[candidate],
                    False,
                )
                if valid:
                    best = candidate
                    break
            positions.append(best)
            anchor = best

        if not self.options.prioritize_connectivity or len(positions) >= len(raw_indices):
            return positions

        # Validate the complete shortcut chain, not just each segment.  A
        # chain of individually valid lines can still spend longer near a
        # narrow boundary than the original A* support sequence.
        baseline = self._connectivity_profile(raw_indices)
        shortcut_supports: list[int] = []
        for segment in range(len(positions) - 1):
            begin = raw_indices[positions[segment]]
            end = raw_indices[positions[segment + 1]]
            supports: list[int] = []
            valid, _ = self._project_geometry(
                self._linear_geometry(self.points[begin], self.points[end]),
                component,
                begin,
                end,
                False,
                support_indices=supports,
            )
            if not valid:
                return list(range(len(raw_indices)))
            for support in supports:
                if not shortcut_supports or shortcut_supports[-1] != support:
                    shortcut_supports.append(support)
        if not self._connectivity_profile_not_worse(
            self._connectivity_profile(shortcut_supports), baseline
        ):
            return list(range(len(raw_indices)))
        return positions

    def _process_path(self, raw_indices: list[int], component: int) -> tuple[list[_Point], dict]:
        stats = {
            "raw_points": len(raw_indices),
            "shortcut_points": len(raw_indices),
            "output_points": len(raw_indices),
            "rounded_corners": 0,
            "fallback_runs": 0,
            "applied": False,
        }
        if not self.options.enable_path_smoothing or len(raw_indices) < 2:
            return [self.points[index] for index in raw_indices], stats

        shortcut_positions = self._shortcut_path(raw_indices, component)
        stats["shortcut_points"] = len(shortcut_positions)
        baseline = self._connectivity_profile(raw_indices)
        candidate_supports: list[int] = []

        def append_supports(supports: list[int]) -> None:
            for support in supports:
                if not candidate_supports or candidate_supports[-1] != support:
                    candidate_supports.append(support)

        def append_support(support: int) -> None:
            if not candidate_supports or candidate_supports[-1] != support:
                candidate_supports.append(support)

        def segment_has_step(begin: int, end: int) -> bool:
            return any(
                (edge := self._edge_between(raw_indices[index], raw_indices[index + 1])) is not None
                and edge.step_transition
                for index in range(begin, end)
            )

        result: list[_Point] = []

        def append_point(point: _Point) -> None:
            if not result or _squared_distance(result[-1], point) >= _EPSILON:
                result.append(point)

        def append_points(points: list[_Point]) -> None:
            if not points:
                return
            begin = 1 if result and _squared_distance(result[-1], points[0]) < _EPSILON else 0
            result.extend(points[begin:])

        def process_run(begin: int, end: int) -> None:
            run_indices = [raw_indices[shortcut_positions[index]] for index in range(begin, end + 1)]
            rounded, rounded_count = self._rounded_geometry(run_indices)
            supports: list[int] = []
            valid, projected = self._project_geometry(
                rounded,
                component,
                run_indices[0],
                run_indices[-1],
                False,
                collect=True,
                support_indices=supports,
            )
            if valid:
                stats["rounded_corners"] += rounded_count
                append_supports(supports)
                append_points(projected)
                return
            stats["fallback_runs"] += 1
            polyline = self._polyline_geometry(run_indices)
            supports.clear()
            valid, projected = self._project_geometry(
                polyline,
                component,
                run_indices[0],
                run_indices[-1],
                False,
                collect=True,
                support_indices=supports,
            )
            if valid:
                append_supports(supports)
                append_points(projected)
                return
            raw_begin = shortcut_positions[begin]
            raw_end = shortcut_positions[end]
            raw_supports = raw_indices[raw_begin : raw_end + 1]
            append_supports(raw_supports)
            append_points([self.points[raw_indices[index]] for index in range(raw_begin, raw_end + 1)])

        run_begin = 0
        for segment in range(len(shortcut_positions) - 1):
            if not segment_has_step(shortcut_positions[segment], shortcut_positions[segment + 1]):
                continue
            process_run(run_begin, segment)
            transition = raw_indices[shortcut_positions[segment + 1]]
            append_point(self.points[transition])
            append_support(transition)
            run_begin = segment + 1
        process_run(run_begin, len(shortcut_positions) - 1)

        if not result or not self._connectivity_profile_not_worse(
            self._connectivity_profile(candidate_supports), baseline
        ):
            result = [self.points[index] for index in raw_indices]
            stats["fallback_runs"] += 1
            candidate_supports = list(raw_indices)
            # Match the SDK diagnostics: once the complete smoothed candidate
            # is rejected, report the raw path as the effective result rather
            # than leaving shortcut/corner counts from the discarded attempt.
            stats["shortcut_points"] = len(raw_indices)
            stats["rounded_corners"] = 0
            stats["output_points"] = len(raw_indices)
            stats["applied"] = False
            return result, stats
        stats["output_points"] = len(result)
        stats["applied"] = (
            stats["shortcut_points"] < stats["raw_points"]
            or stats["rounded_corners"] > 0
            or stats["output_points"] != stats["raw_points"]
        )
        return result, stats

    def _project(self, query: Point3D, required_component: int | None) -> int | None:
        best: int | None = None
        best_score = inf
        for index, point in enumerate(self.points):
            component = self.component_ids[index]
            if (
                component == _INVALID_INDEX
                or self.component_sizes[component] < self.options.minimum_component_size
                or (required_component is not None and component != required_component)
            ):
                continue
            horizontal = hypot(point.x - query[0], point.y - query[1])
            vertical = abs(point.z - query[2])
            if (
                horizontal > self.options.projection_radius + _EPSILON
                or vertical > self.options.projection_max_height + _EPSILON
            ):
                continue
            score = (
                horizontal
                + self.options.projection_vertical_weight * vertical
                + self.options.projection_boundary_weight * self.boundary_penalties[index]
            )
            if score < best_score:
                best_score = score
                best = index
        return best

    def _turn_cost(self, parent: int, current: int, next_index: int) -> float:
        if parent in (current, _INVALID_INDEX):
            return 0.0
        first = self.points[current]
        previous = self.points[parent]
        next_point = self.points[next_index]
        first_x = first.x - previous.x
        first_y = first.y - previous.y
        second_x = next_point.x - first.x
        second_y = next_point.y - first.y
        first_length = hypot(first_x, first_y)
        second_length = hypot(second_x, second_y)
        if first_length < _EPSILON or second_length < _EPSILON:
            return 0.0
        cosine = max(-1.0, min(1.0, (first_x * second_x + first_y * second_y) / (first_length * second_length)))
        return acos(cosine)

    def _heuristic(self, source: int, goal: int) -> float:
        return self.options.distance_cost_weight * _point_distance(self.points[source], self.points[goal])

    def _connectivity_profile(self, indices: list[int]) -> _ConnectivityProfile:
        preferred = self._preferred_boundary_clearance_m()
        minimum_cells = int(
            max(0.0, preferred - _EPSILON) / self.options.resolution
        )
        minimum_m = preferred
        penalty = 0.0
        worst_penalty = 0.0
        for index in indices:
            if (
                index < 0
                or index >= len(self.connectivity_penalties)
                or index >= len(self.boundary_clearance_cells)
                or index >= len(self.boundary_clearance_m)
            ):
                continue
            node_penalty = self.connectivity_penalties[index]
            penalty += node_penalty
            worst_penalty = max(worst_penalty, node_penalty)
            minimum_cells = min(minimum_cells, self.boundary_clearance_cells[index])
            minimum_m = min(minimum_m, self.boundary_clearance_m[index])
        if not indices:
            minimum_cells = 0
            minimum_m = 0.0
        return _ConnectivityProfile(
            penalty=penalty,
            worst_penalty=worst_penalty,
            minimum_clearance_cells=minimum_cells,
            minimum_clearance_m=minimum_m,
        )

    def _connectivity_profile_not_worse(
        self,
        candidate: _ConnectivityProfile,
        baseline: _ConnectivityProfile,
    ) -> bool:
        if not self.options.prioritize_connectivity:
            return True
        if abs(candidate.penalty - baseline.penalty) > _EPSILON:
            return candidate.penalty < baseline.penalty
        if abs(candidate.worst_penalty - baseline.worst_penalty) > _EPSILON:
            return candidate.worst_penalty < baseline.worst_penalty
        return candidate.minimum_clearance_m + _EPSILON >= baseline.minimum_clearance_m

    def plan(self, start: Point3D, goal: Point3D, *, started: float | None = None) -> PathResult:
        started = time.monotonic() if started is None else started

        def check_time() -> None:
            if (time.monotonic() - started) * 1000.0 >= self.options.maximum_planning_time_ms:
                raise _PlanningTimeLimit

        def finish(status: str, **kwargs: object) -> PathResult:
            kwargs.setdefault("status", status)
            kwargs.setdefault("message", _STATUS_MESSAGES[status])
            kwargs.setdefault("planning_time_ms", (time.monotonic() - started) * 1000.0)
            return PathResult(**kwargs)  # type: ignore[arg-type]

        if not self.points:
            return finish("empty_map", success=False, waypoints=[])
        check_time()
        if not all(isfinite(value) for value in (*start, *goal)):
            return finish("start_not_on_map", success=False, waypoints=[])
        start_index = self._project(start, None)
        if start_index is None:
            return finish("start_not_on_map", success=False, waypoints=[])
        start_component = self.component_ids[start_index]
        goal_index = self._project(goal, start_component)
        check_time()
        projected_start = (self.points[start_index].x, self.points[start_index].y, self.points[start_index].z)
        if goal_index is None:
            return finish(
                "goal_not_on_start_component",
                success=False,
                waypoints=[],
                snapped_start=projected_start,
            )
        projected_goal = (self.points[goal_index].x, self.points[goal_index].y, self.points[goal_index].z)
        if start_index == goal_index:
            profile = self._connectivity_profile([start_index])
            return finish(
                "success",
                success=True,
                message="起点和终点位于同一可通行体素",
                waypoints=[projected_start],
                snapped_start=projected_start,
                snapped_goal=projected_goal,
                raw_waypoints=[projected_start],
                smoothing={
                    "raw_points": 1,
                    "shortcut_points": 1,
                    "output_points": 1,
                    "rounded_corners": 0,
                    "fallback_runs": 0,
                    "applied": False,
                },
                connectivity_cost=profile.penalty,
                minimum_boundary_clearance_cells=profile.minimum_clearance_cells,
                minimum_boundary_clearance_m=profile.minimum_clearance_m,
            )

        connectivity_costs = [inf] * len(self.points)
        physical_costs = [inf] * len(self.points)
        parents = [_INVALID_INDEX] * len(self.points)
        closed = [False] * len(self.points)
        connectivity_costs[start_index] = 0.0
        physical_costs[start_index] = 0.0
        parents[start_index] = start_index
        # Queue order is lexicographic: connectivity first, physical cost
        # second.  The heuristic only estimates the physical component.
        frontier: list[tuple[float, float, float, float, int]] = [
            (0.0, self._heuristic(start_index, goal_index), 0.0, 0.0, start_index)
        ]
        terminal_status = "no_path"
        expanded = 0
        while frontier:
            (
                _connectivity_f,
                _physical_f,
                current_connectivity,
                current_physical,
                current,
            ) = heapq.heappop(frontier)
            if (
                closed[current]
                or current_connectivity != connectivity_costs[current]
                or current_physical > physical_costs[current] + _EPSILON
            ):
                continue
            closed[current] = True
            expanded += 1
            if current == goal_index:
                terminal_status = "success"
                break
            if expanded >= self.options.maximum_expanded_nodes:
                terminal_status = "expansion_limit"
                break
            if expanded & 0xFF == 0 and (time.monotonic() - started) * 1000.0 >= self.options.maximum_planning_time_ms:
                terminal_status = "time_limit"
                break
            for edge in self.adjacency[current]:
                if closed[edge.target]:
                    continue
                terrain_cost = 0.5 * (self.points[current].terrain_cost + self.points[edge.target].terrain_cost)
                boundary_cost = 0.5 * (self.boundary_penalties[current] + self.boundary_penalties[edge.target])
                vertical = abs(self.points[edge.target].z - self.points[current].z)
                transition_cost = (
                    self.options.distance_cost_weight * edge.distance
                    + self.options.terrain_cost_weight * terrain_cost
                    + self.options.vertical_cost_weight * vertical
                    + self.options.boundary_cost_weight * boundary_cost
                    + self.options.turn_cost_weight * self._turn_cost(parents[current], current, edge.target)
                    + (self.options.step_transition_cost if edge.step_transition else 0.0)
                )
                connectivity_increment = (
                    0.5
                    * (
                        self.connectivity_penalties[current]
                        + self.connectivity_penalties[edge.target]
                    )
                    if self.options.prioritize_connectivity
                    else 0.0
                )
                candidate_connectivity = (
                    connectivity_costs[current] + connectivity_increment
                )
                candidate_physical = physical_costs[current] + transition_cost
                improves = (
                    candidate_connectivity + _EPSILON
                    < connectivity_costs[edge.target]
                    or (
                        abs(
                            candidate_connectivity
                            - connectivity_costs[edge.target]
                        )
                        <= _EPSILON
                        and candidate_physical + _EPSILON
                        < physical_costs[edge.target]
                    )
                )
                if not improves:
                    continue
                connectivity_costs[edge.target] = candidate_connectivity
                physical_costs[edge.target] = candidate_physical
                parents[edge.target] = current
                heapq.heappush(
                    frontier,
                    (
                        candidate_connectivity,
                        candidate_physical + self._heuristic(edge.target, goal_index),
                        candidate_connectivity,
                        candidate_physical,
                        edge.target,
                    ),
                )

        if terminal_status != "success":
            return finish(
                terminal_status,
                success=False,
                waypoints=[],
                snapped_start=projected_start,
                snapped_goal=projected_goal,
                iterations=expanded,
                total_cost=0.0,
            )

        indices: list[int] = []
        current = goal_index
        while True:
            indices.append(current)
            if current == start_index:
                break
            parent = parents[current]
            if parent == _INVALID_INDEX:
                return finish("no_path", success=False, waypoints=[], iterations=expanded)
            current = parent
        indices.reverse()
        raw = [
            (self.points[index].x, self.points[index].y, self.points[index].z)
            for index in indices
        ]
        processed, smoothing = self._process_path(indices, start_component)
        check_time()
        path = [(point.x, point.y, point.z) for point in processed]
        profile = self._connectivity_profile(indices)
        return finish(
            "success",
            success=True,
            message=f"A* 完成，扩展 {expanded:,} 个节点",
            waypoints=path,
            snapped_start=projected_start,
            snapped_goal=projected_goal,
            iterations=expanded,
            raw_waypoints=raw,
            smoothing=smoothing,
            total_cost=physical_costs[goal_index],
            connectivity_cost=profile.penalty,
            minimum_boundary_clearance_cells=profile.minimum_clearance_cells,
            minimum_boundary_clearance_m=profile.minimum_clearance_m,
        )


_STATUS_MESSAGES = {
    "success": "path found",
    "empty_map": "导航栅格没有可通行体素",
    "start_not_on_map": "起点附近没有可通行体素",
    "goal_not_on_start_component": "终点不在起点所在的可通行连通分量",
    "no_path": "没有找到可通行路径",
    "expansion_limit": "A* 超过最大迭代次数",
    "time_limit": "地图预处理或路径搜索达到时间上限，请重试或缩小地图范围",
}


@lru_cache(maxsize=1)
def _prepared_planner(options: _PlannerOptions, points: tuple[Point3D, ...]) -> _TraversablePlanner:
    # Reuse one immutable graph across endpoint changes, keyed by map contents
    # and settings so an edited grid can never reuse stale connectivity.
    return _TraversablePlanner(options, [_Point(*point) for point in points])


def plan_path(
    grid: NavigationGrid,
    *,
    start: Point3D,
    goal: Point3D,
    max_iterations: int,
    snap_search_radius_cells: int,
    enable_path_smoothing: bool = True,
    maximum_planning_time_ms: float = 1000.0,
    preferred_boundary_clearance_m: float = 0.39,
    exact_endpoints: bool = False,
) -> PathResult:
    """Plan one segment using the same graph and smoothing rules as the SDK."""

    started = time.monotonic()
    resolution = max(float(grid.resolution), 1.0e-6)
    options = _PlannerOptions(
        resolution=resolution,
        terrain_cost_weight=0.0,
        preferred_boundary_clearance_m=max(0.0, float(preferred_boundary_clearance_m)),
        enable_path_smoothing=bool(enable_path_smoothing),
        projection_radius=1e-6 if exact_endpoints else max(resolution, max(0, int(snap_search_radius_cells)) * resolution),
        projection_max_height=1e-6 if exact_endpoints else 1.0,
        maximum_expanded_nodes=max(1, int(max_iterations)),
        maximum_planning_time_ms=max(1.0, float(maximum_planning_time_ms)),
    )
    try:
        planner = _prepared_planner(options, tuple(grid.traversable_points))
        return planner.plan(start, goal, started=started)
    except _PlanningTimeLimit:
        return PathResult(
            success=False,
            message=_STATUS_MESSAGES["time_limit"],
            status="time_limit",
            waypoints=[],
            planning_time_ms=(time.monotonic() - started) * 1000.0,
        )
