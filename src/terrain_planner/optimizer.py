from __future__ import annotations

from dataclasses import asdict, dataclass
from time import perf_counter

import numpy as np
from scipy.interpolate import BSpline, make_smoothing_spline
from scipy.ndimage import distance_transform_edt, maximum_filter1d, minimum_filter1d
from scipy.optimize import minimize
from scipy.sparse.linalg import lsqr
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class Options:
    smoothness: float = 6.0
    max_deviation: float = 0.8
    clearance: float = 0.39
    control_spacing: float = 1.5


def arc_length(points: np.ndarray) -> np.ndarray:
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]


def resample(points: np.ndarray, spacing: float = 0.05) -> tuple[np.ndarray, np.ndarray]:
    s = arc_length(points)
    keep = np.r_[True, np.diff(s) > 1e-9]
    s, points = s[keep], points[keep]
    if len(points) < 2 or s[-1] < 1e-6:
        raise ValueError("The route must contain two distinct XY positions")
    query = np.linspace(0.0, s[-1], max(2, int(np.ceil(s[-1] / spacing)) + 1))
    return np.column_stack([np.interp(query, s, points[:, k]) for k in range(3)]), query


def interpolate_reference(points: np.ndarray, query: np.ndarray) -> np.ndarray:
    s = arc_length(points)
    keep = np.r_[True, np.diff(s) > 1e-9]
    return np.column_stack([np.interp(query, s[keep], points[keep, k]) for k in range(3)])


class GridMap:
    """XY distance field with separate height columns; no flattening of Z layers."""

    def __init__(self, keys: np.ndarray, resolution: float = 0.1):
        self.keys = np.asarray(keys, dtype=np.int32)
        self.resolution = resolution
        self.origin = self.keys[:, :2].min(axis=0) - 3
        shape = self.keys[:, :2].max(axis=0) - self.origin + 4
        self.mask = np.zeros((int(shape[1]), int(shape[0])), dtype=bool)
        local = self.keys[:, :2] - self.origin
        self.mask[local[:, 1], local[:, 0]] = True
        inside = distance_transform_edt(self.mask)
        outside = distance_transform_edt(~self.mask)
        self.distance = np.where(self.mask, inside - 0.5, 0.5 - outside) * resolution
        self.columns: dict[tuple[int, int], list[float]] = {}
        for x, y, z in self.keys:
            self.columns.setdefault((int(x), int(y)), []).append((int(z) + 0.5) * resolution)
        self.columns = {key: sorted(set(zs)) for key, zs in self.columns.items()}

    def field(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # Bilinear values and their exact derivatives keep L-BFGS gradients consistent.
        cell = xy / self.resolution - self.origin - 0.5
        raw_i = np.floor(cell).astype(int)
        i = np.column_stack([
            np.clip(raw_i[:, 0], 0, self.mask.shape[1] - 2),
            np.clip(raw_i[:, 1], 0, self.mask.shape[0] - 2),
        ])
        f = np.clip(cell - i, 0, 1)
        x, y = i.T
        a, b = self.distance[y, x], self.distance[y, x + 1]
        c, d = self.distance[y + 1, x], self.distance[y + 1, x + 1]
        fx, fy = f.T
        value = (1 - fy) * ((1 - fx) * a + fx * b) + fy * ((1 - fx) * c + fx * d)
        gradient = np.column_stack([
            (1 - fy) * (b - a) + fy * (d - c),
            (1 - fx) * (c - a) + fx * (d - b),
        ]) / self.resolution
        return value, gradient

    def heights(self, xy: np.ndarray, expected_z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        keys = np.floor(xy / self.resolution).astype(int)
        zs = np.full(len(xy), np.nan)
        supported = np.zeros(len(xy), dtype=bool)
        for n, (key, z) in enumerate(zip(keys, expected_z)):
            choices = self.columns.get(tuple(key), ())
            if choices:
                nearest = min(choices, key=lambda candidate: abs(candidate - z))
                zs[n] = nearest
                supported[n] = abs(nearest - z) <= 0.080001
        return zs, supported


def protected_weight(reference: np.ndarray, s: np.ndarray) -> tuple[np.ndarray, list[dict]]:
    source_s = arc_length(reference)
    diffs = np.diff(reference, axis=0)
    step_indices = np.flatnonzero(
        (np.abs(diffs[:, 2]) > 0.150001)
        | ((np.linalg.norm(diffs[:, :2], axis=1) > 0.260001) & (np.abs(diffs[:, 2]) > 0.080001))
    )
    weight = np.ones(len(s))
    spans = []
    for i in step_indices:
        start, end = source_s[i] - 0.2, source_s[i + 1] + 0.2
        spans.append({"startM": float(max(0, start)), "endM": float(min(source_s[-1], end))})
        distance = np.maximum(start - s, s - end).clip(0)
        t = (distance / 2.0).clip(0, 1)
        weight = np.minimum(weight, t * t * t * (10 - 15 * t + 6 * t * t))
    weight[[0, -1]] = 0
    return weight, spans


class TerrainProfile:
    """Allow nearby ground transitions to move, without opening unrelated Z layers."""

    def __init__(self, reference: np.ndarray, max_deviation: float):
        points, self.s = resample(reference, 0.025)
        half_window = int(np.ceil(max_deviation / (self.s[1] - self.s[0])))
        self.lower = minimum_filter1d(points[:, 2], 2 * half_window + 1, mode="nearest")
        self.upper = maximum_filter1d(points[:, 2], 2 * half_window + 1, mode="nearest")

    def contains(self, z: np.ndarray, s: np.ndarray) -> np.ndarray:
        return ((z >= np.interp(s, self.s, self.lower) - 0.080001)
                & (z <= np.interp(s, self.s, self.upper) + 0.080001))

    def project(self, grid: GridMap, path: np.ndarray, expected: np.ndarray,
                s: np.ndarray) -> None:
        z, supported = grid.heights(path[:, :2], expected[:, 2])
        moved = np.linalg.norm(path[:, :2] - expected[:, :2], axis=1) > 1e-8
        replace = ~supported & moved & self.contains(z, s)
        path[:, 2] = np.where(replace, z, expected[:, 2])


class ClearanceProfile:
    """Keep the target where possible, with gradual relaxation near narrow anchors."""

    def __init__(self, grid: GridMap, reference: np.ndarray, target: float):
        points, self.s = resample(reference, 0.025)
        distance, _ = grid.field(points[:, :2])
        self.target = target
        required = np.maximum(0, np.minimum(target, distance))
        ramp = .25 * self.s
        forward = np.minimum.accumulate(required - ramp) + ramp
        backward = np.minimum.accumulate((required + ramp)[::-1])[::-1] - ramp
        self.required = np.maximum(0, np.minimum(forward, backward))

    def at(self, s: np.ndarray) -> np.ndarray:
        return np.interp(s, self.s, self.required)


def check_path(grid: GridMap, path: np.ndarray, reference: np.ndarray,
               reference_s: np.ndarray, max_deviation: float,
               terrain: TerrainProfile | None = None, clearance: ClearanceProfile | None = None) -> dict:
    expected = interpolate_reference(reference, reference_s)
    keys = np.floor(path[:, :2] / grid.resolution).astype(int)
    occupied = np.array([tuple(key) in grid.columns for key in keys])
    zs, height_ok = grid.heights(path[:, :2], path[:, 2])
    if terrain is None:
        height_ok &= np.abs(path[:, 2] - expected[:, 2]) < 1e-8
    else:
        height_ok &= terrain.contains(path[:, 2], reference_s)
    height_ok[[0, -1]] &= np.abs(path[[0, -1], 2] - expected[[0, -1], 2]) < 1e-8
    deviation = np.linalg.norm(path[:, :2] - expected[:, :2], axis=1)
    deviation_ok = deviation <= max_deviation + 1e-6
    distance, _ = grid.field(path[:, :2])
    original_distance, _ = grid.field(expected[:, :2])
    requested_clearance = clearance.at(reference_s) if clearance is not None else np.zeros(len(path))
    required_clearance = np.maximum(0, np.minimum(requested_clearance, original_distance))
    clearance_ok = distance >= required_clearance - 1e-6
    diagonal_ok = np.ones(len(path), dtype=bool)
    for i in range(1, len(path)):
        previous, current = keys[i - 1], keys[i]
        delta = np.abs(current - previous)
        if delta.max() > 1:
            diagonal_ok[i] = False
        elif np.all(delta == 1):
            # Reject diagonal corner cutting, including crossings of different floors.
            for key in ((previous[0], current[1]), (current[0], previous[1])):
                choices = grid.columns.get(key, ())
                bridge_tolerance = 0.100001 if terrain is not None else 0.080001
                if not any(abs(z - path[i, 2]) <= bridge_tolerance
                           and abs(z - path[i - 1, 2]) <= bridge_tolerance
                           and (terrain is None or terrain.contains(
                               np.array([z]), reference_s[i:i + 1])[0]) for z in choices):
                    diagonal_ok[i] = False
    height_transition_ok = np.ones(len(path), dtype=bool)
    dz = np.abs(np.diff(zs))
    # New vertical transitions must remain at the original terrain transition.
    for i in np.flatnonzero(dz > 0.150001) + 1:
        close = np.linalg.norm(path[i - 1:i + 1, :2] - expected[i - 1:i + 1, :2], axis=1)
        if close.max() > 1e-6 or dz[i - 1] > 0.200001:
            height_transition_ok[i] = False
    invalid = ~occupied | ~height_ok | ~deviation_ok | ~diagonal_ok | ~height_transition_ok | ~clearance_ok
    return {
        "valid": bool(not invalid.any()),
        "outsideCount": int((~occupied).sum()),
        "heightViolationCount": int((occupied & ~height_ok).sum()),
        "diagonalViolationCount": int((~diagonal_ok).sum()),
        "transitionViolationCount": int((~height_transition_ok).sum()),
        "deviationViolationCount": int((~deviation_ok).sum()),
        "clearanceViolationCount": int((~clearance_ok).sum()),
        "clearanceFloorM": float(required_clearance.min()),
        "clearanceTargetM": float(clearance.target if clearance is not None else 0),
        "invalidIndices": np.flatnonzero(invalid).tolist(),
        "maxDeviationM": float(deviation.max()),
        "sampleCount": len(path),
        "sampleSpacingM": float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).max()),
        "_z": zs,
    }


def project_clearance(grid: GridMap, xy: np.ndarray, required: np.ndarray,
                      movable: np.ndarray) -> np.ndarray:
    projected = xy.copy()
    for _ in range(4):
        distance, gradient = grid.field(projected)
        norm2 = np.sum(gradient ** 2, axis=1)
        active = (distance < required + .005) & (norm2 > 1e-8) & movable
        if not active.any():
            break
        step = np.minimum(.1, np.maximum(0, required - distance + .005))
        projected[active] += gradient[active] * (step[active] / norm2[active])[:, None]
    return projected


def straighten_reference(grid: GridMap, reference: np.ndarray, s: np.ndarray,
                         options: Options, terrain: TerrainProfile,
                         clearance: ClearanceProfile) -> tuple[np.ndarray, dict]:
    expected = interpolate_reference(reference, s)
    output = expected.copy()
    protected, _ = protected_weight(reference, s)
    fixed = protected < 1e-10
    baseline = check_path(grid, expected, reference, s, options.max_deviation, terrain)
    inherited = set(baseline["invalidIndices"])
    fixed[baseline["invalidIndices"]] = True
    fixed[[0, -1]] = True
    boundaries = np.flatnonzero(fixed)
    required_clearance = clearance.at(s)
    shortcuts = 0
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        if end - start < 2:
            continue
        stride = max(1, int(round(0.3 / (s[1] - s[0]))))
        nodes = np.unique(np.r_[np.arange(start, end, stride), end])
        cursor = 0
        while cursor < len(nodes) - 1:
            lo = nodes[cursor]
            chosen = cursor + 1
            for candidate_index in range(len(nodes) - 1, cursor + 1, -1):
                hi = nodes[candidate_index]
                local_s = s[lo:hi + 1]
                fraction = (local_s - local_s[0]) / (local_s[-1] - local_s[0])
                chord = expected[lo:hi + 1].copy()
                chord[:, :2] = (expected[lo, :2] * (1 - fraction[:, None])
                               + expected[hi, :2] * fraction[:, None])
                if np.linalg.norm(chord[:, :2] - expected[lo:hi + 1, :2], axis=1).max() > options.max_deviation * .9:
                    continue
                terrain.project(grid, chord, expected[lo:hi + 1], local_s)
                checked = check_path(grid, chord, reference, local_s, options.max_deviation, terrain)
                # An inherited endpoint issue may stay in place, never inside a shortcut.
                if any(i not in (0, len(chord) - 1) or lo + i not in inherited
                       for i in checked["invalidIndices"]):
                    continue
                distance, _ = grid.field(chord[:, :2])
                required = required_clearance[lo:hi + 1]
                if np.any(distance < required - 1e-6):
                    continue
                chosen = candidate_index
                output[lo:hi + 1] = chord
                if np.linalg.norm(chord[:, :2] - expected[lo:hi + 1, :2], axis=1).max() > 0.001:
                    shortcuts += 1
                break
            cursor = chosen
    return output, {
        "segments": shortcuts,
        "lengthSavedM": float(arc_length(expected)[-1] - arc_length(output)[-1]),
        "path": output.tolist(),
    }


def metrics(grid: GridMap, path: np.ndarray, target_clearance: float) -> dict:
    # Fixed-distance sampling makes curvature comparisons independent of source density.
    sampled, s = resample(path, 0.2)
    dxy = np.diff(sampled[:, :2], axis=0)
    lengths = np.linalg.norm(dxy, axis=1)
    angles = np.unwrap(np.arctan2(dxy[:, 1], dxy[:, 0]))
    turns = np.diff(angles)
    curvature = turns / np.maximum((lengths[:-1] + lengths[1:]) / 2, 1e-6)
    dense, _ = resample(path, 0.05)
    distance, _ = grid.field(dense[:, :2])
    dense_direction = np.diff(dense[:, :2], axis=0)
    dense_angles = np.unwrap(np.arctan2(dense_direction[:, 1], dense_direction[:, 0]))
    dense_turns = np.abs(np.rad2deg(np.diff(dense_angles)))
    return {
        "lengthM": float(arc_length(path)[-1]),
        "length3dM": float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
        "turningDeg": float(np.rad2deg(np.abs(turns).sum())),
        "curvatureRms": float(np.sqrt(np.mean(curvature ** 2))) if len(curvature) else 0.0,
        "curvatureP95": float(np.percentile(np.abs(curvature), 95)) if len(curvature) else 0.0,
        "sharpTurns": int((np.abs(np.rad2deg(turns)) > 5).sum()),
        "maxHeadingChangeDeg": float(dense_turns.max()) if len(dense_turns) else 0.0,
        "denseSharpTurns": int((dense_turns > 5).sum()),
        "sharpTurnPoints": dense[1:-1][dense_turns > 5].tolist(),
        "minClearanceM": float(max(0, distance.min())),
        "clearanceP05M": float(max(0, np.percentile(distance, 5))),
        "belowClearancePercent": float(100 * np.mean(distance < target_clearance)),
        "profile": {
            "s": np.round(s[1:-1], 3).tolist(),
            "curvature": np.round(np.abs(curvature), 5).tolist(),
        },
    }


def continuous_refinement(grid: GridMap, candidate: np.ndarray, source_s: np.ndarray,
                          reference: np.ndarray, options: Options,
                          terrain: TerrainProfile, clearance: ClearanceProfile) -> tuple[np.ndarray, np.ndarray, dict]:
    # A taper against a polyline inherits its tangent jumps. Finish with a single
    # global C2 curve, moving failed-sample constraints into the fit itself.
    s = np.linspace(0.0, source_s[-1], max(5, int(np.ceil(source_s[-1] / 0.025)) + 1))
    ref = interpolate_reference(reference, s)
    baseline_distance, _ = grid.field(ref[:, :2])
    required_clearance = np.maximum(0, np.minimum(clearance.at(s), baseline_distance))
    target = np.column_stack([np.interp(s, source_s, candidate[:, dim]) for dim in range(2)])
    weight = np.ones(len(s))
    protected, _ = protected_weight(reference, s)
    fixed = protected < 1e-10
    base_check = check_path(grid, ref, reference, s, options.max_deviation, terrain, clearance)
    fixed[base_check["invalidIndices"]] = True
    fixed[[0, -1]] = True
    target[fixed] = ref[fixed, :2]
    weight[fixed] = 1e10
    lam = 0.12 * np.sqrt(options.smoothness / 6.0)
    checked = {}
    output = ref.copy()
    for iteration in range(8):
        curves = [make_smoothing_spline(s, target[:, dim], w=weight, lam=lam) for dim in range(2)]
        output[:, :2] = np.column_stack([curve(s) for curve in curves])
        terrain.project(grid, output, ref, s)
        output[[0, -1]] = reference[[0, -1]]
        checked = check_path(grid, output, reference, s, options.max_deviation, terrain, clearance)
        unchanged = np.linalg.norm(output - ref, axis=1) < 1e-8
        inherited = set(i for i in base_check["invalidIndices"] if unchanged[i])
        failures = [i for i in checked["invalidIndices"] if i not in inherited]
        if not failures or iteration == 7:
            break
        distance, _ = grid.field(output[:, :2])
        boundary_failure = distance < required_clearance - 1e-6
        if boundary_failure.any():
            projected = project_clearance(grid, output[:, :2], required_clearance, ~fixed)
            affected = maximum_filter1d(boundary_failure, 9, mode="nearest") & ~fixed
            target[affected] = projected[affected]
            weight[affected] = np.minimum(1e8, weight[affected] * 10)
        for index in failures:
            lower, upper = max(0, index - 1), min(len(s), index + 2)
            displacement = output[lower:upper, :2] - ref[lower:upper, :2]
            norm = np.linalg.norm(displacement, axis=1)
            if norm[index - lower] > options.max_deviation + 1e-6:
                # Correct a corridor overshoot onto its inner boundary; pulling
                # every active sample all the way back creates needless waves.
                scale = np.minimum(1, options.max_deviation * .96 / np.maximum(norm, 1e-9))
                target[lower:upper] = ref[lower:upper, :2] + displacement * scale[:, None]
                weight[lower:upper] = np.minimum(1e8, weight[lower:upper] * 10)
            elif not boundary_failure[index]:
                target[lower:upper] = ref[lower:upper, :2]
                weight[lower:upper] = np.minimum(1e12, weight[lower:upper] * 1000)
    checked.pop("_z")
    checked["inheritedViolationCount"] = sum(i in inherited for i in checked["invalidIndices"])
    checked["newViolationCount"] = len(checked["invalidIndices"]) - checked["inheritedViolationCount"]
    checked["noNewViolations"] = checked["newViolationCount"] == 0
    base_check.pop("_z")
    derivatives = np.column_stack([curve(s, nu=1) for curve in curves])
    return output, ref, {
        "validation": checked, "baselineValidation": base_check, "referenceS": s,
        "iterations": iteration + 1, "minimumSpeed": float(np.linalg.norm(derivatives, axis=1).min()),
        "constraintSamples": int((weight > 1).sum()),
    }


def optimize(grid: GridMap, reference: np.ndarray, options: Options = Options()) -> dict:
    started = perf_counter()
    reference = np.asarray(reference, dtype=float)
    if reference.ndim != 2 or reference.shape[1] != 3 or not np.isfinite(reference).all():
        raise ValueError("Route must be a finite Nx3 array")
    if not (0.1 <= options.smoothness <= 60 and 0.05 <= options.max_deviation <= 3
            and 0 <= options.clearance <= 1 and 0.4 <= options.control_spacing <= 4):
        raise ValueError("Optimization parameters are outside the supported limits")
    _, uniform_s = resample(reference, 0.025)
    s = np.unique(np.r_[uniform_s, arc_length(reference)])
    ref = interpolate_reference(reference, s)
    terrain = TerrainProfile(reference, options.max_deviation)
    clearance = ClearanceProfile(grid, reference, options.clearance)
    baseline_distance, _ = grid.field(ref[:, :2])
    required_clearance = np.maximum(0, np.minimum(clearance.at(s), baseline_distance))
    base_check = check_path(grid, ref, reference, s, options.max_deviation, terrain, clearance)
    base_check.pop("_z")
    total = s[-1]
    straightened, shortcut = straighten_reference(grid, reference, uniform_s, options, terrain, clearance)
    sample_s = np.linspace(0, total, max(12, int(np.ceil(total / 0.15)) + 1))
    target = np.column_stack([np.interp(sample_s, uniform_s, straightened[:, dim]) for dim in range(2)])
    original_target = interpolate_reference(reference, sample_s)[:, :2]
    sample_distance, _ = grid.field(original_target)
    sample_clearance = np.maximum(0, np.minimum(clearance.at(sample_s) + .02, sample_distance))
    segments = max(4, int(np.ceil(total / options.control_spacing)))
    knots = np.r_[np.zeros(4), np.linspace(0, total, segments + 1)[1:-1], np.full(4, total)]
    n_control = len(knots) - 4
    basis = BSpline.design_matrix(sample_s, knots, 3).tocsr()
    derivative_basis = BSpline(knots, np.eye(n_control), 3)(sample_s, nu=2)
    from scipy.sparse import csr_matrix
    second = csr_matrix(derivative_basis)
    controls = np.column_stack([lsqr(basis, target[:, dim], atol=1e-8, btol=1e-8)[0] for dim in range(2)])
    controls[0], controls[-1] = reference[0, :2], reference[-1, :2]
    control_s = np.array([knots[i + 1:i + 4].mean() for i in range(n_control)])
    control_ref = interpolate_reference(reference, control_s)[:, :2]
    radius = options.max_deviation
    bounds = [(float(p - radius), float(p + radius)) for p in control_ref[1:-1].ravel()]
    # A narrow endpoint needs an initially supported departure/arrival tangent;
    # correcting only its first few dense samples creates a short, sharp hook.
    endpoint_distance, _ = grid.field(reference[[0, -1], :2])
    tangent_s = min(.5, total / 3)
    for endpoint, control_index, query, knot_span in (
        (0, 1, tangent_s, knots[4] - knots[3]),
        (-1, n_control - 2, total - tangent_s, knots[-4] - knots[-5]),
    ):
        distance = endpoint_distance[0 if endpoint == 0 else 1]
        if distance >= options.clearance:
            continue
        direction = interpolate_reference(reference, np.array([query]))[0, :2] - reference[endpoint, :2]
        norm = np.linalg.norm(direction)
        if norm < 1e-9:
            continue
        tangent_control = reference[endpoint, :2] + direction / norm * knot_span / 3
        offset = 2 * (control_index - 1)
        if all(bounds[offset + dim][0] <= value <= bounds[offset + dim][1]
               for dim, value in enumerate(tangent_control)):
            controls[control_index] = tangent_control
            bounds[offset:offset + 2] = [(float(value), float(value)) for value in tangent_control]
    weight, spans = protected_weight(reference, sample_s)

    def objective(flat: np.ndarray) -> tuple[float, np.ndarray]:
        c = controls.copy()
        c[1:-1] = flat.reshape(-1, 2)
        curve = basis @ c
        delta = curve - target
        original_delta = curve - original_target
        bend = second @ c
        directions = np.diff(curve, axis=0)
        lengths = np.linalg.norm(directions, axis=1)
        units = directions / np.maximum(lengths[:, None], 1e-9)
        length_weight = 1.5 / (sample_s[1] - sample_s[0])
        length_gradient = np.zeros_like(curve)
        length_gradient[:-1] -= units * length_weight
        length_gradient[1:] += units * length_weight
        field, field_gradient = grid.field(curve)
        desired_gap = np.minimum(field - sample_clearance, 0)
        collision_gap = np.minimum(field - 0.035, 0)
        norm = np.linalg.norm(original_delta, axis=1)
        deviation_gap = np.maximum(norm - radius * 0.95, 0)
        terrain_delta = original_delta * (1 - weight[:, None])
        value = (
            np.sum(delta ** 2) + options.smoothness * np.sum(bend ** 2)
            + 5000 * np.sum(desired_gap ** 2) + 3000 * np.sum(collision_gap ** 2)
            + 1000 * np.sum(deviation_gap ** 2) + 150 * np.sum(terrain_delta ** 2)
            + length_weight * lengths.sum()
        )
        grad_curve = (
            2 * delta + 10000 * desired_gap[:, None] * field_gradient
            + 6000 * collision_gap[:, None] * field_gradient
            + 2000 * (deviation_gap / np.maximum(norm, 1e-9))[:, None] * original_delta
            + 300 * terrain_delta * (1 - weight[:, None])
            + length_gradient
        )
        grad = basis.T @ grad_curve + 2 * options.smoothness * (second.T @ bend)
        return float(value), np.asarray(grad)[1:-1].ravel()

    solution = minimize(objective, controls[1:-1].ravel(), method="L-BFGS-B", jac=True,
                        bounds=bounds, options={"maxiter": 220, "ftol": 1e-9, "maxls": 30})
    controls[1:-1] = solution.x.reshape(-1, 2)
    raw_xy = BSpline(knots, controls, 3)(s)
    weight, _ = protected_weight(reference, s)
    delta = raw_xy - ref[:, :2]
    delta_norm = np.linalg.norm(delta, axis=1)
    weight *= np.minimum(1, radius / np.maximum(delta_norm, 1e-9))
    repair_weight = np.ones(len(s))
    candidate = ref.copy()
    repair_passes = 0
    for repair_passes in range(9):
        candidate[:, :2] = ref[:, :2] + delta * (weight * repair_weight)[:, None]
        candidate[:, :2] = project_clearance(grid, candidate[:, :2], required_clearance, weight > 1e-10)
        terrain.project(grid, candidate, ref, s)
        candidate[[0, -1]] = reference[[0, -1]]
        checked = check_path(grid, candidate, reference, s, radius, terrain, clearance)
        # Existing baseline limitations are reported separately, never silently accepted.
        inherited = np.array(base_check["invalidIndices"], dtype=int)
        unchanged = np.linalg.norm(candidate - ref, axis=1) < 1e-8
        inherited_unchanged = set(int(i) for i in inherited if unchanged[i])
        failures = np.array([i for i in checked["invalidIndices"] if i not in inherited_unchanged], dtype=int)
        if len(failures) == 0 or repair_passes == 8:
            break
        bad_s = s[failures]
        nearest_distance, _ = cKDTree(bad_s[:, None]).query(s[:, None])
        t = ((nearest_distance - 0.15) / 2.0).clip(0, 1)
        taper = t ** 3 * (10 - 15 * t + 6 * t * t)
        repair_weight = np.minimum(repair_weight, taper)
    candidate, ref, refinement = continuous_refinement(grid, candidate, s, reference, options, terrain, clearance)
    s = refinement["referenceS"]
    raw_xy = BSpline(knots, controls, 3)(s)
    checked = refinement["validation"]
    base_check = refinement["baselineValidation"]
    old_metrics = metrics(grid, reference, options.clearance)
    new_metrics = metrics(grid, candidate, options.clearance)
    retained = np.linalg.norm(candidate[:, :2] - ref[:, :2], axis=1) < 0.001
    elapsed = (perf_counter() - started) * 1000
    return {
        "algorithm": "corridor-shortcut-bspline-v3",
        "options": asdict(options),
        "baseline": old_metrics,
        "optimized": new_metrics,
        "validation": checked,
        "baselineValidation": base_check,
        "path": candidate.tolist(),
        "reference": ref.tolist(),
        "rawCandidate": np.column_stack([raw_xy, ref[:, 2]]).tolist(),
        "shortcut": shortcut,
        "retainedIndices": np.flatnonzero(retained).tolist(),
        "protectedSpans": spans,
        "solver": {
            "converged": bool(solution.success), "iterations": int(solution.nit),
            "message": str(solution.message), "repairPasses": repair_passes,
            "controlPoints": n_control, "timeMs": round(elapsed, 1),
            "retainedPercent": round(float(retained.mean() * 100), 2),
            "continuity": "C2", "refinementPasses": refinement["iterations"],
            "minimumSpeed": refinement["minimumSpeed"],
            "constraintSamples": refinement["constraintSamples"],
            "heightRematchedSamples": int((np.abs(candidate[:, 2] - ref[:, 2]) > 1e-8).sum()),
        },
        "scope": {
            "checks": ["XY occupancy", "diagonal support", "local terrain layer", "preserved steps", "deviation", "boundary floor", "endpoints"],
            "production3dGraphVerified": False,
            "robotExecutionVerified": False,
        },
    }
