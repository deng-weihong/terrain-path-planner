# Algorithm Design

[简体中文](../algorithm.md) | [English](algorithm.md)

This document describes `corridor-shortcut-bspline-v3`. The objective is to reduce repeated heading changes and unnecessary detours across an entire route while preserving endpoints, ground support, local terrain, and clearance constraints. The pipeline combines graph search with continuous curve optimization.

## 1. Maps and Coordinates

The input consists of integer indices `(ix, iy, iz)` for traversable ground voxels, rather than occupancy points for all obstacles. For resolution `r`, a voxel center is:

```text
x = (ix + 0.5) * r
y = (iy + 0.5) * r
z = (iz + 0.5) * r
```

The inverse conversion uses `floor(coordinate / r)`, including for negative coordinates. Each XY column retains its own set of elevations, so multiple traversable Z layers can share the same XY position.

Search preserves the XYZ graph. Smoothing builds an XY projection mask and distance field, then checks ground support against the separate height columns. The projection merges layers at the same XY position, so its clearance cannot represent full 3D free space.

## 2. Traversable 3D Graph

Nodes are ground voxel centers. Edges represent allowed ground adjacency or step transitions. Main default rules are:

| Rule | Default or Behavior |
| --- | --- |
| Ordinary ground neighbor radius | 0.16 m |
| Ordinary ground elevation tolerance | 0.04 m |
| Maximum slope | 25 degrees |
| Step height range | 0.05-0.20 m |
| Maximum horizontal step gap | 0.30 m |
| Minimum connected component | 3 nodes |
| Ordinary diagonal adjacency | Support at both orthogonal bridge positions |

These rules jointly determine whether an edge exists. Proximity alone does not create an edge. Separate elevation layers connect only through a valid adjacency or step transition.

Adjacency, boundary features, and connected components are prepared before search. Map points and graph parameters form the cache key. Changing endpoints can reuse the graph; changing the map or parameters rebuilds it.

The viewer's planning adapter uses `exact_endpoints=True` to preserve the selected voxel centers and elevation layers. Unreachable endpoints produce a failure instead of being moved to another floor.

## 3. Search Costs and Boundary Preferences

Search considers connectivity and boundary preferences together with distance, vertical change, turning, and step costs. By default, connectivity-related costs are compared before physical path costs. Clearance is a soft preference, so not every point is guaranteed to meet the requested target.

Graph reachability, a preference for wider ground, and dense validation of the continuous curve are separate properties. Each is checked separately.

The endpoint adapter disables the graph planner's original local smoothing and passes the raw graph route to whole-path optimization. The graph module retains optional local postprocessing for compatibility; the viewer uses the global curve pipeline described here.

Turning costs and connectivity priorities mean that search does not guarantee the Euclidean shortest path across the map. The adapter limits search to 250,000 expanded nodes and 60 s for preparation plus search.

## 4. Route Corridor and Boundary Distance Field

Let `R(s)` be the reference route, with `s` its XY arc length, and `P(s)` the optimized route. The displacement constraint compares points at the same original arc-length parameter:

```text
||Pxy(s) - Rxy(s)|| <= max_deviation
```

This is different from taking the arbitrary nearest point on the original polyline. A fixed correspondence helps prevent jumps to unrelated neighboring branches, while also restricting the ability to select a different route across the map.

An Euclidean distance transform of the XY traversability mask produces the boundary distance field `D(x,y)`. Bilinear interpolation supplies values and matching gradients for numerical optimization and clearance projection.

Narrow endpoints and fixed terrain may not support the global clearance target. The algorithm starts with the minimum of the reference clearance and the target, then recovers toward the target in both directions at no more than `0.25 m/m`. This produces an adaptive floor `c(s)`. Final validation reports both violations of this floor and the actual global target attainment.

Zero violations of the adaptive floor do not imply that every point meets the global target. The distance field also does not subtract the robot footprint radius.

## 5. Removing Unnecessary Detours

Fitting the original polyline directly can preserve unnecessary double bends. Within the allowed route corridor, the algorithm first attempts to connect the current point to the farthest visible point along the route.

Each candidate straight segment is densely checked for XY traversability, diagonal support, ground layers, significant steps, displacement, and adaptive clearance. Only passing segments replace a detour. Step cores and existing unresolved reference positions are protected.

This is a greedy visibility shortcut along the reference route, not a global shortest-path computation. It supplies a more direct target skeleton for the joint curve optimization that follows.

## 6. Global Cubic B-Spline Optimization

The planar curve is represented as a cubic B-spline:

```text
Pxy(s) = sum_i B_i,3(s) * C_i
```

`B_i,3` are basis functions and `C_i` are control points. Route length and `control_spacing` determine the number of controls. Endpoint controls are fixed, and all interior controls are jointly optimized using L-BFGS-B.

The discrete objective includes:

```text
J = sum ||Pxy - target||^2
  + smoothness * sum ||Pxy''||^2
  + length_weight * sum segment_length
  + 5000 * sum min(D(Pxy) - desired_clearance, 0)^2
  + 3000 * sum min(D(Pxy) - 0.035, 0)^2
  + 1000 * sum max(||Pxy - Rxy|| - 0.95 * max_deviation, 0)^2
  + 150 * sum terrain_preservation_error^2
```

`target` comes from the shortcut skeleton, and `length_weight = 1.5 / target_sample_spacing`. The second-derivative term reduces repeated bending; the length term reduces detours. Other terms constrain clearance, the corridor, and protected terrain.

Coordinate box bounds provide additional control-point limits, but the resulting curve still needs dense validation. Soft penalties and solver convergence are not proofs of feasibility. The default limit is 220 L-BFGS-B iterations.

At narrow endpoints, neighboring control points also follow the departure or arrival direction inferred from about 0.5 m of the reference route. This avoids a short reverse hook caused by correcting only a few endpoint samples.

## 7. Ground Elevation and Clearance Correction

If an optimized XY position still supports the original ground elevation, that elevation is preserved. Only when a moved XY position no longer supports it does the algorithm try another ground layer allowed by the nearby reference terrain.

The allowed elevation envelope uses minimum and maximum reference heights within an arc-length window of `+/-max_deviation`, with about 0.08 m extra tolerance. An unrelated upper floor cannot provide support merely because it appears in the same XY projection.

Significant transitions above 0.15 m remain near their original terrain transition. New height changes above 0.20 m are rejected. Diagonal bridge positions must also support the appropriate elevation rather than borrowing another floor.

Insufficient clearance is corrected by projection along the distance-field gradient. Other failing positions reduce local displacement, with a gradual transition back to normal displacement nearby. This stage prepares refined target constraints; the final continuous curve is solved again afterward.

## 8. Refinement as One Continuous Curve

Blending parts of the original polyline back into the output can reintroduce tangent jumps. The final XY output is solved over the entire route using `SciPy.make_smoothing_spline`, with the basic objective:

```text
min_f sum_i w_i * ||f(s_i) - target_i||^2 + lambda * integral ||f''(s)||^2 ds
lambda = 0.12 * sqrt(smoothness / 6)
```

Fixed terrain, failed positions, and clearance corrections enter the same solve through higher fitting weights. Refinement and validation repeat without hiding discontinuities by splicing in the original polyline.

The XY spline is C2 continuous. Z is matched to ground layers, so C2 continuity is not promised across steps. C2 continuity alone does not ensure small curvature: near-zero tangent speed or conflicting constraints can still cause large local turns. Minimum tangent speed and dense heading changes are therefore checked separately.

## 9. Validation and Failure Classification

The reference route and final curve undergo the same dense checks. Sampling uses an original arc-length parameter spacing of at most 0.025 m, and the maximum actual spacing of output samples is reported. Checks cover:

- XY positions inside the traversable projection.
- Support for the current ground elevation.
- Diagonal bridge support at the appropriate heights.
- Height changes and significant step rules.
- Displacement from the reference at the same parameter.
- The adaptive clearance floor.
- Endpoint preservation.

An existing reference violation is retained in its original classification only when that position is unchanged. Results distinguish `newViolationCount` from `inheritedViolationCount`. `validation.valid` indicates whether all current sampled checks pass. The algorithm may return a curve with failure information for inspection; callers must inspect validation fields before deciding how to use it.

Reference-data problems, newly introduced violations, solver nonconvergence, and excessive curvature are different states and should not be collapsed into a single planning-success flag.

## 10. Metrics

| Metric | Convention |
| --- | --- |
| Planar length | XY polyline length |
| 3D length | XYZ polyline length, reported separately |
| Curvature RMS / accumulated turning | Uniform 0.2 m sampling along each route's own arc length |
| Maximum local turn / sharp-turn sample count | 0.05 m sampling; a 5-degree screening threshold |
| Minimum clearance / percentage below target | XY distance-field queries at 0.05 m spacing |
| Approximately retained route | Samples with less than 1 mm XY displacement from the reference at the same parameter |

The sharp-turn threshold is a comparison metric and has not been calibrated to a particular robot's kinematics. Curvature plots use each route's own traveled distance and show absolute curvature. Display clipping affects the chart, not the underlying metrics.

## 11. Current Scope

Input requires an existing traversable ground map. This project does not extract ground from a complete SLAM point cloud or implement localization, dynamic obstacle avoidance, speed planning, or chassis control.

The viewer renders actual XYZ coordinates, but rendering does not add algorithmic constraints. The final curve has not been revalidated edge by edge against the complete XYZ graph. Robot footprint and full 3D obstacle clearance are not modeled, and physical route tracking has not been verified.

Conflicting corridor and clearance requirements may prevent a valid result. Parameter changes do not guarantee monotonic metric improvements, and search plus smoothing does not guarantee the globally shortest route.
