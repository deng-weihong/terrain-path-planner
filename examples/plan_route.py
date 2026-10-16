"""Plan and smooth a route without HTTP, ROS or a robot."""

import json

from terrain_planner import Options, optimize
from terrain_planner.datasets import load_datasets
from terrain_planner.planner import plan_reference


def main():
    scene = load_datasets(include_local=False)["zigzag"]
    options = Options()
    route, search = plan_reference(scene, (2.05, 3.05, .05), (20.05, 9.05, .05), options.clearance)
    result = optimize(scene.grid, route, options)
    print(json.dumps({
        "algorithm": result["algorithm"],
        "planning": search,
        "lengthM": result["optimized"]["lengthM"],
        "curvatureRms": result["optimized"]["curvatureRms"],
        "valid": result["validation"]["valid"],
        "newViolationCount": result["validation"]["newViolationCount"],
        "inheritedViolationCount": result["validation"]["inheritedViolationCount"],
        "start": result["path"][0],
        "goal": result["path"][-1],
    }, indent=2))


if __name__ == "__main__":
    main()
