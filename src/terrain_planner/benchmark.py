"""Reproduce parameter comparisons without a browser or external services."""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from .datasets import load_datasets
from .optimizer import Options, optimize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--include-local", action="store_true", help="Include the optional local map")
    parser.add_argument("--output", type=Path, default=Path("artifacts/benchmark.json"))
    args = parser.parse_args()
    presets = {
        "conservative": Options(smoothness=2, max_deviation=.4, control_spacing=1),
        "balanced": Options(),
        "smooth": Options(smoothness=30, max_deviation=1.5, control_spacing=2.5),
    }
    records = []
    for dataset in load_datasets(include_local=args.include_local).values():
        for name, options in presets.items():
            result = optimize(dataset.grid, dataset.route, options)
            before, after = result["baseline"], result["optimized"]
            record = {key: result[key] for key in ("algorithm", "options", "validation", "baselineValidation", "solver")}
            record.update({
                "dataset": dataset.id, "preset": name, "mapSha256": dataset.digest,
                "baseline": {key: value for key, value in before.items() if key != "profile"},
                "optimized": {key: value for key, value in after.items() if key != "profile"},
                "curvatureReductionPercent": (1 - after["curvatureRms"] / before["curvatureRms"]) * 100,
                "turningReductionPercent": (1 - after["turningDeg"] / before["turningDeg"]) * 100,
            })
            records.append(record)
            print(f'{dataset.id:12} {name:12} RMS {before["curvatureRms"]:.3f} -> {after["curvatureRms"]:.3f}'
                  f' | turn {before["turningDeg"]:.0f} -> {after["turningDeg"]:.0f} deg'
                  f' | new violations {result["validation"]["newViolationCount"]}')
    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"createdAt": datetime.now(timezone.utc).isoformat(), "experiments": records}, indent=2))
    print(output)


if __name__ == "__main__":
    main()
