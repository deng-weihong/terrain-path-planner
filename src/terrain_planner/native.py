"""Optional JSON adapter for the independently built native executable."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess


def native_binary() -> Path | None:
    configured = os.environ.get("PATH_PLANNER_CPP")
    default = Path(__file__).resolve().parents[2] / "build" / "cpp" / (
        "terrain-plan.exe" if os.name == "nt" else "terrain-plan"
    )
    candidate = Path(configured).expanduser().resolve() if configured else default
    return candidate if candidate.is_file() and os.access(candidate, os.X_OK) else None


def run_native(request: dict) -> dict:
    binary = native_binary()
    if binary is None:
        raise ValueError("C++ executable is unavailable; build cpp/ or set PATH_PLANNER_CPP")
    try:
        completed = subprocess.run(
            [str(binary)], input=json.dumps(request, allow_nan=False),
            capture_output=True, text=True, encoding="utf-8", timeout=120,
        )
    except subprocess.TimeoutExpired as error:
        raise ValueError("C++ computation exceeded 120 seconds") from error
    if completed.returncode:
        try:
            message = json.loads(completed.stderr)["error"]
        except (ValueError, KeyError, TypeError):
            message = "Native process failed"
        raise ValueError(f"C++: {message}")
    result = json.loads(completed.stdout)
    if not isinstance(result, dict):
        raise ValueError("C++ returned an invalid result")
    return result
