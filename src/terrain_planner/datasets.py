from __future__ import annotations

import hashlib
import io
import json
import os
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .optimizer import GridMap, metrics, resample

LOCAL_DATA = Path(__file__).resolve().parents[2] / "data" / "local" / "site-map"


@dataclass
class Dataset:
    id: str
    name: str
    grid: GridMap
    route: np.ndarray
    digest: str

    def describe(self) -> dict:
        resolution = self.grid.resolution
        minimum = self.grid.origin * resolution
        height, width = self.grid.mask.shape
        return {
            "id": self.id, "name": self.name, "mapSha256": self.digest,
            "resolution": resolution, "voxelCount": len(self.grid.keys),
            "areaM2": round(float(self.grid.mask.sum() * resolution ** 2), 2),
            "bounds": {"minX": float(minimum[0]), "minY": float(minimum[1]),
                       "maxX": float(minimum[0] + width * resolution),
                       "maxY": float(minimum[1] + height * resolution)},
            "imageUrl": f"/api/datasets/{self.id}/map.png",
            "voxelUrl": f"/api/datasets/{self.id}/voxels.bin?v={self.digest}",
            "voxelEncoding": "int32-le-xyz",
            "heightRange": {"min": float((self.grid.keys[:, 2].min() + .5) * resolution),
                            "max": float((self.grid.keys[:, 2].max() + .5) * resolution)},
            "route": self.route.tolist(),
            "baseline": metrics(self.grid, self.route, 0.39),
        }

    def image(self) -> bytes:
        from PIL import Image

        rgba = np.zeros((*self.grid.mask.shape, 4), dtype=np.uint8)
        rgba[self.grid.mask] = [194, 218, 210, 255]
        edge = self.grid.mask & (self.grid.distance < 0.15)
        rgba[edge] = [137, 170, 160, 255]
        rgba[self.grid.mask & (self.grid.distance >= 0.5)] = [213, 230, 222, 255]
        out = io.BytesIO()
        Image.fromarray(np.flipud(rgba)).save(out, format="PNG")
        return out.getvalue()


def load_dataset(directory: str | Path) -> Dataset:
    directory = Path(directory).resolve()
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if not re.fullmatch(r"[a-z0-9][a-z0-9-]{0,63}", manifest["id"]):
        raise ValueError("Dataset id must contain lowercase letters, digits or hyphens")
    resolution = float(manifest["resolution"])
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("Map resolution must be finite and positive")

    def read_file(key: str) -> bytes:
        file = (directory / manifest[key]).resolve()
        if not file.is_relative_to(directory):
            raise ValueError(f"{key} must be inside the dataset directory")
        return file.read_bytes()

    raw = read_file("mapFile")
    if not raw or len(raw) % 12:
        raise ValueError("Map must contain little-endian int32 XYZ triples")
    keys = np.frombuffer(raw, dtype="<i4").reshape(-1, 3)
    fixture = json.loads(read_file("routeFile"))
    points = fixture if isinstance(fixture, list) else fixture["pathPoints"]
    route = np.asarray([[p[k] for k in ("x", "y", "z")] if isinstance(p, dict) else p for p in points], dtype=float)
    if route.ndim != 2 or route.shape[1] != 3 or not 2 <= len(route) <= 20000 or not np.isfinite(route).all():
        raise ValueError("Route must contain 2 to 20000 finite XYZ points")
    return Dataset(manifest["id"], manifest["name"], GridMap(keys, resolution), route,
                   hashlib.sha256(raw).hexdigest())


def load_datasets(*, include_local: bool = True, data_dir: str | Path | None = None) -> dict[str, Dataset]:

    x, y = np.meshgrid(np.arange(240), np.arange(120))
    open_keys = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size, dtype=int)])
    xs = np.linspace(1.05, 22.05, 85)
    ys = 5.05 + 0.36 * np.sin(np.linspace(0, 20 * np.pi, len(xs)))
    zigzag = np.column_stack([xs, ys, np.full(len(xs), 0.05)])
    open_map = Dataset("zigzag", "连续摆动", GridMap(open_keys), zigzag,
                       hashlib.sha256(open_keys.astype("<i4").tobytes()).hexdigest())

    blocked = ((x >= 65) & (x <= 90) & (y <= 65)) | ((x >= 145) & (x <= 170) & (y >= 45))
    corridor_keys = np.column_stack([x[~blocked], y[~blocked], np.zeros((~blocked).sum(), dtype=int)])
    corridor_route = np.array([
        [1.05, 5.05, .05], [5.05, 5.05, .05], [5.05, 8.05, .05],
        [10.05, 8.05, .05], [12.05, 8.05, .05], [12.05, 2.05, .05],
        [18.05, 2.05, .05], [19.05, 5.05, .05], [22.05, 5.05, .05],
    ])
    corridor_route, _ = resample(corridor_route)
    corridor = Dataset("obstacles", "障碍绕行", GridMap(corridor_keys), corridor_route,
                       hashlib.sha256(corridor_keys.astype("<i4").tobytes()).hexdigest())
    datasets = {dataset.id: dataset for dataset in (corridor, open_map)}
    if include_local and os.environ.get("PATH_PLANNER_NO_LOCAL_DATA") != "1":
        configured = data_dir if data_dir is not None else os.environ.get("PATH_PLANNER_DATA_DIR")
        directory = Path(configured) if configured else LOCAL_DATA
        if configured is not None or (directory / "manifest.json").is_file():
            local = load_dataset(directory)
            if local.id in datasets:
                raise ValueError(f"Local dataset id conflicts with a built-in scene: {local.id}")
            datasets[local.id] = local
    return datasets
