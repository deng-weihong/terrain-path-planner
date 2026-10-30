from __future__ import annotations

import argparse
import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from pathlib import Path
from typing import Literal
from uuid import uuid4

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, model_validator

from .datasets import load_datasets
from .native import native_binary, run_native
from .optimizer import Options, optimize
from .planner import pick_point, plan_reference

WEB_ROOT = Path(os.environ.get("PATH_PLANNER_WEB_DIR", Path(__file__).resolve().parents[2] / "dist"))
app = FastAPI(title="Terrain Path Planner", docs_url="/api/docs")
datasets = load_datasets()
descriptions = {key: dataset.describe() for key, dataset in datasets.items()}
images = {key: dataset.image() for key, dataset in datasets.items()}
voxels = {key: np.asarray(dataset.grid.keys, dtype="<i4").tobytes() for key, dataset in datasets.items()}
pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="path-planner")
jobs: dict[str, dict] = {}
lock = threading.Lock()


class OptimizeRequest(BaseModel):
    dataset: str = Field(default="obstacles", min_length=1, max_length=64)
    engine: Literal["python", "cpp"] = "python"
    smoothness: float = Field(default=6, ge=0.1, le=60)
    max_deviation: float = Field(default=0.8, ge=0.05, le=3)
    clearance: float = Field(default=0.39, ge=0, le=1)
    control_spacing: float = Field(default=1.5, ge=0.4, le=4)
    navigation_height: float = Field(default=.35, ge=.1, le=1.5)
    elevation_radius: float = Field(default=1.5, ge=.3, le=3)
    route: list[tuple[float, float, float]] | None = Field(default=None, min_length=2, max_length=20000)
    start: tuple[float, float, float] | None = None
    goal: tuple[float, float, float] | None = None

    @model_validator(mode="after")
    def check_endpoints(self):
        if (self.start is None) != (self.goal is None):
            raise ValueError("起点和终点需要同时提供")
        if self.start is not None and self.route is not None:
            raise ValueError("手动端点规划不能同时指定导入路线")
        return self


class PickRequest(BaseModel):
    x: float = Field(allow_inf_nan=False)
    y: float = Field(allow_inf_nan=False)
    z: float | None = Field(default=None, allow_inf_nan=False)


@app.get("/api/health")
def health():
    return {"status": "ok", "project": "terrain-path-planner"}


@app.get("/api/datasets")
def list_datasets():
    return [{"id": value.id, "name": value.name} for value in datasets.values()]


@app.get("/api/engines")
def list_engines():
    return [{"id": "python", "name": "Python", "available": True},
            {"id": "cpp", "name": "C++", "available": native_binary() is not None}]


@app.get("/api/datasets/{dataset_id}")
def get_dataset(dataset_id: str):
    if dataset_id not in datasets:
        raise HTTPException(404, "Dataset not found")
    return descriptions[dataset_id]


@app.get("/api/datasets/{dataset_id}/map.png")
def get_image(dataset_id: str):
    if dataset_id not in images:
        raise HTTPException(404, "Dataset not found")
    return Response(images[dataset_id], media_type="image/png", headers={"Cache-Control": "public, max-age=3600"})


@app.post("/api/datasets/{dataset_id}/pick")
def pick_endpoint(dataset_id: str, request: PickRequest):
    if dataset_id not in datasets:
        raise HTTPException(404, "Dataset not found")
    try:
        return pick_point(datasets[dataset_id], request.x, request.y, request.z)
    except ValueError as error:
        raise HTTPException(422, str(error)) from error


@app.get("/api/datasets/{dataset_id}/voxels.bin")
def get_voxels(dataset_id: str):
    if dataset_id not in voxels:
        raise HTTPException(404, "Dataset not found")
    return Response(voxels[dataset_id], media_type="application/octet-stream",
                    headers={"Cache-Control": "public, max-age=3600",
                             "ETag": f'"{datasets[dataset_id].digest}"'})


def run_job(job_id: str, request: OptimizeRequest):
    try:
        dataset = datasets[request.dataset]
        options = Options(**request.model_dump(exclude={"dataset", "engine", "route", "start", "goal"}))
        native_input = {"resolution": dataset.grid.resolution, "voxels": dataset.grid.keys.tolist(),
                        "options": asdict(options)} if request.engine == "cpp" else None
        planning = None
        if request.start is not None and request.goal is not None:
            if native_input is not None:
                searched = run_native({**native_input, "operation": "search",
                                       "start": request.start, "goal": request.goal})
                if not searched["success"]:
                    raise ValueError(f'C++ search failed: {searched["status"]}')
                reference, planning = np.asarray(searched["path"]), searched["planning"]
            else:
                reference, planning = plan_reference(dataset, request.start, request.goal, options.clearance)
        else:
            reference = np.asarray(request.route) if request.route is not None else dataset.route
        with lock:
            jobs[job_id]["phase"] = "optimizing"
        if not np.isfinite(reference).all():
            raise ValueError("Route coordinates must be finite")
        s = np.linalg.norm(np.diff(reference[:, :2], axis=0), axis=1).sum()
        if s < 0.1 or s > 1000:
            raise ValueError("Route length must be between 0.1 and 1000 m")
        supported = np.array([tuple(key) in dataset.grid.columns
                              for key in np.floor(reference[:, :2] / dataset.grid.resolution).astype(int)])
        if not supported.all():
            raise ValueError(f"Imported reference has {int((~supported).sum())} points outside this map")
        result = (run_native({**native_input, "route": reference.tolist()}) if native_input is not None
                  else optimize(dataset.grid, reference, options))
        result.update({"dataset": dataset.id, "mapSha256": dataset.digest, "jobId": job_id,
                       "engine": request.engine, "sourceRoute": reference.tolist(), "planning": planning})
        with lock:
            jobs[job_id].update(status="completed", result=result)
    except Exception as error:
        with lock:
            jobs[job_id].update(status="failed", error=str(error))


@app.post("/api/optimize", status_code=202)
def create_job(request: OptimizeRequest):
    if request.dataset not in datasets:
        raise HTTPException(404, "Dataset not found")
    if request.engine == "cpp" and native_binary() is None:
        raise HTTPException(503, "C++ executable is unavailable; build cpp/ or set PATH_PLANNER_CPP")
    with lock:
        if any(job["status"] == "running" for job in jobs.values()):
            raise HTTPException(409, "An optimization is already running")
        if len(jobs) >= 24:
            oldest = next(iter(jobs))
            del jobs[oldest]
        job_id = uuid4().hex
        jobs[job_id] = {"id": job_id, "status": "running",
                       "phase": "planning" if request.start is not None else "optimizing",
                       "options": request.model_dump(exclude={"route"})}
    pool.submit(run_job, job_id, request)
    return {"id": job_id, "status": "running"}


@app.get("/api/jobs/{job_id}")
def get_job(job_id: str):
    with lock:
        if job_id not in jobs:
            raise HTTPException(404, "Job not found")
        return jobs[job_id].copy()


@app.get("/api/jobs/{job_id}/export")
def export_job(job_id: str):
    job = get_job(job_id)
    if job["status"] != "completed":
        raise HTTPException(409, "Job is not complete")
    return Response(json.dumps(job["result"], ensure_ascii=False, indent=2), media_type="application/json",
                    headers={"Content-Disposition": f'attachment; filename="path-planner-{job_id[:8]}.json"'})


@app.get("/")
def index():
    file = WEB_ROOT / "index.html"
    if not file.exists():
        raise HTTPException(503, "Build the frontend with npm run build first")
    return FileResponse(file)


if (WEB_ROOT / "assets").exists():
    app.mount("/assets", StaticFiles(directory=WEB_ROOT / "assets"), name="assets")


def main():
    import uvicorn

    parser = argparse.ArgumentParser(description="Terrain Path Planner viewer")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5181)
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
