import time
import unittest
from unittest.mock import patch

import numpy as np
from fastapi.testclient import TestClient

from terrain_planner import viewer as server
from terrain_planner.datasets import Dataset
from terrain_planner.optimizer import GridMap
from terrain_planner.native import native_binary


class PlanningApiTests(unittest.TestCase):
    def setUp(self):
        self.client = TestClient(server.app)
        with server.lock:
            server.jobs.clear()

    def wait_job(self, job_id):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            result = self.client.get(f"/api/jobs/{job_id}").json()
            if result["status"] != "running":
                return result
            time.sleep(.01)
        self.fail("Planning did not finish")

    def test_complete_job_exports_matching_map_options_and_path(self):
        response = self.client.post("/api/optimize", json={"dataset": "zigzag", "max_deviation": .6})
        self.assertEqual(response.status_code, 202)
        job_id = response.json()["id"]
        result = self.wait_job(job_id)
        self.assertEqual(result["status"], "completed", result)
        exported = self.client.get(f"/api/jobs/{job_id}/export")
        self.assertEqual(exported.status_code, 200)
        payload = exported.json()
        self.assertEqual(payload["mapSha256"], server.datasets["zigzag"].digest)
        self.assertEqual(payload["options"]["max_deviation"], .6)
        self.assertEqual(payload["validation"]["newViolationCount"], 0)
        self.assertEqual(payload["validation"]["clearanceViolationCount"], 0)
        self.assertEqual(payload["algorithm"], "corridor-shortcut-bspline-v3")
        self.assertGreater(payload["shortcut"]["segments"], 0)
        self.assertEqual(payload["path"][0], server.datasets["zigzag"].route[0].tolist())

    def test_invalid_parameters_and_unknown_resources(self):
        self.assertEqual(self.client.post("/api/optimize", json={"max_deviation": 4}).status_code, 422)
        self.assertEqual(self.client.post("/api/optimize", json={"route": [[0, 0, 0]]}).status_code, 422)
        self.assertEqual(self.client.get("/api/datasets/missing").status_code, 404)
        self.assertEqual(self.client.get("/api/jobs/missing").status_code, 404)

    def test_engine_availability_and_missing_native_binary(self):
        engines = self.client.get("/api/engines").json()
        self.assertTrue(next(item for item in engines if item["id"] == "python")["available"])
        self.assertEqual(self.client.post("/api/optimize", json={"engine": "unknown"}).status_code, 422)
        with patch.object(server, "native_binary", return_value=None):
            self.assertEqual(self.client.post("/api/optimize", json={"engine": "cpp"}).status_code, 503)

    @unittest.skipUnless(native_binary() is not None, "C++ executable is not built")
    def test_native_endpoint_planning_and_export_use_cpp(self):
        start = self.client.post("/api/datasets/zigzag/pick", json={"x": 2, "y": 3}).json()["point"]
        goal = self.client.post("/api/datasets/zigzag/pick", json={"x": 10, "y": 8}).json()["point"]
        response = self.client.post("/api/optimize", json={"dataset": "zigzag", "engine": "cpp", "start": start, "goal": goal})
        self.assertEqual(response.status_code, 202)
        job = self.wait_job(response.json()["id"])
        self.assertEqual(job["status"], "completed", job)
        result = self.client.get(f'/api/jobs/{job["id"]}/export').json()
        self.assertEqual(result["engine"], "cpp")
        self.assertEqual(result["planning"]["engine"], "traversable-graph-cpp-v1")
        self.assertTrue(result["validation"]["valid"], result["validation"])
        np.testing.assert_allclose(np.asarray(result["path"])[[0, -1]], [start, goal], atol=1e-9, rtol=0)

    def test_import_failure_finishes_and_does_not_block_next_job(self):
        response = self.client.post("/api/optimize", json={
            "dataset": "zigzag", "route": [[-100, -100, 0], [-99, -100, 0]],
        })
        result = self.wait_job(response.json()["id"])
        self.assertEqual(result["status"], "failed")
        self.assertIn("outside", result["error"])
        retry = self.client.post("/api/optimize", json={"dataset": "zigzag"})
        self.assertEqual(retry.status_code, 202)
        self.assertEqual(self.wait_job(retry.json()["id"])["status"], "completed")

    def test_concurrent_job_and_unfinished_export_are_rejected(self):
        with patch.object(server.pool, "submit"):
            response = self.client.post("/api/optimize", json={"dataset": "zigzag"})
            job_id = response.json()["id"]
            self.assertEqual(self.client.post("/api/optimize", json={}).status_code, 409)
            self.assertEqual(self.client.get(f"/api/jobs/{job_id}/export").status_code, 409)
        with server.lock:
            server.jobs.clear()

    def test_pick_snaps_only_inside_map_and_lists_ground_heights(self):
        response = self.client.post("/api/datasets/zigzag/pick", json={"x": 2.01, "y": 3.09})
        self.assertEqual(response.status_code, 200)
        self.assertEqual([round(v, 2) for v in response.json()["point"]], [2.05, 3.05, .05])
        self.assertEqual(response.json()["heights"], [.05])
        self.assertEqual(self.client.post("/api/datasets/zigzag/pick", json={"x": -1, "y": -1}).status_code, 422)
        self.assertEqual(self.client.post("/api/datasets/zigzag/pick", json={"x": "NaN", "y": 1}).status_code, 422)
        self.assertEqual(self.client.post("/api/datasets/missing/pick", json={"x": 1, "y": 1}).status_code, 404)

    def test_three_dimensional_map_binary_matches_original_xyz_voxels(self):
        for dataset_id in server.datasets:
            dataset = server.datasets[dataset_id]
            description = self.client.get(f"/api/datasets/{dataset_id}").json()
            self.assertEqual(description["voxelEncoding"], "int32-le-xyz")
            response = self.client.get(description["voxelUrl"])
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.headers["content-type"], "application/octet-stream")
            self.assertEqual(len(response.content), len(dataset.grid.keys) * 12)
            decoded = np.frombuffer(response.content, dtype="<i4").reshape(-1, 3)
            np.testing.assert_array_equal(decoded, dataset.grid.keys)
            self.assertAlmostEqual(description["heightRange"]["min"], (decoded[:, 2].min() + .5) * dataset.grid.resolution)
            self.assertAlmostEqual(description["heightRange"]["max"], (decoded[:, 2].max() + .5) * dataset.grid.resolution)
        self.assertEqual(self.client.get("/api/datasets/missing/voxels.bin").status_code, 404)

    def test_ray_picked_height_is_preserved_instead_of_using_reference_floor(self):
        keys = np.array([(x, y, z) for x in range(5) for y in range(5) for z in (0, 8)])
        dataset = Dataset("height-test", "height-test", GridMap(keys), np.array([[.05, .05, .85], [.45, .45, .85]]), "test")
        with patch.dict(server.datasets, {dataset.id: dataset}):
            default = self.client.post("/api/datasets/height-test/pick", json={"x": .25, "y": .25}).json()
            self.assertAlmostEqual(default["point"][2], .85)
            selected = self.client.post("/api/datasets/height-test/pick", json={"x": .25, "y": .25, "z": .05})
            self.assertEqual(selected.status_code, 200)
            self.assertAlmostEqual(selected.json()["point"][2], .05)
            self.assertEqual(len(selected.json()["heights"]), 2)
            self.assertEqual(self.client.post("/api/datasets/height-test/pick", json={"x": .25, "y": .25, "z": .45}).status_code, 422)
            self.assertEqual(self.client.post("/api/datasets/height-test/pick", json={"x": .25, "y": .25, "z": "NaN"}).status_code, 422)

    def test_manual_endpoints_plan_optimize_export_and_rerun_original_grid_route(self):
        start = self.client.post("/api/datasets/zigzag/pick", json={"x": 2, "y": 3}).json()["point"]
        goal = self.client.post("/api/datasets/zigzag/pick", json={"x": 10, "y": 8}).json()["point"]
        response = self.client.post("/api/optimize", json={"dataset": "zigzag", "start": start, "goal": goal})
        self.assertEqual(response.status_code, 202)
        job = self.wait_job(response.json()["id"])
        self.assertEqual(job["status"], "completed", job)
        result = job["result"]
        self.assertEqual(result["planning"]["engine"], "traversable-graph-v1")
        self.assertEqual(result["sourceRoute"][0], start)
        self.assertEqual(result["sourceRoute"][-1], goal)
        self.assertEqual(result["path"][0], start)
        self.assertEqual(result["path"][-1], goal)
        self.assertEqual(result["validation"]["newViolationCount"], 0)
        self.assertEqual(self.client.get(f'/api/jobs/{job["id"]}/export').json()["sourceRoute"], result["sourceRoute"])
        rerun = self.client.post("/api/optimize", json={"dataset": "zigzag", "route": result["sourceRoute"], "smoothness": 8})
        repeated = self.wait_job(rerun.json()["id"])
        self.assertEqual(repeated["status"], "completed", repeated)
        self.assertEqual(repeated["result"]["sourceRoute"], result["sourceRoute"])

    def test_endpoints_require_pair_and_exclude_import_and_recover_from_bad_height(self):
        point = [1.05, 5.05, .05]
        self.assertEqual(self.client.post("/api/optimize", json={"start": point}).status_code, 422)
        self.assertEqual(self.client.post("/api/optimize", json={"start": point, "goal": point, "route": [point, point]}).status_code, 422)
        response = self.client.post("/api/optimize", json={"dataset": "zigzag", "start": point, "goal": [3.05, 5.05, 4]})
        job = self.wait_job(response.json()["id"])
        self.assertEqual(job["status"], "failed")
        self.assertIn("高程", job["error"])


if __name__ == "__main__":
    unittest.main()
