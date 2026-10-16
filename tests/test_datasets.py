import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from terrain_planner.datasets import load_dataset, load_datasets


class DatasetTests(unittest.TestCase):
    def test_public_scenes_work_without_local_files_and_hash_downloaded_encoding(self):
        scenes = load_datasets(include_local=False)
        self.assertEqual(set(scenes), {"zigzag", "obstacles"})
        for scene in scenes.values():
            raw = np.asarray(scene.grid.keys, dtype="<i4").tobytes()
            self.assertEqual(scene.digest, hashlib.sha256(raw).hexdigest())
            self.assertEqual(scene.route.shape[1], 3)

    def test_custom_map_loads_from_manifest_without_workspace_dependency(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            keys = np.array([[x, y, 0] for x in range(5) for y in range(5)], dtype="<i4")
            (folder / "map.bin").write_bytes(keys.tobytes())
            (folder / "route.json").write_text(json.dumps([[.05, .05, .05], [.45, .45, .05]]))
            manifest = {"id": "custom-map", "name": "Custom", "resolution": .1,
                        "mapFile": "map.bin", "routeFile": "route.json"}
            (folder / "manifest.json").write_text(json.dumps(manifest))
            scene = load_dataset(folder)
            np.testing.assert_array_equal(scene.grid.keys, keys)
            self.assertEqual(scene.id, "custom-map")
            (folder / "map.bin").write_bytes(b"short")
            with self.assertRaisesRegex(ValueError, "int32 XYZ"):
                load_dataset(folder)

    def test_manifest_file_paths_stay_inside_dataset(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            manifest = {"id": "custom-map", "name": "Custom", "resolution": .1,
                        "mapFile": "../outside.bin", "routeFile": "route.json"}
            (folder / "manifest.json").write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "inside the dataset"):
                load_dataset(folder)


if __name__ == "__main__":
    unittest.main()
