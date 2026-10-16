"""Create a reviewable source snapshot using an explicit allowlist."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import tarfile


ROOT = Path(__file__).resolve().parents[1]
FILES = (
    "README.md", "README.en.md", "CONTRIBUTING.md", "CONTRIBUTING.en.md",
    "NOTICE.md", "NOTICE.en.md", "LICENSE",
    ".gitignore", ".gitattributes", ".editorconfig", ".clang-format",
    "pyproject.toml", "package.json", "package-lock.json",
    "index.html", "tsconfig.json", "vite.config.ts", "data/README.md", "data/README.en.md",
)
DIRECTORIES = ("src", "cpp", "web", "tests", "examples", "scripts", "docs", ".github")
EXCLUDED = {"__pycache__", "node_modules", ".venv", ".pytest_cache", ".ruff_cache",
            "build", "dist", "CMakeFiles", "_deps", ".git"}


def source_files():
    files = [ROOT / name for name in FILES if (ROOT / name).is_file()]
    for name in DIRECTORIES:
        directory = ROOT / name
        if directory.exists():
            files.extend(directory.rglob("*"))
    for file in sorted(set(files)):
        relative = file.relative_to(ROOT)
        if any(part in EXCLUDED or part.endswith(".egg-info") or part.startswith("cmake-build-")
               for part in relative.parts):
            continue
        if file.suffix in {".pyc", ".pyo", ".log", ".pid"}:
            continue
        if file.is_symlink():
            raise ValueError(f"Source snapshot cannot contain symlinks: {relative}")
        if file.is_file():
            yield file, relative


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "releases")
    args = parser.parse_args()
    version = json.loads((ROOT / "package.json").read_text(encoding="utf-8"))["version"]
    prefix = f"terrain-path-planner-{version}"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f"{prefix}-source.tar.gz"
    count = 0
    with output.open("wb") as target:
        with gzip.GzipFile(filename="", mode="wb", fileobj=target, mtime=0) as compressed:
            with tarfile.open(mode="w", fileobj=compressed, format=tarfile.PAX_FORMAT) as archive:
                for file, relative in source_files():
                    info = archive.gettarinfo(str(file), arcname=f"{prefix}/{relative.as_posix()}")
                    info.uid = info.gid = info.mtime = 0
                    info.uname = info.gname = ""
                    info.mode = 0o644
                    with file.open("rb") as source:
                        archive.addfile(info, source)
                    count += 1
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_name(output.name + ".sha256").write_text(f"{digest}  {output.name}\n", encoding="ascii")
    print(f"{output}\n{count} source files; sha256 {digest}")


if __name__ == "__main__":
    main()
