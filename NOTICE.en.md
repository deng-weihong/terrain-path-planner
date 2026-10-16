# Provenance and Dependencies

[简体中文](NOTICE.md) | [English](NOTICE.en.md)

The 3D graph search originated in an existing robot navigation project and is maintained independently here. This project uses local types and coordinate conversions and does not require the original workspace. Whole-path smoothing, examples, and visualization are maintained in this standalone project.

This project is licensed under [MIT](LICENSE), copyright deng-weihong. Each third-party dependency retains its upstream license. Dependencies are installed through pip, npm, system packages, or CMake; their source files are not vendored into this project.

| Dependency | Purpose | Upstream License |
| --- | --- | --- |
| NumPy | Arrays and numerical operations | BSD-3-Clause |
| SciPy | Distance transforms, splines, sparse operations, and optimization | BSD-3-Clause |
| Pillow | Optional PNG map previews | HPND |
| FastAPI | Optional HTTP API | MIT |
| Uvicorn | ASGI server | BSD-3-Clause |
| HTTPX | API tests | BSD-3-Clause |
| Vue | Viewer interface | MIT |
| Three.js | 3D rendering and picking | MIT |
| Lucide | Interface icons | ISC |
| Vite / Vue plugin | Frontend build | MIT |
| TypeScript | Type checking | Apache-2.0 |
| vue-tsc | Vue type checking | MIT |
| Playwright | Browser tests | Apache-2.0 |
| Eigen | C++ sparse linear algebra and spline solving | MPL-2.0 |
| LBFGSpp v0.3.0 | C++ box-constrained L-BFGS-B optimization | MIT |
| nlohmann/json | C++ JSON input and output | MIT |

Use the license files of the actual resolved dependency versions when preparing a release. Recorded maps and routes remain in the local data directory and are excluded from the source archive.

The [C++ third-party notice](cpp/third_party/NOTICE.txt) preserves upstream copyright notices and MIT terms and is installed with the C++ library. Eigen source and licensing are available at <https://gitlab.com/libeigen/eigen>; Eigen files are not modified by this project.
