# 代码来源及依赖

[简体中文](NOTICE.md) | [English](NOTICE.en.md)

三维图搜索源自既有机器人导航项目的路径规划实现，在此独立维护。本项目使用本地类型和坐标转换，不需要原工作区运行。全路径平滑、示例和可视化在独立目录中维护。

本项目采用 [MIT 许可证](LICENSE)，版权所有者为 deng-weihong。第三方依赖各自遵守其上游许可证，通过 pip/npm、系统包或 CMake 获取；未将依赖源码复制进本项目。

| 依赖 | 用途 | 上游许可证 |
| --- | --- | --- |
| NumPy | 数组与数值运算 | BSD-3-Clause |
| SciPy | 距离变换、样条、稀疏计算与求解 | BSD-3-Clause |
| Pillow | 可选 PNG 地图预览 | HPND |
| FastAPI | 可选 HTTP 接口 | MIT |
| Uvicorn | ASGI 服务 | BSD-3-Clause |
| HTTPX | 接口测试 | BSD-3-Clause |
| Vue | 可视化界面 | MIT |
| Three.js | 三维显示与点选 | MIT |
| Lucide | 界面图标 | ISC |
| Vite / Vue 插件 | 前端构建 | MIT |
| TypeScript | 类型检查 | Apache-2.0 |
| vue-tsc | Vue 类型检查 | MIT |
| Playwright | 浏览器测试 | Apache-2.0 |
| Eigen | C++ 稀疏线性代数与样条求解 | MPL-2.0 |
| LBFGSpp v0.3.0 | C++ L-BFGS-B 有界优化 | MIT |
| nlohmann/json | C++ JSON 输入输出 | MIT |

发行时应以实际锁定版本的许可证文件为准。现场地图与路线保留在本地数据目录，未纳入源码归档。

C++ 使用的上游版权和 MIT 授权条款保留在 [第三方声明](cpp/third_party/NOTICE.txt)，安装 C++ 库时一并安装。Eigen 源码与授权位于 <https://gitlab.com/libeigen/eigen>，Eigen 文件未在本项目中修改。
