# 代码来源及依赖

[简体中文](NOTICE.md) | [English](NOTICE.en.md)

三维图搜索源自既有机器人导航项目的路径规划实现，在此独立维护。本项目使用本地类型和坐标转换，不需要原工作区运行。全路径平滑、示例和可视化在独立目录中维护。

当前未选定本项目的许可证；第三方依赖各自遵守其上游许可证。依赖通过 pip/npm 安装，未将其源文件复制进本项目。

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

发行时应以实际锁定版本的许可证文件为准。现场地图与路线保留在本地数据目录，未纳入源码归档。
