# 开发与验证

[简体中文](development.md) | [English](en/development.md)

## 环境

按 README 创建 `.venv` 并安装 `.[web,test]`，前端执行 `npm ci`。算法基础依赖为 NumPy 和 SciPy，HTTP 与 PNG 预览属于可选依赖。

npm 中的 Python 启动器按 `PATH_PLANNER_PYTHON`、项目 `.venv`、系统 `python3` 的顺序选择解释器，并为子进程设置本项目的 `src` 路径。算法导入本身不修改 `sys.path`。

```bash
npm start -- --host 0.0.0.0 --port 5181
npm run dev
```

Vite 开发地址为 5182，API 默认代理到 5181。更改 API 开发端口时需同步调整 `vite.config.ts`。构建后的页面由 Python 接口直接提供。

## Python 测试

先按 [C++ 构建说明](cpp.md) 安装数值依赖，再执行 `npm run build:cpp` 和 `npm run test:cpp`。Python 测试会自动检测 C++ 并运行交叉回归；缺少原生可执行文件时明确跳过。CI 设置 `PATH_PLANNER_REQUIRE_CPP=1`，缺少 C++ 会失败。

```bash
npm test
```

也可以使用安装后的包执行 `python -m unittest discover -s tests -v`。覆盖范围包括：

- 搜索：地图缺口、对角支撑、明确台阶、精确端点、楼层隔离、图缓存和规划时限。
- 平滑：连续摆动、障碍绕行、地面重匹配、其他楼层隔离、支撑和失败结果不被掩盖。
- 接口：地图二进制、高程点选、端点规划、原始路线重跑、导出、无效输入和并发拒绝。
- 独立运行：屏蔽 Web 接口依赖后在独立进程中导入和搜路。

现场地图的回归测试是可选的；缺少本地 `site-map` 时明确跳过对应的 4 项测试，其他测试照常执行。强制仅使用公开示例：

```bash
PATH_PLANNER_NO_LOCAL_DATA=1 npm test
```

## 批量指标

```bash
npm run benchmark
npm run benchmark -- --include-local
npm run benchmark -- --output artifacts/my-benchmark.json
```

默认对两个合成场景分别运行保守、均衡、强平滑三组参数，保存地图摘要、参数、耗时、指标和校验。比较结果时要同时查看约束是否通过；强平滑不保证每个指标都更好。

基准输出用于复现和对比，不是性能保证。报告硬件、Python/SciPy 版本、地图规模、首次或缓存搜路、是否包含预处理，才能比较耗时。

## 浏览器检查

```bash
npm run build
npx playwright install chromium
npm run test:web
```

构建 C++ 后，使用 `PATH_PLANNER_TEST_CPP=1 npm run test:web` 额外验证桌面、笔记本和手机的 C++ 引擎切换、计算、导出、历史恢复与 WebGL 像素。CI 默认启用该检查。

检查脚本默认自动启动临时本地 API，并在完成后关闭。用已有地址：

```bash
PATH_PLANNER_URL=http://127.0.0.1:5181 npm run test:web
```

可选现场地图回归：

```bash
PATH_PLANNER_TEST_LOCAL=1 npm run test:web
```

默认检查不依赖现场数据，覆盖 1440×1000、1280×800、390×844 视口，验证真实 WebGL 像素、三维旋转、点云/体素、显示高程倍率、点选、拖动和触摸取消、参数重跑、交换端点、历史、导入导出、计算失败和地图加载重试。

截图、导出结果与检查记录位于 `artifacts/`。失败时也保存页面截图和文本。CI 中首次安装浏览器可使用 `npx playwright install --with-deps chromium`。

文档中保留了经过核验的 [桌面预览](images/desktop.png) 和 [手机预览](images/mobile.png)，均使用公开合成数据。

## 三维显示约定

Z 轴向上。全部体素中心从原始小端 int32 XYZ 解码，不用二维底图伪装高度。体素按空间块使用 `InstancedMesh`，远处小于 1.25 像素时自动使用相同完整点集，放大后恢复体素面。

路线使用固定屏幕像素宽度，以便大地图上辨识。显示时略微抬高路线，减少与地面重合；导出坐标不含显示偏移。高程倍率只改变场景显示，不改变算法输入、点选返回的原始坐标和数值指标。

三维射线返回实际命中的体素中心与高度。未命中体素时明确报错，不用平面交点代替。地图切换、重试和卸载时释放几何、材质、纹理及 WebGL 上下文。

## 常见问题

| 现象 | 检查 |
| --- | --- |
| Python 缺少 SciPy/FastAPI | 确认使用项目 `.venv`，按所需 extras 安装 |
| 创建虚拟环境提示缺少 ensurepip | Debian/Ubuntu 安装对应版本的 `python3-venv`，或使用已有可用解释器 |
| 系统 Python 来自交叉编译工具链 | 设置 `PATH_PLANNER_PYTHON` 指向可运行的本机解释器 |
| 首页返回 503 | 先执行 `npm run build` |
| 自定义地图没有出现 | 检查 manifest、目录环境变量，并重启 API |
| 页面有曲线但校验失败 | 查看新增/原有异常、边距、地面层与急转，不按画面判断可执行性 |
| WebGL 页面空白 | 查看控制台、WebGL 2 支持与地图下载状态 |
