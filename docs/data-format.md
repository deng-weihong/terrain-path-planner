# 数据格式与 API

[简体中文](data-format.md) | [English](en/data-format.md)

## 地图

合成数据集为 `obstacles` 和 `zigzag`。现场数据可放在 `data/local/site-map/`，也可以通过 `PATH_PLANNER_DATA_DIR` 指向一个含 `manifest.json` 的目录。源码归档不包含本地地图。

目录示例：

```text
my-map/
  manifest.json
  traversable.bin
  baseline-route.json
```

```json
{
  "id": "my-map",
  "name": "场地地图",
  "resolution": 0.1,
  "mapFile": "traversable.bin",
  "routeFile": "baseline-route.json"
}
```

`id` 使用小写字母、数字和连字符，最多 64 字符，不能与内置场景冲突。分辨率为正的有限米制数值。文件必须位于该数据集目录内。服务启动时加载数据，修改后需要重启。

`traversable.bin` 是连续的小端有符号 int32 三元组，每个体素 12 字节：`ix, iy, iz`。这些索引表示可通行地面，而不是障碍物。中心坐标为 `(index + 0.5) * resolution`。面积按唯一 XY 投影计数，同一 XY 的不同 Z 层不重复计入。

`mapSha256` 是上述 int32 二进制的 SHA-256。它不包含分辨率，因此匹配地图时还应检查分辨率和数据来源。

## 路线

所有坐标使用与地图一致的局部米制 XYZ，不是经纬度。最小格式为：

```json
[[1.05, 2.05, 0.05], [1.15, 2.05, 0.05]]
```

数据集路线文件还接受 `{"pathPoints": [...]}`，其中每点可用数组或 `{"x": 1.05, "y": 2.05, "z": 0.05}`。浏览器导入还接受 `path`、`route` 数组字段。HTTP `route` 参数使用 XYZ 数组。

HTTP 输入限制为 2–20,000 点、XY 长度 0.1–1,000 m，所有坐标有限。导入路线的 XY 点必须处于地图投影中；完整几何约束仍由后续校验判断。

## 优化参数

| 字段 | 默认值 | 范围 | 含义 |
| --- | --- | --- | --- |
| `smoothness` | 6 | 0.1–60 | 二阶导能量权重 |
| `max_deviation` | 0.8 | 0.05–3 m | 同原始弧长参数下的最大 XY 位移 |
| `clearance` | 0.39 | 0–1 m | 目标投影边距；窄处采用自适应下限 |
| `control_spacing` | 1.5 | 0.4–4 m | 初始 B-Spline 控制点间距 |

## HTTP 接口

接口文档：<http://localhost:5181/api/docs>。接口由本项目提供，不是外部服务依赖。

| 方法与路径 | 用途 |
| --- | --- |
| `GET /api/health` | 健康状态与项目标识 |
| `GET /api/engines` | Python / C++ 引擎及可用状态 |
| `GET /api/datasets` | 可用场景列表 |
| `GET /api/datasets/{id}` | 地图分辨率、范围、体素数、样本路线与指标 |
| `GET /api/datasets/{id}/voxels.bin` | 全量 XYZ int32 二进制 |
| `GET /api/datasets/{id}/map.png` | XY 投影 PNG |
| `POST /api/datasets/{id}/pick` | 端点所在体素和高度层 |
| `POST /api/optimize` | 新建异步计算任务 |
| `GET /api/jobs/{id}` | 任务阶段、状态、结果或错误 |
| `GET /api/jobs/{id}/export` | 导出完成的结果 |

三维点选：

```json
{"x": 2.05, "y": 3.05, "z": 0.05}
```

返回 `point`（体素中心 XYZ）和 `heights`（同一 XY 的所有高度层）。指定 `z` 时必须存在该层；省略时选与附近样本路线高度最接近的层。浏览器三维射线点选始终发送实际命中的 Z。

端点规划：

```json
{
  "dataset": "zigzag",
  "engine": "cpp",
  "start": [2.05, 3.05, 0.05],
  "goal": [20.05, 9.05, 0.05],
  "smoothness": 6,
  "max_deviation": 0.8,
  "clearance": 0.39,
  "control_spacing": 1.5
}
```

不提供 `start`、`goal` 时，使用 `route` 或场景样本路线。端点必须成对，且不能同时提供 `route`。成功创建返回 202 和任务 `id`；同一时间已有任务运行返回 409；无效参数返回 422，未知数据集返回 404。

`engine` 默认 `python`，也可指定 `cpp`。原生程序未构建或不可执行时返回 503，不自动降级。独立 C++ CLI 使用 `voxels`、`resolution`、`route` 或端点及嵌套 `options`，格式见 [C++ JSON 协议](cpp.md)。

轮询状态为 `running`、`completed` 或 `failed`。运行阶段为 `planning` 或 `optimizing`。任务失败后释放执行槽，可继续提交下一次计算。

## 结果字段

| 字段 | 内容 |
| --- | --- |
| `algorithm` | 优化算法版本 |
| `engine` | 实际执行的 `python` 或 `cpp`；由接口或原生 JSON 适配器补充 |
| `options` | 本次实际参数 |
| `dataset` / `mapSha256` / `jobId` | 接口任务来源 |
| `sourceRoute` | 搜索得到或导入的原始 XYZ 顶点 |
| `planning` | 图搜索引擎、精确端点、时间、展开数；未搜路时为 null |
| `reference` | 按输出参数采样的原路线对照 |
| `path` | 最终 XYZ 曲线采样 |
| `rawCandidate` | 初始 B-Spline 候选，仅供对比 |
| `baseline` / `optimized` | 原始与优化后的指标 |
| `baselineValidation` / `validation` | 原始与最终密集检查 |
| `solver` | 收敛、迭代、耗时、连续性及修正信息 |
| `scope` | 已实施的检查与尚未验证的范围 |

直接调用 `optimize()` 返回核心算法字段；`sourceRoute`、`planning` 和任务来源由接口层补充。

结果不是只有一条点列表。使用前需检查 `validation.valid`、新增与原有异常、求解状态、密集转角，以及 `scope` 中的验证范围。
