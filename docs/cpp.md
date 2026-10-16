# C++ 使用与原理

[简体中文](cpp.md) | [English](en/cpp.md)

## 独立构建

核心算法使用 C++17、Eigen 3.3+ 和 LBFGSpp v0.3.0；JSON 适配器使用 nlohmann/json。不依赖 Python、ROS、Node.js 或 HTTP 服务。

```bash
sudo apt-get install -y build-essential cmake libeigen3-dev nlohmann-json3-dev
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build/cpp --parallel 2
ctest --test-dir build/cpp --output-on-failure
./build/cpp/terrain-plan-example
```

首次配置从上游下载 LBFGSpp v0.3.0，并校验固定的 SHA-256。已有头文件时可离线配置：

```bash
cmake -S cpp -B build/cpp -DLBFGSPP_INCLUDE_DIR=/path/to/LBFGSpp/include
```

自定义 Eigen 安装位置使用 `Eigen3_DIR`，自定义 JSON 头文件路径使用 `NLOHMANN_JSON_INCLUDE_DIR`。不支持源码目录内构建；依赖和生成物保留在 `build/`，不会进入源码归档。

安装及独立消费：

```bash
cmake --install build/cpp --prefix /tmp/terrain-planner-install
cmake -S cpp/examples/consumer -B build/consumer -DCMAKE_PREFIX_PATH=/tmp/terrain-planner-install
cmake --build build/consumer
./build/consumer/consumer
```

下游 CMake 使用 `find_package(terrain_planner CONFIG REQUIRED)`，链接 `terrain_planner::terrain_planner`。需要 JSON 时额外链接 `terrain_planner::terrain_planner_json`。

## 原生库接口

```cpp
#include <terrain_planner/planner.hpp>

using namespace terrain_planner;
std::vector<Key> keys{{0,0,0}, {1,0,0}, {2,0,0}, {3,0,0}, {4,0,0}};
GridMap map(keys, 0.1);
GraphPlanner graph(map);
auto search = graph.plan({0.05,0.05,0.05}, {0.45,0.05,0.05});
if (search.success) {
    auto result = optimize(map, search.path);
    // Read result.validation and result.solver before using result.path.
}
```

`Point` 是 `std::array<double,3>`，`Key` 是 `std::array<int32_t,3>`，`Path` 是 XYZ 点列表。公开头文件不要求包含 Eigen。`GridMap` 保存独立高度列和 XY 距离场，`GraphPlanner` 预处理三维邻接、边界与连通分量；同一个对象可查询不同端点，修改地图时重新构造。

`SearchOptions` 定义坡度、台阶、边距、分量大小、展开上限与时间上限。端点按精确体素中心检查，容差为 1e-6 m。`plan_path()` 包含预处理与搜索，合计计时；`GraphPlanner::plan()` 只计本次查询，可进一步传入更小的时间预算。返回 `success`、`status`、原始图路线、代价和耗时。默认展开上限 250,000，时间预算 60 s。

`Options` 与 Python 的四个参数相同。`OptimizationResult` 提供曲线、原路线、校验、曲率与边距指标、求解状态及保护路段。异常输入抛出标准 C++ 异常；几何检查未通过时仍返回结果和失败计数。

## JSON 命令行

```bash
./build/cpp/terrain-plan --input examples/native-request.json --output result.json
```

省略文件参数时，从标准输入读取一个 JSON 对象，向标准输出写入结果。错误写入标准错误流的 `error` 字段，退出码为 2；正常计算退出码为 0，仍需检查返回字段。

请求包含 `resolution` 和整数 XYZ 数组 `voxels`。输入是可通行地面体素，不是障碍物。其余字段为：

| 字段 | 用法 |
| --- | --- |
| `route` | 要平滑的米制 XYZ 路线；与端点互斥 |
| `start` / `goal` | 同时提供，先执行原生 XYZ 图搜索 |
| `options` | `smoothness`、`max_deviation`、`clearance`、`control_spacing` |
| `operation` | 默认 `optimize`；`search` 仅返回图搜索结果 |
| `search` | 可覆盖 `max_iterations` 和 `maximum_planning_time_ms` |
| `dataset` / `mapSha256` / `jobId` | 可选来源字段，原样写入优化结果 |

`operation=search` 即使不可达也会正常返回，必须读取 `success` 和 `status`；默认完整链路无法搜路时返回错误。搜索状态包括 `success`、`start_not_on_map`、`goal_not_on_start_component`、`no_path`、`expansion_limit` 和 `time_limit`。

地图最多 2,000,000 个输入体素，带填充的 XY 距离场最多 20,000,000 个单元。路线最多 20,000 个输入点，XY 长度为 0.1–1000 m。过大输入明确拒绝。完整优化输出与 Web 结果使用相同字段，增加 `engine: "cpp"`。

## 数值实现

图搜索保留 XYZ、对角桥接、坡度和显式台阶。词典序优先比较边界相关连通代价，再比较含距离、竖直变化、边界、转向和台阶的代价。与 Python 页面入口一致，搜索输出原始路线，随后统一做整条路线优化。

距离场采用可分离的平方欧氏距离变换，计算二维抛物线下包络，再开方和减去半个单元。查询采用双线性插值及解析梯度。高度列单独校验，不把 XY 投影中的上层地面当作当前地面。

初始三次 B-Spline 使用稀疏基矩阵及二阶导矩阵。内部控制点通过 LBFGSpp 的 L-BFGS-B 联合求解，目标项、控制点盒边界、窄端点切向、原路线走廊、保护路段和自适应边距与 Python 对应。软惩罚求解之后仍执行密集校验和修正。

最终自然平滑样条直接求解以下问题：

```text
min_f sum_i w_i * (f(s_i) - target_i)^2 + lambda * integral (f''(s))^2 ds
lambda = 0.12 * sqrt(smoothness / 6)

h_i = s_(i+1) - s_i
Q: each column has [1/h_left, -1/h_left - 1/h_right, 1/h_right]
R: diagonal (h_left+h_right)/3, off-diagonal h_right/6
(R + lambda * Q^T * W^-1 * Q) * m = Q^T * target
f = target - lambda * W^-1 * Q * m
```

Eigen 的稀疏 LDLT 求解内部二阶导 `m`，自然边界二阶导为 0。两端使用精确固定值，其他受约束位置使用增大的权重；随后重复边距投影、地形匹配和密集校验。输出来自统一的 C² 平面样条，高程仍按地面层处理。这个步骤不调用 SciPy。

## 跨语言验证

```bash
npm run build:cpp
npm run test:cpp
PATH_PLANNER_REQUIRE_CPP=1 npm test
PATH_PLANNER_TEST_CPP=1 npm run test:web
```

CTest 覆盖原生库、负坐标、距离场、精确端点、地图缺口、对角支撑、楼层、台阶、展开上限、全路径平滑和失败输入。`tests/test_native.py` 使用相同地图和参数调用两个实现，覆盖两个公开场景的三组预设，以及小高差重匹配、上层地面隔离和可选本地地图。

端点比较容差为 1e-9 m，公开场景的基线长度和最小边距核对至 1e-8；连通性与失败分类必须一致。优化结果要求无新增问题、满足走廊与采样约束，六组预设的长度差不超过 Python 结果的 5%，曲率 RMS 低于原路线的 60%。这不是逐点相等的承诺：LBFGSpp 和 SciPy 可能收敛到不同的可行曲线。对比指标保存到 `artifacts/cpp-parity.json`。

## 三维可视化

构建 C++ 后启动原有可视化，在“计算引擎”选择 C++。`GET /api/engines` 返回可用性；未构建时禁用 C++，直接请求不可用引擎返回 503。也可用 `PATH_PLANNER_CPP` 指定另一处可执行文件。适配器只传递 JSON 和地图数据，不参与原生算法求解；显式选择 C++ 后不会降级到 Python。

可视化提供相同的起终点、参数、曲率、校验、历史和导出流程。原生 C++ 库和 CLI 可以脱离这层适配器使用。最终曲线的完整三维图逐边校验、机器人外形、动态避障、速度规划和实机执行仍不在当前范围内，详见 [算法适用范围](algorithm.md)。
