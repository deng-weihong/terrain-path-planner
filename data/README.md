# 数据

[简体中文](README.md) | [English](README.en.md)

公开示例由 `src/terrain_planner/datasets.py` 生成：

- `obstacles`：24×12 m 可通行区域，两个交错缺口与绕行路线。
- `zigzag`：同尺寸平地与反复摆动的输入路线。

本地地图可放在 `local/site-map/`，包含 `manifest.json`、小端 int32 XYZ 地图和样本路线。`local/` 不进入版本管理与源码归档。

使用其他位置时设置 `PATH_PLANNER_DATA_DIR`。格式和高程约定见 [数据格式](../docs/data-format.md)。
