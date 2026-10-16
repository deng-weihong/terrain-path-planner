# 贡献说明

[简体中文](CONTRIBUTING.md) | [English](CONTRIBUTING.en.md)

修改前先阅读 [算法原理](docs/algorithm.md) 和 [项目架构](docs/architecture.md)。保持算法、可视化、接口三者的边界，不添加对业务 Server、机器人服务或本机绝对路径的依赖。

算法修改应说明具体输入、原始行为、修改后行为，以及涉及的几何约束。回归用例优先使用可重复生成的小型合成场景，外部现场数据放在 `data/local/` 并保持可选。

提交前运行：

```bash
npm test
npm run build
```

涉及曲线、约束或参数行为时补充相应回归并运行 `npm run benchmark`；涉及页面交互或三维显示时运行 `npm run test:web` 并查看截图。

不要提交依赖目录、虚拟环境、构建物、运行日志或未获授权的地图数据。结果对比应同时报告指标和校验，不只展示更短或更平滑的曲线。

公共字段或参数含义变化需要同步更新数据格式文档。中文为默认文档，修改时同步维护英文版本、代码示例和语言切换链接。C++ 与 Python 修改应同步维护共享格式、交叉回归和数值容差；原生改动需运行 `npm run build:cpp`、`npm run test:cpp` 和 `npm run test:parity`。
