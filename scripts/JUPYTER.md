# HIARM bag 分析环境

在仓库根目录运行：

```bash
uv sync --locked
./scripts/jupyter.sh
```

浏览器会打开 `scripts/analyze_trajectory_tracking.ipynb`。Notebook 为每段轨迹显示一张图：上方四格为 XY、X、Y、Z 跟踪，下方两行分别为横跨整行的三维速度跟踪和三维位置误差。当前分析的 bag 路径由 Notebook 的 `BAG` 单元格决定；PNG、CSV、JSON 保存在对应 bag 同目录的 `_tracking/` 输出目录。请选择项目的 Jupyter 内核运行。

也可在终端直接重算而不打开 Notebook：

```bash
uv run --locked python scripts/analyze_trajectory_tracking.py
```

依赖列在仓库根目录的 `pyproject.toml`，确切版本保存在 `uv.lock`。虚拟环境为 `.venv/`，已加入 `.gitignore`。离线 bag 分析不需要启动 ROS。
