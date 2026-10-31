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

## 移动抓取三维轨迹

打开 `scripts/analyze_moving_grasp.ipynb`，在配置单元格设置 `BAG` 后运行全部单元格。Notebook 使用录包时间同步三个话题，在 bag 同目录保存三维和 XY 平面 GIF。`START_S`、`END_S` 可截取时间范围；`PLAYBACK_SPEED` 控制播放倍速，`FPS` 控制帧率，`GIF_DPI` 控制分辨率。默认导出两个 GIF，可用 `MAKE_GIF = False` 关闭三维 GIF、`MAKE_XY_GIF = False` 关闭 XY GIF。倍速增加或帧率降低会减少绘制帧数；分辨率降低也可能加快生成，但会降低画面清晰度。GIF 以 10 ms 为单位保存帧延迟，因此实际帧率和播放时长可能与设定略有差异。XY 图中的朝向和小车速度均投影到水平面。

图中使用 `world` 坐标，三维图以 Z=0 平面表示地面，姿态箭头分别表示飞机机体 +X 和小车刚体 +Y；XY 图的小车速度箭头使用 twist 线速度。消息 `frame_id` 为空时会提示并假定与 `world` 一致，需结合实际系统核实；不执行 TF 变换。轨迹不能单独证明抓取成功。

同一个 notebook 还生成位置与水平速度时序 PNG，红线标记匹配闭爪目标的每组位置指令的首次发布时间，阴影表示从该时刻起设定的闭爪持续时间（`CLOSE_DURATION_S`，默认 0.85 秒），不证明整个区间内持续发布或实际闭合。目标自动读取规划参数快照 `grasp_position`；无快照或值不一致时会提示并跳过闭爪识别。连续发布的分组间隔可通过 `CLOSE_BURST_GAP_S` 调整。该标记不证明舵机响应或实际闭合。

小车 XY 速度先分别对 vx、vy 做一阶低通，再计算水平速度；`CAR_VELOCITY_CUTOFF_HZ` 默认为 3 Hz，设为 0 关闭。时序图同时显示原始与滤波速度，XY GIF 的速度箭头使用滤波结果。滤波以 bag 录包时间计算，超过 `MAX_GAP_S` 的间隔会重置，因此不应视作规划器内部滤波状态的精确复现。

闭爪条件证据图位于 notebook 前部，可在配置 `BAG` 后单独运行，无需生成 GIF。默认显示第一次触发前 2 秒至本次首次匹配闭爪指令后 5 秒；若没有识别到闭爪指令，显示到下一次触发或 bag 结束。没有触发记录时会提示并使用可用录包范围。多次触发可通过 `GATE_TRIGGER_INDEX` 选择，索引从 0 开始；`START_S` 和 `END_S` 会进一步限制显示范围。它读取规划器记录的 `grasp_alignment`、参数快照、飞行状态及输入话题，生成 `_grasp_gate_conditions.png`。缺失 alignment 或参数快照时，不能判定相应闭爪条件；确认实际门限后可在该单元格显式填写 `GATE_LIMITS_OVERRIDE`。绿色数值样本不代表完整闭爪门限通过，bag 录制间隔也不能证明规划器回调新鲜。

图中还比较无人机 EKF 的 yaw、小车水平速度经一级滤波后的方向、以及继续经二级滤波后的方向。滤波参数优先使用首份 bag 参数快照，缺失时使用默认假设并在输出中提示；小车曲线由 bag 离线近似重算，不是规划器内部记录的实际 yaw。低速和数据中断处不显示不可靠的方向，角度展开以避免 ±180° 处的显示跳变。
