# 自动起飞的 ArduCopter SITL 闭环测试

在 Mac 项目目录执行：

```bash
./scripts/test_auto_takeoff_sitl.sh
```

要求 Docker 可用，并且已有本项目的 `dynamic-grasp:ros-one-jammy-arm64`
基础镜像。首次运行会从 ArduPilot 官方仓库检出 `Copter-4.7.2`，
编译 Linux ARM64 SITL，并安装真实 MAVROS。版本可通过 `COPTER_REF` 指定。
固件 tag、源码 commit、项目 commit 与运行中飞控报告的版本均写入结果。

运行测试的容器使用 `--network none`，不映射主机端口，也不挂载串口。
SITL、MAVROS、ROS master 和本项目飞行节点在同一容器内通过回环地址连接。
SITL SERIAL0/5760 连接 MAVROS，SERIAL1/5762 提供辅助 GCS 心跳。
模拟遥控器经 UDP/5501 向 SITL 的原生接收机发送 16 通道输入。
测试使用普通解锁命令，不强制解锁或关闭整套解锁检查。

位姿与世界系速度来自 MAVROS 转换后的 SITL 遥测，经适配器送到
`/ekf/ekf_odom`。控制器计算的姿态/推力送回实际 ArduCopter 固件；
适配器不把控制参考当作飞机状态。

采用 ArduPilot 内置四旋翼动力学模型，不模拟 HIARM 的变形机构。
该模型默认质量 3 kg、参考悬停输出 0.39，与真机配置不同，因此仅在
测试进程的 ROS 参数中把 `hover_percentage` 设为 0.39。
`params/hiarm.params` 和真机 YAML 不被覆盖。
SITL 从官方机型默认参数启动，叠加 GUID_OPTIONS=9、GUID_TIMEOUT=3、
16 个 RC 通道和 MAVLink 2 等测试配置。飞行模式通道设为 CH8，
以便本项目使用 CH5 触发自动起飞，不让 APM 的模式开关同时占用 CH5。

测试依次验证：按当前 `takeoff_height` 相对升高并稳定悬停、低油门杆不导致起飞完成后下降、
CH5 退出后确认 ALT_HOLD、第二次起飞中途取消、里程计中断时的飞控接管。
各轮之间使用 APM LAND 模式实际落地，确认地面状态后解锁下一轮。

每次运行的日志与结果位于 `.runtime_captures/apm-sitl/<时间>/`：

- `result.json`：版本、配置与各场景结果。
- `project.patch`：被测项目相对 HEAD 的已跟踪文件差异，其摘要写入结果。
- `telemetry.json`：SITL 测得的位置、速度、飞控模式及控制参考。
- `commands.json`：姿态指令中的推力值。
- `events.json`：测试阶段及飞行节点警告。
- `arducopter.log`、`mavros.log`、`flight.log`：完整原始输出。

该测试证明的是指定 SITL 模型和配置下的逻辑及闭环行为，真机推力映射、
传感器延迟、动捕丢帧与变形机构仍需对应硬件验证。

已完成的 4.7.2 测试见 [测试记录](../../docs/sitl_auto_takeoff_results.md)。
