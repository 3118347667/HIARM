# dynamic_grasp 真机工作区

自动起飞分支：真机在遥控器解锁后，由 CH5 触发相对高度起飞并悬停。操作、参数、接管和验证范围见 [ArduPilot 自动起飞说明](docs/auto_takeoff.md)。

飞行控制的传感器回调、控制循环和切模服务分别运行；控制循环只消费输入快照，切模结果仍以 `/mavros/state` 为准。`/flygripper/runtime_diagnostics` 以约 10 Hz 发布时间戳年龄、单调时钟回调间隔、输入序号缺口、控制耗时和切模耗时，`bags/bag.sh` 会连同 `/rosout`、`/rosout_agg` 一起录制。`*_callback_gap_ms` 对应控制快照中的接收时间，`*_live_callback_gap_ms` 对应接收线程的最新回调，可用来区分控制快照滞后与接收停顿。超过现有 IMU/EKF 新鲜度门限仍触发交接，独立接收线程不改变数据有效性要求。工作线程取消会清除排队请求并忽略过期结果，但已经发到飞控的服务请求不能撤回。

隔离 ROS 回归测试：在构建容器中加载 `devel/setup.bash` 后执行 `rostest flygripper_flight auto_takeoff.test`，覆盖自动起飞、数据失效、慢切模服务及迟到确认；这些测试使用模拟服务和输入，不证明实飞性能。

针对 IMU/EKF 消息仍到达但时间戳超过 200 ms 的退模，`run.sh` 和 `mavros.sh` 统一调用 `scripts/configure_mavlink_streams.py`：RAW_IMU/ATTITUDE 请求 150 Hz、RC_CHANNELS 50 Hz、BATTERY_STATUS 3 Hz、EXTENDED_SYS_STATE 5 Hz，默认请求关闭当前姿态输出路径未使用的 ESC 遥测和 OPTICAL_FLOW/DEBUG_VECTOR/TUNNEL。脚本在上锁、连接状态新鲜时逐项发送命令并检查 ACK；核心请求失败时停止后续启动，辅助流关闭未获确认时明确提示。`python3 scripts/configure_mavlink_streams.py --dry-run` 只显示配置；如确实需要辅助流，可用 `MAVLINK_OPTICAL_FLOW_RATE`、`MAVLINK_DEBUG_VECTOR_RATE`、`MAVLINK_TUNNEL_RATE` 设置正频率。配置 ACK 不代表实际发布率或延迟已达标，其他地面站也可能改动请求。控制循环仍为 333 Hz；EKF 按 IMU 消息时间戳计算积分步长，输出频率跟随实际 IMU 输入。

EKF 的 `~odom_publish_queue_size` 在实机 launch 中设为 10，只缩短输出状态队列；IMU 积分输入、测量时间戳与 200 ms 保护保持原有行为。这只能限制 ROS 发布队列积压，不能清除 USB/内核/TCP 缓冲区里的旧数据。运行诊断新增 `imu_age_at_callback_ms` 和 `odom_age_at_callback_ms`，表示消息到达控制节点回调时已有的年龄（也包含时间戳映射误差）；与 `*_callback_gap_ms` 合看，区分上游旧消息与回调停顿。

更新并在板端上锁启动节点后，先做 10 分钟地面录制：一个终端运行 `bash scripts/capture_board_health.sh` 保存 CPU、I/O、内核与 NetworkManager 日志，另一个终端运行 `./bags/bag.sh`。录包新增 `/mavros/imu/data_raw`、`/mavros/timesync_status`、`/mavros/time_reference` 和 `/diagnostics`，停止后检查实际消息是否录入。先确认地面没有新鲜度错误及持续高 RTT，再评估飞行验证；地面结果不替代实飞验证。这组改动不会保证消除尚未定位的板端或飞控链路延迟。

此分支于 2026-10-19 从本地 `dynamic_grasp` 工作区导入源码快照，作为与 HIARM 仿真分支无共同提交历史的独立分支。源目录没有 Git 元数据，因此不包含此前的提交历史。

- `src/`：ROS/catkin 软件包。
- `scripts/`：运行与维护脚本。
- `params/`：飞控参数文件。
- `bags/`：录制脚本和 XML 配置；录制生成的 `.bag` 数据未导入。

构建产物、日志和 macOS 元数据未导入。`src/CMakeLists.txt` 保留了源工作区的符号链接，目标为 `/opt/ros/one/share/catkin/cmake/toplevel.cmake`；构建时需要对应的 ROS 环境。

Radxa Zero 3W 的 Docker 编译与板端工作区上传说明见 [docker/ros-one-jammy/README.md](docker/ros-one-jammy/README.md)。当前分支的 17 个选定包已在本机 ARM64 容器中编译成功；真机运行效果仍需验证。

飞控重启脚本使用 `pymavlink` 直接连接开发板上的 `/dev/ttyACM0:57600`，不依赖 ROS 服务。先在开发板的 Python 3 环境安装 `pymavlink`，停止占用该串口的 MAVROS，并确认飞控已安全上锁，然后运行 `./scripts/reboot_fcu.sh`。可用 `--device`、`--baud` 指定其他串口配置。脚本收到飞控心跳并确认已上锁后，会直接发送重启命令，无需再次输入确认。该命令不重启开发板；即使收到接受回执，也需观察飞控心跳断开并重新出现，才能确认重启完成。

飞控参数导入：`params/hiarm.params` 是 ArduPilot 五列参数导出文件。SSH/ADB 推送脚本会连同 `params/` 上传到板端工作区。在板端安装 `pymavlink`、停止占用 `/dev/ttyACM0` 的 MAVROS 并确认飞控已上锁后，可先运行 `./scripts/load_fcu_params.sh --dry-run` 查看差异，再运行 `./scripts/load_fcu_params.sh` 写入。脚本逐项读取和核对参数类型，只写入不同的值；写入前将原值保存在 `params/backups/`，写入后回读确认。只读/系统状态参数会跳过；如果连接设置有差异，默认中止，需人工核对后显式传入 `--allow-link-changes`。发生中断时不会自动回滚，需根据备份核对已写入的参数。请仅对匹配的 ArduPilot 固件和硬件使用该参数文件。

飞控参数导出：同样先停止占用串口的 MAVROS，再在板端运行 `./scripts/save_fcu_params.sh`。脚本从 ArduPilot 读取完整参数列表，补读漏掉的序号；只有确认全部收到后，才将旧 `params/hiarm.params` 备份到 `params/backups/` 并替换为新的五列文件。可用 `--output` 指定其他输出文件。导出只读取飞控参数，不修改飞控。板端导出后，需要将新文件拉回 Mac；下次从 Mac 推送 `params/` 时会覆盖板端同名文件，例如在 Mac 项目目录执行 `scp rock@开发板IP:/home/rock/dynamic_grasp/params/hiarm.params ./params/hiarm.params`。

板端串口权限只需配置一次。运行脚本不保存或输入管理员密码；将代码推送到板端后，可在 root ADB shell 执行 `adb shell 'bash /home/rock/dynamic_grasp/scripts/setup_serial_permissions.sh rock'`。该脚本将 `rock` 加入 `dialout`、为飞控与执行器串口设置 udev 权限，并设置执行器所需的实时调度优先级。完成后重新连接 SSH，使新用户组和优先级限制生效。之后 `mavros.sh`、`run.sh`、`run_old.sh` 和 `fake_ctrl.sh` 不再调用 `sudo chmod 777`；串口不可访问时会给出明确错误。若 ADB 不是 root，可在板端以 root 身份执行同一脚本一次。

CAN 接口若尚未配置为 UP，仍需用 root 身份运行一次 `src/flygripper_actuator/scripts/can.sh`；运行时不会交互索取密码。

Wi-Fi 切换脚本 `./scripts/switch_wifi.sh` 只激活 NetworkManager 中已有的 `DIiffRobot_2_5G` 或 `dongbudemo` 连接配置。新板首次使用前，先在板上配置好这两个连接及其密码，再以 root 身份运行一次 `bash /home/rock/dynamic_grasp/scripts/setup_wifi_switch_sudoers.sh`，安装并校验 `/etc/sudoers.d/hiarm-wifi-switch`。如果 ADB shell 是 root，也可以从 Mac 执行 `adb shell 'bash /home/rock/dynamic_grasp/scripts/setup_wifi_switch_sudoers.sh'`。普通代码上传只复制安装脚本和规则文件，不会修改板上的 sudoers。此后 `rock` 可在两个已保存的连接间切换，无需输入 sudo 密码；上传脚本不会传输本地 `scripts/.wifi_credentials`。已在板上的旧副本不会被上传脚本删除，确认无需保留后可在板上手动删除。

无网络时，如需手动用 Mac 时间设置板端系统时钟，在 Mac 项目目录运行 `./scripts/sync_board_time_adb.sh`。脚本要求 ADB shell 为 root；连接多个设备时传入设备序列号，例如 `./scripts/sync_board_time_adb.sh 2f49935e4210e61b`。它不会修改 NTP 设置或硬件 RTC。

录制实飞数据时，在节点启动后运行 `./bags/bag.sh`。脚本固定订阅参数快照、控制器/机体/飞行状态机参数更新、ROS 日志和运行诊断；默认最多等待 30 秒加载控制器与机体参数，可用 `SNAPSHOT_TIMEOUT_SECONDS` 调整。快照失败会明确报错并停止启动录包，避免静默漏录；按 Ctrl-C 停止后会检查实际 bag 中的快照内容和三个参数更新话题，缺失时返回失败并保留 bag，ROS 日志或运行诊断无消息时另行警告。快照是录制开始时的一次初始参数，后续变化由 `parameter_updates` 记录。SSH/ADB 上传脚本均包含 `bags/` 下的录制程序，并排除录包数据；更新程序后也应同步更新板端录包脚本。
