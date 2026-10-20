# dynamic_grasp 真机工作区

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
