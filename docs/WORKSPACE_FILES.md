# HIARM 当前工作区文件导览

生成日期：2026-10-23；清点对象：Git 已跟踪文件及未被忽略的新增文件，共 **620** 个（含本导览）。

## 阅读顺序

1. 真机主链路：`scripts/run.sh` → MAVROS → `vrpn_client_ros` → `ekf_pose` → `flygripper_flight` → `flygripper_bringup` / `flygripper_actuator`。这是当前脚本的启动顺序，不等于所有节点都已在真机验证。
2. 仿真与规划：`src/flygripper_description`、`src/uav_simulator`、`src/planning`、`src/flygripper_planner`。
3. 数据与部署：`bags/`、`scripts/`、`docker/`、`params/`；Python 分析依赖由 `pyproject.toml` 和 `uv.lock` 管理。

## 模块地图

| 目录 | 主要职责 |
|---|---|
| [`src/ekf_pose/`](../src/ekf_pose/) | 将动捕或 VIO 观测与 IMU 融合为位姿/里程计 |
| [`src/vrpn_client_ros/`](../src/vrpn_client_ros/) | 从 VRPN 服务器接收动捕数据并发布 ROS 消息 |
| [`src/flygripper_flight/`](../src/flygripper_flight/) | 处理真机飞行状态机、自动起飞、控制输入和运行诊断 |
| [`src/flygripper_controller/`](../src/flygripper_controller/) | 几何飞控及抓取控制算法 |
| [`src/flygripper_actuator/`](../src/flygripper_actuator/) | 通过串口和 CAN 驱动夹爪舵机、无刷电机 |
| [`src/flygripper_bringup/`](../src/flygripper_bringup/) | 组合机器人模型、执行器和显示启动文件 |
| `src/flygripper_common/`、`src/flygripper_msgs/` | 共享 C++ 类型、工具及 ROS 消息 |
| `src/flygripper_planner/`、`src/planning/planning/`、`src/planning/poly_planner/`、`src/planning/traj_opt/` | 目标预测、抓取任务和轨迹优化 |
| [`src/flygripper_description/`](../src/flygripper_description/) | 机器人与场景的 URDF/Xacro/SDF、网格和纹理 |
| [`src/uav_simulator/`](../src/uav_simulator/) | 地图、局部感知、动力学、控制和可视化仿真 |
| `src/utils/`、`src/planning/DecompROS/` | 仿真消息、RViz 扩展与空间分解依赖 |
| [`src/flygripper_teleop/`](../src/flygripper_teleop/) | 操作输入到机器人命令的映射 |

## 统计与边界

下表逐个列出 620 个项目文件。说明依据路径、文件类型、文件内容和仓库 README；短句只描述静态职责，不宣称运行效果。源码、模型与第三方子项目中的细节需要沿调用链继续阅读。
`build/`、`build_catkin/`、`devel/`、`logs_catkin/`、`.venv/`、`.git/`、运行缓存和输出目录含大量生成/依赖文件，未逐个写人工用途；它们的目录角色见文末。

## 仓库根目录（7 个）

| 文件 | 作用概况 |
|---|---|
| [`.python-version`](../.python-version) | 项目 Python 解释器版本选择 |
| [`pyproject.toml`](../pyproject.toml) | 离线数据分析/Jupyter 项目的 Python 版本和直接依赖 |
| [`uv.lock`](../uv.lock) | uv 解析后的 Python 依赖版本、来源与校验信息 |
| [`.catkin_workspace`](../.catkin_workspace) | 标记 catkin 工作区的文件 |
| [`.gitattributes`](../.gitattributes) | Git 文件属性与文本处理规则 |
| [`.gitignore`](../.gitignore) | Git 忽略规则，排除构建结果、环境和运行数据 |
| [`README.md`](../README.md) | 工作区总说明：真机启动、录包、参数、板端部署和验证边界 |

## bags（10 个）

| 文件 | 作用概况 |
|---|---|
| [`bags/test_param_recording.py`](../bags/test_param_recording.py) | 参数录制逻辑的测试程序 |
| [`bags/verify_param_recording.py`](../bags/verify_param_recording.py) | 检查 bag 中参数快照及更新记录 |
| [`bags/bag.sh`](../bags/bag.sh) | 真机 ROS topic 录制与参数快照校验入口 |
| [`bags/bollon.xml`](../bags/bollon.xml) | PlotJuggler 曲线布局，展示触觉、VINS 和执行器等数据 |
| [`bags/control.xml`](../bags/control.xml) | PlotJuggler 曲线布局，展示控制位置、外力等数据 |
| [`bags/publish_param_snapshot.py`](../bags/publish_param_snapshot.py) | 发布录制起点的 ROS 参数快照 |
| [`bags/record_cali_bag.sh`](../bags/record_cali_bag.sh) | 标定数据录制脚本 |
| [`bags/record_nokov_vins.sh`](../bags/record_nokov_vins.sh) | Nokov 与 VINS 数据录制脚本 |
| [`bags/record_yolo.sh`](../bags/record_yolo.sh) | YOLO 感知数据录制脚本 |
| [`bags/tact.xml`](../bags/tact.xml) | PlotJuggler 曲线布局，展示触觉、位置、推力和执行器数据 |

## scripts（56 个）

| 文件 | 作用概况 |
|---|---|
| [`scripts/JUPYTER.md`](../scripts/JUPYTER.md) | Jupyter 数据分析环境说明 |
| [`scripts/analyze_trajectory_tracking.ipynb`](../scripts/analyze_trajectory_tracking.ipynb) | 轨迹跟踪分析的 Jupyter Notebook |
| [`scripts/analyze_trajectory_tracking.py`](../scripts/analyze_trajectory_tracking.py) | 分析轨迹跟踪误差并绘图 |
| [`scripts/capture_board_health.sh`](../scripts/capture_board_health.sh) | 采集板端 CPU、I/O、网络和内核运行状态 |
| [`scripts/configure_mavlink_streams.py`](../scripts/configure_mavlink_streams.py) | 通过 MAVLink 命令配置所需消息流并检查 ACK |
| [`scripts/jupyter.sh`](../scripts/jupyter.sh) | 启动项目 Jupyter 环境 |
| [`scripts/test_mavlink_streams.py`](../scripts/test_mavlink_streams.py) | MAVLink 流配置的离线契约测试；可选 ROS CLI 检查 |
| [`scripts/99-cameras.rules`](../scripts/99-cameras.rules) | 相机设备 udev 权限规则 |
| [`scripts/PROFILE_RUNTIME.md`](../scripts/PROFILE_RUNTIME.md) | 运行性能采集与分析说明 |
| [`scripts/alias.zsh`](../scripts/alias.zsh) | 交互终端快捷命令定义 |
| [`scripts/battery_voltage.py`](../scripts/battery_voltage.py) | 读取并显示电池电压的 Python 实现 |
| [`scripts/battery_voltage.sh`](../scripts/battery_voltage.sh) | 启动电池电压读取程序 |
| [`scripts/build_ros_one_arm64.sh`](../scripts/build_ros_one_arm64.sh) | 构建 ARM64 ROS One 工作区镜像/软件包 |
| [`scripts/ctrl.sh`](../scripts/ctrl.sh) | 启动动捕/EKF（或仿真重置）及飞行控制节点 |
| [`scripts/eight.sh`](../scripts/eight.sh) | 启动八字轨迹任务 |
| [`scripts/ekf.sh`](../scripts/ekf.sh) | 启动 MAVROS、配置 IMU/姿态流，再启动动捕与 EKF |
| [`scripts/fake_ctrl.sh`](../scripts/fake_ctrl.sh) | 启动模拟控制输入/控制链路 |
| [`scripts/land_triger.sh`](../scripts/land_triger.sh) | 触发降落命令的辅助脚本 |
| [`scripts/load_fcu_params.py`](../scripts/load_fcu_params.py) | 读取、比对、备份并写入飞控参数 |
| [`scripts/load_fcu_params.sh`](../scripts/load_fcu_params.sh) | 飞控参数导入入口 |
| [`scripts/mavros.sh`](../scripts/mavros.sh) | 单独启动 MAVROS 和遥测流配置 |
| [`scripts/perception.sh`](../scripts/perception.sh) | 启动感知链路 |
| [`scripts/plan.sh`](../scripts/plan.sh) | 启动规划节点 |
| [`scripts/profile_system.py`](../scripts/profile_system.py) | 采集系统运行性能数据 |
| [`scripts/pub_morph.sh`](../scripts/pub_morph.sh) | 发布夹爪形态控制命令 |
| [`scripts/pub_release.sh`](../scripts/pub_release.sh) | 发布夹爪释放命令 |
| [`scripts/pub_triger.sh`](../scripts/pub_triger.sh) | 发布触发命令 |
| [`scripts/push_ros_one_arm64.sh`](../scripts/push_ros_one_arm64.sh) | 通过 SSH 上传工作区到开发板 |
| [`scripts/push_ros_one_arm64_adb.sh`](../scripts/push_ros_one_arm64_adb.sh) | 通过 ADB 上传工作区到开发板 |
| [`scripts/reboot_fcu.py`](../scripts/reboot_fcu.py) | 发送飞控重启 MAVLink 命令 |
| [`scripts/reboot_fcu.sh`](../scripts/reboot_fcu.sh) | 飞控重启入口 |
| [`scripts/require_serial_access.sh`](../scripts/require_serial_access.sh) | 检查当前用户对串口设备的访问权限 |
| [`scripts/rockchip/freq_set_status`](../scripts/rockchip/freq_set_status) | Rockchip CPU 频率设置/查看工具 |
| [`scripts/rockchip/scaling_frequency.sh`](../scripts/rockchip/scaling_frequency.sh) | Rockchip CPU 频率设置脚本 |
| [`scripts/rockchip/temperature.sh`](../scripts/rockchip/temperature.sh) | 读取 Rockchip 板端温度 |
| [`scripts/rockchip/wlan_kernel5.ko`](../scripts/rockchip/wlan_kernel5.ko) | Rockchip Wi-Fi 内核模块二进制文件；文件名标明内核版本 |
| [`scripts/rockchip/wlan_kernel6.1.43.ko`](../scripts/rockchip/wlan_kernel6.1.43.ko) | Rockchip Wi-Fi 内核模块二进制文件；文件名标明内核版本 |
| [`scripts/run.sh`](../scripts/run.sh) | 真机主启动入口，串联 MAVROS、动捕、EKF、飞行节点与 bringup |
| [`scripts/run_old.sh`](../scripts/run_old.sh) | 旧版真机主启动流程，供对照 |
| [`scripts/save_eight_trajectory.py`](../scripts/save_eight_trajectory.py) | 保存八字飞行轨迹数据 |
| [`scripts/save_eight_trajectory.sh`](../scripts/save_eight_trajectory.sh) | 八字轨迹保存入口 |
| [`scripts/save_fcu_params.py`](../scripts/save_fcu_params.py) | 从飞控导出参数并备份原文件 |
| [`scripts/save_fcu_params.sh`](../scripts/save_fcu_params.sh) | 飞控参数导出入口 |
| [`scripts/servo.sh`](../scripts/servo.sh) | 启动 flygripper_bringup/run.launch 所组合的模型与执行器 |
| [`scripts/setup_fastsam.sh`](../scripts/setup_fastsam.sh) | 准备 FastSAM 相关环境 |
| [`scripts/setup_serial_permissions.sh`](../scripts/setup_serial_permissions.sh) | 安装串口 udev 规则和用户权限 |
| [`scripts/setup_wifi_switch_sudoers.sh`](../scripts/setup_wifi_switch_sudoers.sh) | 安装 Wi-Fi 切换专用 sudoers 规则 |
| [`scripts/sitl/test_auto_takeoff.py`](../scripts/sitl/test_auto_takeoff.py) | SITL 自动起飞场景驱动与结果检查 |
| [`scripts/summarize_runtime.py`](../scripts/summarize_runtime.py) | 汇总运行性能记录 |
| [`scripts/switch_wifi.sh`](../scripts/switch_wifi.sh) | 切换板端已保存的 Wi-Fi 连接 |
| [`scripts/sync_board_time_adb.sh`](../scripts/sync_board_time_adb.sh) | 通过 ADB 设置板端系统时间 |
| [`scripts/sync_board_time_ssh.sh`](../scripts/sync_board_time_ssh.sh) | 通过 SSH 设置板端系统时间 |
| [`scripts/tact.sh`](../scripts/tact.sh) | 启动触觉相关节点 |
| [`scripts/test_auto_takeoff_sitl.sh`](../scripts/test_auto_takeoff_sitl.sh) | 构建并运行 ArduCopter SITL 自动起飞检查 |
| [`scripts/traj.sh`](../scripts/traj.sh) | 启动轨迹任务 |
| [`scripts/wifi_switch.sudoers`](../scripts/wifi_switch.sudoers) | 允许特定 Wi-Fi 切换命令的 sudoers 规则 |

## src/flygripper_flight（34 个）

真机飞行状态机、起飞与控制节点。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_flight/include/flygripper_flight/runtime_io.hpp`](../src/flygripper_flight/include/flygripper_flight/runtime_io.hpp) | 运行时输入输出的接口/数据结构声明；定义 ModeWorker、Status、ScopeExit |
| [`src/flygripper_flight/test/test_runtime_io.cpp`](../src/flygripper_flight/test/test_runtime_io.cpp) | 运行时输入输出相关测试源码 |
| [`src/flygripper_flight/CMakeLists.txt`](../src/flygripper_flight/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_flight/README.md`](../src/flygripper_flight/README.md) | 该目录的说明文档 |
| [`src/flygripper_flight/cfg/RosFsmParameter.cfg`](../src/flygripper_flight/cfg/RosFsmParameter.cfg) | 飞行状态机动态参数定义或配置 |
| [`src/flygripper_flight/include/flygripper_flight/admittance_control.hpp`](../src/flygripper_flight/include/flygripper_flight/admittance_control.hpp) | 导纳控制的接口/数据结构声明；定义 AdmittanceControlSystem、State、Input |
| [`src/flygripper_flight/include/flygripper_flight/external_wrench_estimator.hpp`](../src/flygripper_flight/include/flygripper_flight/external_wrench_estimator.hpp) | 外力估计的接口/数据结构声明；定义 WrenchBuffer、ExternalWrenchEstimator、AppliedState |
| [`src/flygripper_flight/include/flygripper_flight/fsm.hpp`](../src/flygripper_flight/include/flygripper_flight/fsm.hpp) | 飞行状态机的接口/数据结构声明；定义 Fsm、Message、InputSlot |
| [`src/flygripper_flight/include/flygripper_flight/fsm_parm.hpp`](../src/flygripper_flight/include/flygripper_flight/fsm_parm.hpp) | 飞行状态机的接口/数据结构声明；定义 AdmittanceMode、FsmParameter、FsmParameterConfig |
| [`src/flygripper_flight/include/flygripper_flight/rc_utils.hpp`](../src/flygripper_flight/include/flygripper_flight/rc_utils.hpp) | 遥控器输入的接口/数据结构声明；定义 ToggleSwitch、RcInput |
| [`src/flygripper_flight/include/flygripper_flight/rpm_map.hpp`](../src/flygripper_flight/include/flygripper_flight/rpm_map.hpp) | 转速映射的接口/数据结构声明；定义 RpmMap、MapType |
| [`src/flygripper_flight/include/flygripper_flight/sampler.hpp`](../src/flygripper_flight/include/flygripper_flight/sampler.hpp) | “sampler”模块的接口/数据结构声明；定义 Sampler |
| [`src/flygripper_flight/include/flygripper_flight/takeoff.hpp`](../src/flygripper_flight/include/flygripper_flight/takeoff.hpp) | 起飞的接口/数据结构声明；定义 TakeoffPlan、Settings、Sample |
| [`src/flygripper_flight/include/flygripper_flight/thrust_acc_map.hpp`](../src/flygripper_flight/include/flygripper_flight/thrust_acc_map.hpp) | 推力与加速度映射的接口/数据结构声明；定义 ThrustAccMap、MapType |
| [`src/flygripper_flight/include/flygripper_flight/velocity_control.hpp`](../src/flygripper_flight/include/flygripper_flight/velocity_control.hpp) | 速度控制的接口/数据结构声明；定义 VelocityControlSystem、State、Input |
| [`src/flygripper_flight/launch/calibr.launch`](../src/flygripper_flight/launch/calibr.launch) | ROS 启动文件：“calibr”模块；节点包 flygripper_flight、rosbag |
| [`src/flygripper_flight/launch/flight.launch`](../src/flygripper_flight/launch/flight.launch) | ROS 启动文件：“flight”模块；节点包 flygripper_flight |
| [`src/flygripper_flight/launch/flight_root.launch`](../src/flygripper_flight/launch/flight_root.launch) | ROS 启动文件：“flight_root”模块；节点包 flygripper_flight |
| [`src/flygripper_flight/package.xml`](../src/flygripper_flight/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_flight/params/flygripper.yaml`](../src/flygripper_flight/params/flygripper.yaml) | “flygripper”模块配置参数；顶层键：robot_parameter |
| [`src/flygripper_flight/params/flygripper_sim.yaml`](../src/flygripper_flight/params/flygripper_sim.yaml) | “flygripper_sim”模块配置参数；顶层键：robot_parameter |
| [`src/flygripper_flight/params/flygripper_simple_sim.yaml`](../src/flygripper_flight/params/flygripper_simple_sim.yaml) | “flygripper_simple_sim”模块配置参数；顶层键：robot_parameter |
| [`src/flygripper_flight/params/fsm.yaml`](../src/flygripper_flight/params/fsm.yaml) | 飞行状态机配置参数；顶层键：loop_frequency、loop_time_error_threshold、imu_timeout、odom_timeout、sensor_future_tolerance |
| [`src/flygripper_flight/params/fsm_sim.yaml`](../src/flygripper_flight/params/fsm_sim.yaml) | 飞行状态机配置参数；顶层键：loop_frequency、loop_time_error_threshold、imu_timeout、odom_timeout、rotor_speed_timeout |
| [`src/flygripper_flight/params/hummingbird_sim.yaml`](../src/flygripper_flight/params/hummingbird_sim.yaml) | “hummingbird_sim”模块配置参数；顶层键：robot_parameter |
| [`src/flygripper_flight/scripts/launch_with_root.sh`](../src/flygripper_flight/scripts/launch_with_root.sh) | “launch_with_root”模块的 shell 启动/配置脚本 |
| [`src/flygripper_flight/src/calibr.cpp`](../src/flygripper_flight/src/calibr.cpp) | “calibr”模块的实现源码 |
| [`src/flygripper_flight/src/calibr2.cpp`](../src/flygripper_flight/src/calibr2.cpp) | “calibr2”模块的实现源码 |
| [`src/flygripper_flight/src/flygripper_flight_node.cpp`](../src/flygripper_flight/src/flygripper_flight_node.cpp) | “flygripper_flight_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/flygripper_flight/src/fsm.cpp`](../src/flygripper_flight/src/fsm.cpp) | 飞行状态机的实现源码 |
| [`src/flygripper_flight/src/test.cpp`](../src/flygripper_flight/src/test.cpp) | “test”模块相关测试源码 |
| [`src/flygripper_flight/test/auto_takeoff.test`](../src/flygripper_flight/test/auto_takeoff.test) | rostest 测试启动文件：起飞 |
| [`src/flygripper_flight/test/test_auto_takeoff.py`](../src/flygripper_flight/test/test_auto_takeoff.py) | 起飞的 Python 实现或测试 |
| [`src/flygripper_flight/test/test_takeoff.cpp`](../src/flygripper_flight/test/test_takeoff.cpp) | 起飞相关测试源码 |

## docker（5 个）

| 文件 | 作用概况 |
|---|---|
| [`docker/arducopter-sitl/Dockerfile`](../docker/arducopter-sitl/Dockerfile) | 构建该目录所述 Docker 镜像的步骤 |
| [`docker/arducopter-sitl/README.md`](../docker/arducopter-sitl/README.md) | 该目录的说明文档 |
| [`docker/ros-one-jammy/.dockerignore`](../docker/ros-one-jammy/.dockerignore) | Docker 构建上下文排除规则 |
| [`docker/ros-one-jammy/Dockerfile`](../docker/ros-one-jammy/Dockerfile) | 构建该目录所述 Docker 镜像的步骤 |
| [`docker/ros-one-jammy/README.md`](../docker/ros-one-jammy/README.md) | 该目录的说明文档 |

## docs（3 个）

| 文件 | 作用概况 |
|---|---|
| [`docs/auto_takeoff.md`](../docs/auto_takeoff.md) | 自动起飞操作与安全验证说明 |
| [`docs/sitl_auto_takeoff_results.md`](../docs/sitl_auto_takeoff_results.md) | SITL 自动起飞结果记录 |
| [`docs/WORKSPACE_FILES.md`](../docs/WORKSPACE_FILES.md) | 当前工作区项目文件的逐项用途导览（本文件） |

## params（1 个）

| 文件 | 作用概况 |
|---|---|
| [`params/hiarm.params`](../params/hiarm.params) | ArduPilot 五列飞控参数导出文件；用于核对或导入 |

## src/.rosinstall（1 个）

| 文件 | 作用概况 |
|---|---|
| [`src/.rosinstall`](../src/.rosinstall) | ROS 源仓库拉取配置 |

## src/.rosinstall.bak（1 个）

| 文件 | 作用概况 |
|---|---|
| [`src/.rosinstall.bak`](../src/.rosinstall.bak) | ROS 源仓库拉取配置的备份 |

## src/CMakeLists.txt（1 个）

| 文件 | 作用概况 |
|---|---|
| [`src/CMakeLists.txt`](../src/CMakeLists.txt) | catkin 工作区顶层 CMake 入口；指向 /opt/ros/one/share/catkin/cmake/toplevel.cmake，当前 Mac 上目标不存在 |

## src/ekf_pose（19 个）

动捕/VIO 与 IMU 融合定位。

| 文件 | 作用概况 |
|---|---|
| [`src/ekf_pose/CMakeLists.txt`](../src/ekf_pose/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/ekf_pose/cmake/FindEigen3.cmake`](../src/ekf_pose/cmake/FindEigen3.cmake) | CMake 构建/模板辅助文件 |
| [`src/ekf_pose/cmake/FindGLIB2.cmake`](../src/ekf_pose/cmake/FindGLIB2.cmake) | CMake 构建/模板辅助文件 |
| [`src/ekf_pose/cmake/FindGTHREAD2.cmake`](../src/ekf_pose/cmake/FindGTHREAD2.cmake) | CMake 构建/模板辅助文件 |
| [`src/ekf_pose/config/TA-camera.yml`](../src/ekf_pose/config/TA-camera.yml) | “TA-camera”模块配置参数；顶层键：calibration_time、nframes、image_width、image_height、board_width |
| [`src/ekf_pose/config/ekf_pose.rviz`](../src/ekf_pose/config/ekf_pose.rviz) | “ekf_pose”模块的 RViz 显示布局 |
| [`src/ekf_pose/config/tag_1.yml`](../src/ekf_pose/config/tag_1.yml) | “tag_1”模块配置参数；顶层键：aruco_bc_nmarkers、aruco_bc_mInfoType、aruco_bc_markers |
| [`src/ekf_pose/include/conversion.h`](../src/ekf_pose/include/conversion.h) | 数据格式转换的接口/数据结构声明 |
| [`src/ekf_pose/include/ekf.h`](../src/ekf_pose/include/ekf.h) | “ekf”模块的接口/数据结构声明；定义 ekf_estimation |
| [`src/ekf_pose/launch/PX4_vio_drone.yaml`](../src/ekf_pose/launch/PX4_vio_drone.yaml) | 视觉惯性里程计配置参数；顶层键：Rr_i、tr_i |
| [`src/ekf_pose/launch/nokov.launch`](../src/ekf_pose/launch/nokov.launch) | ROS 启动文件：“nokov”模块；节点包 ekf |
| [`src/ekf_pose/package.xml`](../src/ekf_pose/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/ekf_pose/src/conversion.cpp`](../src/ekf_pose/src/conversion.cpp) | 数据格式转换的实现源码 |
| [`src/ekf_pose/src/ekf_node.cpp`](../src/ekf_pose/src/ekf_node.cpp) | “ekf_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/ekf_pose/src/ekf_node_mocap.cpp`](../src/ekf_pose/src/ekf_node_mocap.cpp) | 动捕的实现源码 |
| [`src/ekf_pose/src/ekf_node_vio.cpp`](../src/ekf_pose/src/ekf_node_vio.cpp) | 视觉惯性里程计的实现源码 |
| [`src/ekf_pose/src/ekf_node_vio_timesync.cpp`](../src/ekf_pose/src/ekf_node_vio_timesync.cpp) | 时间同步、视觉惯性里程计的实现源码 |
| [`src/ekf_pose/src/ekf_node_vio_timesync_with_acc_pub.cpp`](../src/ekf_pose/src/ekf_node_vio_timesync_with_acc_pub.cpp) | 时间同步、视觉惯性里程计的实现源码 |
| [`src/ekf_pose/src/imu_add_noise.cpp`](../src/ekf_pose/src/imu_add_noise.cpp) | IMU的实现源码 |

## src/flygripper_actuator（38 个）

夹爪舵机与无刷电机驱动。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_actuator/CMakeLists.txt`](../src/flygripper_actuator/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_actuator/README.md`](../src/flygripper_actuator/README.md) | 该目录的说明文档 |
| [`src/flygripper_actuator/cfg/BLDC.cfg`](../src/flygripper_actuator/cfg/BLDC.cfg) | “BLDC”模块动态参数定义或配置 |
| [`src/flygripper_actuator/cfg/Servo.cfg`](../src/flygripper_actuator/cfg/Servo.cfg) | “Servo”模块动态参数定义或配置 |
| [`src/flygripper_actuator/external/feetech_servo/INST.h`](../src/flygripper_actuator/external/feetech_servo/INST.h) | “INST”模块的接口/数据结构声明 |
| [`src/flygripper_actuator/external/feetech_servo/SCS.cpp`](../src/flygripper_actuator/external/feetech_servo/SCS.cpp) | “SCS”模块的实现源码 |
| [`src/flygripper_actuator/external/feetech_servo/SCS.h`](../src/flygripper_actuator/external/feetech_servo/SCS.h) | “SCS”模块的接口/数据结构声明；定义 SCS |
| [`src/flygripper_actuator/external/feetech_servo/SCSerial.cpp`](../src/flygripper_actuator/external/feetech_servo/SCSerial.cpp) | 串口通信的实现源码 |
| [`src/flygripper_actuator/external/feetech_servo/SCSerial.h`](../src/flygripper_actuator/external/feetech_servo/SCSerial.h) | 串口通信的接口/数据结构声明；定义 SCSerial、termios |
| [`src/flygripper_actuator/external/feetech_servo/SMS_STS.cpp`](../src/flygripper_actuator/external/feetech_servo/SMS_STS.cpp) | “SMS_STS”模块的实现源码 |
| [`src/flygripper_actuator/external/feetech_servo/SMS_STS.h`](../src/flygripper_actuator/external/feetech_servo/SMS_STS.h) | “SMS_STS”模块的接口/数据结构声明；定义 SMS_STS |
| [`src/flygripper_actuator/include/flygripper_actuator/bldc/bldc.hpp`](../src/flygripper_actuator/include/flygripper_actuator/bldc/bldc.hpp) | “bldc”模块的接口/数据结构声明；定义 ActuatorState、ActuatorCommand、ActuatorParam |
| [`src/flygripper_actuator/include/flygripper_actuator/bldc/bldc_control.hpp`](../src/flygripper_actuator/include/flygripper_actuator/bldc/bldc_control.hpp) | 无刷电机控制的接口/数据结构声明；定义 ActuatorControl |
| [`src/flygripper_actuator/include/flygripper_actuator/bldc/bldc_control_param.hpp`](../src/flygripper_actuator/include/flygripper_actuator/bldc/bldc_control_param.hpp) | 无刷电机控制的接口/数据结构声明；定义 Parameters、ParameterConfig |
| [`src/flygripper_actuator/include/flygripper_actuator/bldc/can_bldc.hpp`](../src/flygripper_actuator/include/flygripper_actuator/bldc/can_bldc.hpp) | “can_bldc”模块的接口/数据结构声明；定义 StampedCanFrame、RMCommandCanFrame、CanActuator |
| [`src/flygripper_actuator/include/flygripper_actuator/hardware/can_base.hpp`](../src/flygripper_actuator/include/flygripper_actuator/hardware/can_base.hpp) | CAN 接口基类的接口/数据结构声明；定义 CanBase |
| [`src/flygripper_actuator/include/flygripper_actuator/hardware/serial.hpp`](../src/flygripper_actuator/include/flygripper_actuator/hardware/serial.hpp) | 串口通信的接口/数据结构声明；定义 Serial、termios |
| [`src/flygripper_actuator/include/flygripper_actuator/hardware/socket_can.hpp`](../src/flygripper_actuator/include/flygripper_actuator/hardware/socket_can.hpp) | SocketCAN 通信的接口/数据结构声明；定义 SocketCan、sockaddr、timeval |
| [`src/flygripper_actuator/include/flygripper_actuator/hardware/uart_can.hpp`](../src/flygripper_actuator/include/flygripper_actuator/hardware/uart_can.hpp) | UART-CAN 通信的接口/数据结构声明；定义 for、UartCanAdaptor、UartCan |
| [`src/flygripper_actuator/include/flygripper_actuator/servo/feetech_servo.hpp`](../src/flygripper_actuator/include/flygripper_actuator/servo/feetech_servo.hpp) | 飞特舵机协议封装的接口/数据结构声明；定义 FeetechServo |
| [`src/flygripper_actuator/include/flygripper_actuator/servo/servo.hpp`](../src/flygripper_actuator/include/flygripper_actuator/servo/servo.hpp) | “servo”模块的接口/数据结构声明；定义 ActuatorState、ActuatorCommand、ActuatorParam |
| [`src/flygripper_actuator/include/flygripper_actuator/servo/servo_control.hpp`](../src/flygripper_actuator/include/flygripper_actuator/servo/servo_control.hpp) | 舵机控制的接口/数据结构声明；定义 ActuatorControl |
| [`src/flygripper_actuator/include/flygripper_actuator/servo/servo_control_param.hpp`](../src/flygripper_actuator/include/flygripper_actuator/servo/servo_control_param.hpp) | 舵机控制的接口/数据结构声明；定义 TeleopMode、Parameters、ParameterConfig |
| [`src/flygripper_actuator/include/flygripper_actuator/utils/circular_buffer.hpp`](../src/flygripper_actuator/include/flygripper_actuator/utils/circular_buffer.hpp) | 遥控器输入的接口/数据结构声明；定义 T、CircularBuffer |
| [`src/flygripper_actuator/launch/actuator_root.launch`](../src/flygripper_actuator/launch/actuator_root.launch) | ROS 启动文件：“actuator_root”模块；节点包 flygripper_actuator；包含其他 launch |
| [`src/flygripper_actuator/launch/bldc.launch`](../src/flygripper_actuator/launch/bldc.launch) | ROS 启动文件：“bldc”模块；节点包 flygripper_actuator |
| [`src/flygripper_actuator/launch/servo.launch`](../src/flygripper_actuator/launch/servo.launch) | ROS 启动文件：“servo”模块；节点包 flygripper_actuator |
| [`src/flygripper_actuator/package.xml`](../src/flygripper_actuator/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_actuator/params/bldc.yaml`](../src/flygripper_actuator/params/bldc.yaml) | “bldc”模块配置参数；顶层键：actuator_id、actuator_product、actuator_joint_name、enable_joint_state_publisher、actuators |
| [`src/flygripper_actuator/params/bldc_control.yaml`](../src/flygripper_actuator/params/bldc_control.yaml) | 无刷电机控制配置参数；顶层键：loop_frequency、loop_time_error_threshold、k_p、k_i、k_d |
| [`src/flygripper_actuator/params/servo.yaml`](../src/flygripper_actuator/params/servo.yaml) | “servo”模块配置参数；顶层键：baud_rate、actuator_id、actuator_product、actuator_joint_name、enable_joint_state_publisher |
| [`src/flygripper_actuator/params/servo_control.yaml`](../src/flygripper_actuator/params/servo_control.yaml) | 舵机控制配置参数；顶层键：loop_frequency、loop_time_error_threshold、teleop_mode、reference_position、one_morphing_angle |
| [`src/flygripper_actuator/scripts/bldc_udev.sh`](../src/flygripper_actuator/scripts/bldc_udev.sh) | “bldc_udev”模块的 shell 启动/配置脚本 |
| [`src/flygripper_actuator/scripts/can.sh`](../src/flygripper_actuator/scripts/can.sh) | “can”模块的 shell 启动/配置脚本 |
| [`src/flygripper_actuator/scripts/feetech_udev.sh`](../src/flygripper_actuator/scripts/feetech_udev.sh) | “feetech_udev”模块的 shell 启动/配置脚本 |
| [`src/flygripper_actuator/scripts/launch_with_root.sh`](../src/flygripper_actuator/scripts/launch_with_root.sh) | “launch_with_root”模块的 shell 启动/配置脚本 |
| [`src/flygripper_actuator/src/bldc_node.cpp`](../src/flygripper_actuator/src/bldc_node.cpp) | “bldc_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/flygripper_actuator/src/servo_node.cpp`](../src/flygripper_actuator/src/servo_node.cpp) | “servo_node”模块的 ROS 节点/Nodelet 实现 |

## src/flygripper_bringup（7 个）

整机模型、执行器与 RViz 启动。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_bringup/CMakeLists.txt`](../src/flygripper_bringup/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_bringup/launch/actuator.launch`](../src/flygripper_bringup/launch/actuator.launch) | ROS 启动文件：“actuator”模块；节点包 rqt_gui；包含其他 launch |
| [`src/flygripper_bringup/launch/reset_world.launch`](../src/flygripper_bringup/launch/reset_world.launch) | ROS 启动文件：“reset_world”模块；节点包 rosservice |
| [`src/flygripper_bringup/launch/run.launch`](../src/flygripper_bringup/launch/run.launch) | ROS 启动文件：“run”模块；包含其他 launch |
| [`src/flygripper_bringup/launch/urdf.launch`](../src/flygripper_bringup/launch/urdf.launch) | ROS 启动文件：“urdf”模块；节点包 rviz；包含其他 launch |
| [`src/flygripper_bringup/package.xml`](../src/flygripper_bringup/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_bringup/rviz/default.rviz`](../src/flygripper_bringup/rviz/default.rviz) | “default”模块的 RViz 显示布局 |

## src/flygripper_common（40 个）

机器人共享类型、数学、轨迹和 ROS 工具。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_common/CMakeLists.txt`](../src/flygripper_common/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_common/cfg/RosRobotParameter.cfg`](../src/flygripper_common/cfg/RosRobotParameter.cfg) | “RosRobotParameter”模块动态参数定义或配置 |
| [`src/flygripper_common/cfg/RosTestParameter.cfg`](../src/flygripper_common/cfg/RosTestParameter.cfg) | “RosTestParameter”模块动态参数定义或配置 |
| [`src/flygripper_common/include/flygripper_common/controller/pid.hpp`](../src/flygripper_common/include/flygripper_common/controller/pid.hpp) | “pid”模块的接口/数据结构声明；定义 PidParams、Pid |
| [`src/flygripper_common/include/flygripper_common/filter/butterworth.hpp`](../src/flygripper_common/include/flygripper_common/filter/butterworth.hpp) | 巴特沃斯滤波的接口/数据结构声明；定义 ButterworthLPF1D、ButterworthLPF |
| [`src/flygripper_common/include/flygripper_common/intergrator/integrator_base.h`](../src/flygripper_common/include/flygripper_common/intergrator/integrator_base.h) | 数值积分的接口/数据结构声明；定义 IntegratorBase |
| [`src/flygripper_common/include/flygripper_common/intergrator/integrator_rk4.h`](../src/flygripper_common/include/flygripper_common/intergrator/integrator_rk4.h) | 数值积分的接口/数据结构声明；定义 IntegratorRK4 |
| [`src/flygripper_common/include/flygripper_common/quadrotor/flatness.hpp`](../src/flygripper_common/include/flygripper_common/quadrotor/flatness.hpp) | 微分平坦性的接口/数据结构声明；定义 Flatness |
| [`src/flygripper_common/include/flygripper_common/ros/callback.hpp`](../src/flygripper_common/include/flygripper_common/ros/callback.hpp) | “callback”模块的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/ros/conversions.hpp`](../src/flygripper_common/include/flygripper_common/ros/conversions.hpp) | 数据格式转换的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/ros/dynamic_parameter_config.hpp`](../src/flygripper_common/include/flygripper_common/ros/dynamic_parameter_config.hpp) | “dynamic_parameter_config”模块的接口/数据结构声明；定义 DynamicParameterConfig |
| [`src/flygripper_common/include/flygripper_common/ros/joint_state_publisher.hpp`](../src/flygripper_common/include/flygripper_common/ros/joint_state_publisher.hpp) | 关节状态发布的接口/数据结构声明；定义 JointState、JointStatePublisher |
| [`src/flygripper_common/include/flygripper_common/ros/parameters.hpp`](../src/flygripper_common/include/flygripper_common/ros/parameters.hpp) | “parameters”模块的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/trajectory/polynomial_trajectory.hpp`](../src/flygripper_common/include/flygripper_common/trajectory/polynomial_trajectory.hpp) | 多项式轨迹的接口/数据结构声明；定义 Piece、PolynomialTrajectory |
| [`src/flygripper_common/include/flygripper_common/trajectory/root_finder.hpp`](../src/flygripper_common/include/flygripper_common/trajectory/root_finder.hpp) | 求根工具的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/type/actuator_command.hpp`](../src/flygripper_common/include/flygripper_common/type/actuator_command.hpp) | 执行器命令的接口/数据结构声明；定义 ActuatorCommandMode、ActuatorCommand |
| [`src/flygripper_common/include/flygripper_common/type/actuator_state.hpp`](../src/flygripper_common/include/flygripper_common/type/actuator_state.hpp) | 执行器状态的接口/数据结构声明；定义 ActuatorState |
| [`src/flygripper_common/include/flygripper_common/type/base.hpp`](../src/flygripper_common/include/flygripper_common/type/base.hpp) | “base”模块的接口/数据结构声明；定义 Derived |
| [`src/flygripper_common/include/flygripper_common/type/battery.hpp`](../src/flygripper_common/include/flygripper_common/type/battery.hpp) | 电池状态的接口/数据结构声明；定义 Battery |
| [`src/flygripper_common/include/flygripper_common/type/const.hpp`](../src/flygripper_common/include/flygripper_common/type/const.hpp) | “const”模块的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/type/fc_state.hpp`](../src/flygripper_common/include/flygripper_common/type/fc_state.hpp) | 飞控状态的接口/数据结构声明；定义 FcState |
| [`src/flygripper_common/include/flygripper_common/type/imu.hpp`](../src/flygripper_common/include/flygripper_common/type/imu.hpp) | IMU的接口/数据结构声明；定义 Imu |
| [`src/flygripper_common/include/flygripper_common/type/odometry.hpp`](../src/flygripper_common/include/flygripper_common/type/odometry.hpp) | 里程计的接口/数据结构声明；定义 Odometry |
| [`src/flygripper_common/include/flygripper_common/type/rc.hpp`](../src/flygripper_common/include/flygripper_common/type/rc.hpp) | 遥控器输入的接口/数据结构声明；定义 Rc |
| [`src/flygripper_common/include/flygripper_common/type/reference.hpp`](../src/flygripper_common/include/flygripper_common/type/reference.hpp) | 参考指令的接口/数据结构声明；定义 Reference |
| [`src/flygripper_common/include/flygripper_common/type/robot_command.hpp`](../src/flygripper_common/include/flygripper_common/type/robot_command.hpp) | “robot_command”模块的接口/数据结构声明；定义 RobotCommandMode、RobotCommand |
| [`src/flygripper_common/include/flygripper_common/type/robot_parameter.hpp`](../src/flygripper_common/include/flygripper_common/type/robot_parameter.hpp) | 机器人参数的接口/数据结构声明；定义 RobotParameter、RobotParameterConfig |
| [`src/flygripper_common/include/flygripper_common/type/robot_state.hpp`](../src/flygripper_common/include/flygripper_common/type/robot_state.hpp) | 机器人状态的接口/数据结构声明；定义 RobotState |
| [`src/flygripper_common/include/flygripper_common/type/rotor_speed.hpp`](../src/flygripper_common/include/flygripper_common/type/rotor_speed.hpp) | 旋翼转速的接口/数据结构声明；定义 RotorSpeed |
| [`src/flygripper_common/include/flygripper_common/type/trajectory.hpp`](../src/flygripper_common/include/flygripper_common/type/trajectory.hpp) | “trajectory”模块的接口/数据结构声明；定义 Trajectory、XYZMode、HeadingMode |
| [`src/flygripper_common/include/flygripper_common/utils/log.hpp`](../src/flygripper_common/include/flygripper_common/utils/log.hpp) | “log”模块的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/utils/math.hpp`](../src/flygripper_common/include/flygripper_common/utils/math.hpp) | “math”模块的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/utils/realtime.hpp`](../src/flygripper_common/include/flygripper_common/utils/realtime.hpp) | 实时运行工具的接口/数据结构声明 |
| [`src/flygripper_common/include/flygripper_common/utils/rotation.hpp`](../src/flygripper_common/include/flygripper_common/utils/rotation.hpp) | “rotation”模块的接口/数据结构声明；定义 Quat2RPY |
| [`src/flygripper_common/include/flygripper_common/utils/runtime_profiler.hpp`](../src/flygripper_common/include/flygripper_common/utils/runtime_profiler.hpp) | 运行耗时统计的接口/数据结构声明；定义 RuntimeProfiler、ScopedRuntimeSample |
| [`src/flygripper_common/include/flygripper_common/utils/time.hpp`](../src/flygripper_common/include/flygripper_common/utils/time.hpp) | “time”模块的接口/数据结构声明；定义 Timer |
| [`src/flygripper_common/package.xml`](../src/flygripper_common/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_common/scripts/robot_model.py`](../src/flygripper_common/scripts/robot_model.py) | “robot_model”模块的 Python 实现或测试 |
| [`src/flygripper_common/src/flygripper_common.cpp`](../src/flygripper_common/src/flygripper_common.cpp) | “flygripper_common”模块的实现源码 |
| [`src/flygripper_common/test/parameter_test.cpp`](../src/flygripper_common/test/parameter_test.cpp) | “parameter_test”模块相关测试源码 |

## src/flygripper_controller（24 个）

几何飞控和抓取控制器。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_controller/CMakeLists.txt`](../src/flygripper_controller/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_controller/README.md`](../src/flygripper_controller/README.md) | 该目录的说明文档 |
| [`src/flygripper_controller/cfg/RosGeometricControllerParameter.cfg`](../src/flygripper_controller/cfg/RosGeometricControllerParameter.cfg) | “RosGeometricControllerParameter”模块动态参数定义或配置 |
| [`src/flygripper_controller/cfg/benchmark.xml`](../src/flygripper_controller/cfg/benchmark.xml) | PlotJuggler 控制基准曲线布局 |
| [`src/flygripper_controller/cfg/flygripper.xml`](../src/flygripper_controller/cfg/flygripper.xml) | PlotJuggler 飞爪位置等曲线布局 |
| [`src/flygripper_controller/cfg/hummingbird.xml`](../src/flygripper_controller/cfg/hummingbird.xml) | PlotJuggler hummingbird 机型曲线布局 |
| [`src/flygripper_controller/cfg/mapping.xml`](../src/flygripper_controller/cfg/mapping.xml) | PlotJuggler 映射实验曲线布局 |
| [`src/flygripper_controller/cfg/mapping2.xml`](../src/flygripper_controller/cfg/mapping2.xml) | PlotJuggler 映射实验第二版曲线布局 |
| [`src/flygripper_controller/cfg/sim.xml`](../src/flygripper_controller/cfg/sim.xml) | PlotJuggler 仿真曲线布局 |
| [`src/flygripper_controller/cfg/simple.xml`](../src/flygripper_controller/cfg/simple.xml) | PlotJuggler 简化机型曲线布局 |
| [`src/flygripper_controller/include/flygripper_controller/controller_base.hpp`](../src/flygripper_controller/include/flygripper_controller/controller_base.hpp) | 控制器基类的接口/数据结构声明；定义 ControllerBase |
| [`src/flygripper_controller/include/flygripper_controller/geometric_controller.hpp`](../src/flygripper_controller/include/flygripper_controller/geometric_controller.hpp) | 几何控制的接口/数据结构声明；定义 GeometricControllerData、GeometricController |
| [`src/flygripper_controller/include/flygripper_controller/geometric_controller_param.hpp`](../src/flygripper_controller/include/flygripper_controller/geometric_controller_param.hpp) | 几何控制的接口/数据结构声明；定义 CompensationMethod、GeometricControllerParameter、GeometricControllerParameterConfig |
| [`src/flygripper_controller/include/flygripper_controller/grasp_controller.hpp`](../src/flygripper_controller/include/flygripper_controller/grasp_controller.hpp) | 抓取控制的接口/数据结构声明 |
| [`src/flygripper_controller/package.xml`](../src/flygripper_controller/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_controller/params/flygripper.yaml`](../src/flygripper_controller/params/flygripper.yaml) | “flygripper”模块配置参数；顶层键：geometric_controller |
| [`src/flygripper_controller/params/flygripper_grasp.yaml`](../src/flygripper_controller/params/flygripper_grasp.yaml) | “flygripper_grasp”模块配置参数；顶层键：geometric_controller |
| [`src/flygripper_controller/params/flygripper_sim.yaml`](../src/flygripper_controller/params/flygripper_sim.yaml) | “flygripper_sim”模块配置参数；顶层键：geometric_controller |
| [`src/flygripper_controller/params/flygripper_simple_sim.yaml`](../src/flygripper_controller/params/flygripper_simple_sim.yaml) | “flygripper_simple_sim”模块配置参数；顶层键：geometric_controller |
| [`src/flygripper_controller/params/hummingbird_sim.yaml`](../src/flygripper_controller/params/hummingbird_sim.yaml) | “hummingbird_sim”模块配置参数；顶层键：geometric_controller |
| [`src/flygripper_controller/src/controller_benchmark.cpp`](../src/flygripper_controller/src/controller_benchmark.cpp) | “controller_benchmark”模块的实现源码 |
| [`src/flygripper_controller/src/geometric_controller.cpp`](../src/flygripper_controller/src/geometric_controller.cpp) | 几何控制的实现源码 |
| [`src/flygripper_controller/src/grasp_controller.cpp`](../src/flygripper_controller/src/grasp_controller.cpp) | 抓取控制的实现源码 |
| [`src/flygripper_controller/src/test.cpp`](../src/flygripper_controller/src/test.cpp) | “test”模块相关测试源码 |

## src/flygripper_description（80 个）

机体、夹爪及场景模型。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_description/CMakeLists.txt`](../src/flygripper_description/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_description/launch/load_object_urdf.launch`](../src/flygripper_description/launch/load_object_urdf.launch) | ROS 启动文件：“load_object_urdf”模块；节点包 gazebo_ros、robot_state_publisher |
| [`src/flygripper_description/launch/load_robot_urdf.launch`](../src/flygripper_description/launch/load_robot_urdf.launch) | ROS 启动文件：“load_robot_urdf”模块；节点包 gazebo_ros、tf、robot_state_publisher |
| [`src/flygripper_description/launch/test_urdf.launch`](../src/flygripper_description/launch/test_urdf.launch) | ROS 启动文件：“test_urdf”模块；节点包 joint_state_publisher_gui、robot_state_publisher、rviz |
| [`src/flygripper_description/models/README.md`](../src/flygripper_description/models/README.md) | 该目录的说明文档 |
| [`src/flygripper_description/models/bookshelf/model-1_2.sdf`](../src/flygripper_description/models/bookshelf/model-1_2.sdf) | “bookshelf”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/bookshelf/model-1_3.sdf`](../src/flygripper_description/models/bookshelf/model-1_3.sdf) | “bookshelf”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/bookshelf/model-1_4.sdf`](../src/flygripper_description/models/bookshelf/model-1_4.sdf) | “bookshelf”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/bookshelf/model.config`](../src/flygripper_description/models/bookshelf/model.config) | “bookshelf”场景模型的 CONFIG 描述或元数据 |
| [`src/flygripper_description/models/bookshelf/model.sdf`](../src/flygripper_description/models/bookshelf/model.sdf) | “bookshelf”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/bookshelf/model.urdf`](../src/flygripper_description/models/bookshelf/model.urdf) | “bookshelf”场景模型的 URDF 描述或元数据 |
| [`src/flygripper_description/models/coke_can/materials/textures/coke_can.png`](../src/flygripper_description/models/coke_can/materials/textures/coke_can.png) | “coke_can”可视化/碰撞资源（PNG） |
| [`src/flygripper_description/models/coke_can/meshes/coke_can.dae`](../src/flygripper_description/models/coke_can/meshes/coke_can.dae) | “coke_can”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/coke_can/model-1_2.sdf`](../src/flygripper_description/models/coke_can/model-1_2.sdf) | “coke_can”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/coke_can/model-1_3.sdf`](../src/flygripper_description/models/coke_can/model-1_3.sdf) | “coke_can”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/coke_can/model-1_4.sdf`](../src/flygripper_description/models/coke_can/model-1_4.sdf) | “coke_can”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/coke_can/model.config`](../src/flygripper_description/models/coke_can/model.config) | “coke_can”场景模型的 CONFIG 描述或元数据 |
| [`src/flygripper_description/models/coke_can/model.sdf`](../src/flygripper_description/models/coke_can/model.sdf) | “coke_can”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/coke_can/model.urdf`](../src/flygripper_description/models/coke_can/model.urdf) | “coke_can”场景模型的 URDF 描述或元数据 |
| [`src/flygripper_description/models/flygripper/carbon_fiber.jpg`](../src/flygripper_description/models/flygripper/carbon_fiber.jpg) | “flygripper”可视化/碰撞资源（JPG） |
| [`src/flygripper_description/models/flygripper/down_slide_rail_link.STL`](../src/flygripper_description/models/flygripper/down_slide_rail_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/left_down_link.STL`](../src/flygripper_description/models/flygripper/left_down_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/left_down_link.dae`](../src/flygripper_description/models/flygripper/left_down_link.dae) | “flygripper”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/flygripper/left_down_rotor_link.STL`](../src/flygripper_description/models/flygripper/left_down_rotor_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/left_slide_rail_link.STL`](../src/flygripper_description/models/flygripper/left_slide_rail_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/left_up_link.STL`](../src/flygripper_description/models/flygripper/left_up_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/left_up_link.dae`](../src/flygripper_description/models/flygripper/left_up_link.dae) | “flygripper”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/flygripper/left_up_rotor_link.STL`](../src/flygripper_description/models/flygripper/left_up_rotor_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/right_down_link.STL`](../src/flygripper_description/models/flygripper/right_down_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/right_down_link.dae`](../src/flygripper_description/models/flygripper/right_down_link.dae) | “flygripper”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/flygripper/right_down_rotor_link.STL`](../src/flygripper_description/models/flygripper/right_down_rotor_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/right_slide_rail_link.STL`](../src/flygripper_description/models/flygripper/right_slide_rail_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/right_slide_rail_link.dae`](../src/flygripper_description/models/flygripper/right_slide_rail_link.dae) | “flygripper”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/flygripper/right_up_link.STL`](../src/flygripper_description/models/flygripper/right_up_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/right_up_link.dae`](../src/flygripper_description/models/flygripper/right_up_link.dae) | “flygripper”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/flygripper/right_up_rotor_link.STL`](../src/flygripper_description/models/flygripper/right_up_rotor_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/servo_horn_link.STL`](../src/flygripper_description/models/flygripper/servo_horn_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/flygripper/servo_link.STL`](../src/flygripper_description/models/flygripper/servo_link.STL) | “flygripper”可视化/碰撞资源（STL） |
| [`src/flygripper_description/models/hummingbird.dae`](../src/flygripper_description/models/hummingbird.dae) | “hummingbird.dae”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/lamp_post/materials/textures/Lamp_Post_Diffuse.png`](../src/flygripper_description/models/lamp_post/materials/textures/Lamp_Post_Diffuse.png) | “lamp_post”可视化/碰撞资源（PNG） |
| [`src/flygripper_description/models/lamp_post/materials/textures/Lamp_Post_Emissive.png`](../src/flygripper_description/models/lamp_post/materials/textures/Lamp_Post_Emissive.png) | “lamp_post”可视化/碰撞资源（PNG） |
| [`src/flygripper_description/models/lamp_post/materials/textures/Lamp_Post_Spec.png`](../src/flygripper_description/models/lamp_post/materials/textures/Lamp_Post_Spec.png) | “lamp_post”可视化/碰撞资源（PNG） |
| [`src/flygripper_description/models/lamp_post/meshes/lamp_post.dae`](../src/flygripper_description/models/lamp_post/meshes/lamp_post.dae) | “lamp_post”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/lamp_post/model-1_3.sdf`](../src/flygripper_description/models/lamp_post/model-1_3.sdf) | “lamp_post”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/lamp_post/model-1_4.sdf`](../src/flygripper_description/models/lamp_post/model-1_4.sdf) | “lamp_post”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/lamp_post/model.config`](../src/flygripper_description/models/lamp_post/model.config) | “lamp_post”场景模型的 CONFIG 描述或元数据 |
| [`src/flygripper_description/models/lamp_post/model.sdf`](../src/flygripper_description/models/lamp_post/model.sdf) | “lamp_post”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/mustard_bottle/meshes/obj_000005.dae`](../src/flygripper_description/models/mustard_bottle/meshes/obj_000005.dae) | “mustard_bottle”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/mustard_bottle/meshes/obj_000005_collision.dae`](../src/flygripper_description/models/mustard_bottle/meshes/obj_000005_collision.dae) | “mustard_bottle”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/mustard_bottle/model.config`](../src/flygripper_description/models/mustard_bottle/model.config) | “mustard_bottle”场景模型的 CONFIG 描述或元数据 |
| [`src/flygripper_description/models/mustard_bottle/model.rsdf`](../src/flygripper_description/models/mustard_bottle/model.rsdf) | “mustard_bottle”场景模型的 RSDF 描述或元数据 |
| [`src/flygripper_description/models/mustard_bottle/model.sdf`](../src/flygripper_description/models/mustard_bottle/model.sdf) | “mustard_bottle”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/mustard_bottle/model.urdf`](../src/flygripper_description/models/mustard_bottle/model.urdf) | “mustard_bottle”场景模型的 URDF 描述或元数据 |
| [`src/flygripper_description/models/mustard_bottle/textures/obj_000005.png`](../src/flygripper_description/models/mustard_bottle/textures/obj_000005.png) | “mustard_bottle”可视化/碰撞资源（PNG） |
| [`src/flygripper_description/models/propeller_ccw.dae`](../src/flygripper_description/models/propeller_ccw.dae) | “propeller_ccw.dae”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/propeller_cw.dae`](../src/flygripper_description/models/propeller_cw.dae) | “propeller_cw.dae”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/soccer_ball/materials/textures/rcs-soccerball.png`](../src/flygripper_description/models/soccer_ball/materials/textures/rcs-soccerball.png) | “soccer_ball”可视化/碰撞资源（PNG） |
| [`src/flygripper_description/models/soccer_ball/meshes/ball.dae`](../src/flygripper_description/models/soccer_ball/meshes/ball.dae) | “soccer_ball”可视化/碰撞资源（DAE） |
| [`src/flygripper_description/models/soccer_ball/model.config`](../src/flygripper_description/models/soccer_ball/model.config) | “soccer_ball”场景模型的 CONFIG 描述或元数据 |
| [`src/flygripper_description/models/soccer_ball/model.sdf`](../src/flygripper_description/models/soccer_ball/model.sdf) | “soccer_ball”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/soccer_ball/model.urdf`](../src/flygripper_description/models/soccer_ball/model.urdf) | “soccer_ball”场景模型的 URDF 描述或元数据 |
| [`src/flygripper_description/models/table/model.config`](../src/flygripper_description/models/table/model.config) | “table”场景模型的 CONFIG 描述或元数据 |
| [`src/flygripper_description/models/table/model.rsdf`](../src/flygripper_description/models/table/model.rsdf) | “table”场景模型的 RSDF 描述或元数据 |
| [`src/flygripper_description/models/table/model.sdf`](../src/flygripper_description/models/table/model.sdf) | “table”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/unit_box/model.config`](../src/flygripper_description/models/unit_box/model.config) | “unit_box”场景模型的 CONFIG 描述或元数据 |
| [`src/flygripper_description/models/unit_box/model.sdf`](../src/flygripper_description/models/unit_box/model.sdf) | “unit_box”场景模型的 SDF 描述或元数据 |
| [`src/flygripper_description/models/unit_box/model.urdf`](../src/flygripper_description/models/unit_box/model.urdf) | “unit_box”场景模型的 URDF 描述或元数据 |
| [`src/flygripper_description/package.xml`](../src/flygripper_description/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_description/rviz/urdf.rviz`](../src/flygripper_description/rviz/urdf.rviz) | “urdf”模块的 RViz 显示布局 |
| [`src/flygripper_description/urdf/common.xacro`](../src/flygripper_description/urdf/common.xacro) | “common”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/flygripper.xacro`](../src/flygripper_description/urdf/flygripper.xacro) | “flygripper”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/flygripper_base.xacro`](../src/flygripper_description/urdf/flygripper_base.xacro) | “flygripper_base”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/flygripper_simple.xacro`](../src/flygripper_description/urdf/flygripper_simple.xacro) | “flygripper_simple”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/flygripper_simple_base.xacro`](../src/flygripper_description/urdf/flygripper_simple_base.xacro) | “flygripper_simple_base”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/hummingbird.xacro`](../src/flygripper_description/urdf/hummingbird.xacro) | “hummingbird”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/hummingbird_base.xacro`](../src/flygripper_description/urdf/hummingbird_base.xacro) | “hummingbird_base”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/materials.xacro`](../src/flygripper_description/urdf/materials.xacro) | “materials”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/object.xacro`](../src/flygripper_description/urdf/object.xacro) | “object”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/template_robot.xacro`](../src/flygripper_description/urdf/template_robot.xacro) | “template_robot”模块的机器人模型描述/宏 |
| [`src/flygripper_description/urdf/template_robot_base.xacro`](../src/flygripper_description/urdf/template_robot_base.xacro) | “template_robot_base”模块的机器人模型描述/宏 |

## src/flygripper_msgs（15 个）

夹爪与飞行 ROS 消息定义。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_msgs/CMakeLists.txt`](../src/flygripper_msgs/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_msgs/msg/ActuatorCommand.msg`](../src/flygripper_msgs/msg/ActuatorCommand.msg) | ROS 消息：ActuatorCommand；字段包括 header、mode、reference_position、reference_velocity、reference_effort |
| [`src/flygripper_msgs/msg/ActuatorState.msg`](../src/flygripper_msgs/msg/ActuatorState.msg) | ROS 消息：ActuatorState；字段包括 header、position、velocity、effort、voltage、current 等 |
| [`src/flygripper_msgs/msg/GeometricControllerData.msg`](../src/flygripper_msgs/msg/GeometricControllerData.msg) | ROS 消息：GeometricControllerData；字段包括 position_reference、position_current、velocity_reference、velocity_current、velocity_current_filtered、acceleration_reference 等 |
| [`src/flygripper_msgs/msg/GraspCommand.msg`](../src/flygripper_msgs/msg/GraspCommand.msg) | ROS 消息：GraspCommand；字段包括 header、grasp_mode、grasp_position |
| [`src/flygripper_msgs/msg/GraspState.msg`](../src/flygripper_msgs/msg/GraspState.msg) | ROS 消息：GraspState；字段包括 force_grasp、force_shear、error、reference_position |
| [`src/flygripper_msgs/msg/PidControllerState.msg`](../src/flygripper_msgs/msg/PidControllerState.msg) | ROS 消息：PidControllerState；字段包括 header、reference_value、reference_value_dot、current_value、current_value_dot、error 等 |
| [`src/flygripper_msgs/msg/Polynomial.msg`](../src/flygripper_msgs/msg/Polynomial.msg) | ROS 消息：Polynomial；字段包括 order、dim、coeff、duration |
| [`src/flygripper_msgs/msg/PolynomialTrajectory.msg`](../src/flygripper_msgs/msg/PolynomialTrajectory.msg) | ROS 消息：PolynomialTrajectory；字段包括 header、id、xyz_pieces、heading_mode、heading_pieces |
| [`src/flygripper_msgs/msg/RobotState.msg`](../src/flygripper_msgs/msg/RobotState.msg) | ROS 消息：RobotState；字段包括 header、position、attitude、linear_velocity、angular_velocity、linear_acceleration 等 |
| [`src/flygripper_msgs/msg/RotorCommand.msg`](../src/flygripper_msgs/msg/RotorCommand.msg) | ROS 消息：RotorCommand；字段包括 header、mode、collective_thrust、attitude、bodyrates、rotor_velocities |
| [`src/flygripper_msgs/msg/RotorVelocityFeedback.msg`](../src/flygripper_msgs/msg/RotorVelocityFeedback.msg) | ROS 消息：RotorVelocityFeedback；字段包括 header、rotor_velocities |
| [`src/flygripper_msgs/msg/TactSensor.msg`](../src/flygripper_msgs/msg/TactSensor.msg) | ROS 消息：TactSensor；字段包括 header、id、pos、force、force_raw、force_body 等 |
| [`src/flygripper_msgs/msg/TactState.msg`](../src/flygripper_msgs/msg/TactState.msg) | ROS 消息：TactState；字段包括 sensors、has_calibrated、force_shear、force_grasp、size |
| [`src/flygripper_msgs/package.xml`](../src/flygripper_msgs/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |

## src/flygripper_planner（48 个）

夹爪轨迹与抓取任务规划。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_planner/CMakeLists.txt`](../src/flygripper_planner/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_planner/cfg/RosTrajOptParameter.cfg`](../src/flygripper_planner/cfg/RosTrajOptParameter.cfg) | “RosTrajOptParameter”模块动态参数定义或配置 |
| [`src/flygripper_planner/config/grasp.rviz`](../src/flygripper_planner/config/grasp.rviz) | “grasp”模块的 RViz 显示布局 |
| [`src/flygripper_planner/config/test.rviz`](../src/flygripper_planner/config/test.rviz) | “test”模块的 RViz 显示布局 |
| [`src/flygripper_planner/config/traj_opt.rviz`](../src/flygripper_planner/config/traj_opt.rviz) | 轨迹优化的 RViz 显示布局 |
| [`src/flygripper_planner/include/flygripper_planner/lbfgs.hpp`](../src/flygripper_planner/include/flygripper_planner/lbfgs.hpp) | L-BFGS 优化的接口/数据结构声明；定义 lbfgs_parameter_t、callback_data_t、iteration_data_t |
| [`src/flygripper_planner/include/flygripper_planner/minco.hpp`](../src/flygripper_planner/include/flygripper_planner/minco.hpp) | MINCO 多项式轨迹的接口/数据结构声明；定义 with、is、BandedSystem |
| [`src/flygripper_planner/include/flygripper_planner/rc_utils.hpp`](../src/flygripper_planner/include/flygripper_planner/rc_utils.hpp) | 遥控器输入的接口/数据结构声明；定义 ToggleSwitch、RcInput |
| [`src/flygripper_planner/include/flygripper_planner/traj_opt.h`](../src/flygripper_planner/include/flygripper_planner/traj_opt.h) | 轨迹优化的接口/数据结构声明；定义 TrajOpt |
| [`src/flygripper_planner/include/flygripper_planner/traj_opt_param.hpp`](../src/flygripper_planner/include/flygripper_planner/traj_opt_param.hpp) | 轨迹优化的接口/数据结构声明；定义 TrajOptParameter、TrajOptParameterConfig |
| [`src/flygripper_planner/include/flygripper_planner/traj_opt_real.h`](../src/flygripper_planner/include/flygripper_planner/traj_opt_real.h) | 轨迹优化的接口/数据结构声明；定义 TrajOpt |
| [`src/flygripper_planner/include/flygripper_planner/traj_opt_utils.hpp`](../src/flygripper_planner/include/flygripper_planner/traj_opt_utils.hpp) | 轨迹优化的接口/数据结构声明 |
| [`src/flygripper_planner/include/flygripper_planner/velocity_control.hpp`](../src/flygripper_planner/include/flygripper_planner/velocity_control.hpp) | 速度控制的接口/数据结构声明；定义 VelocityControlSystem、State、Input |
| [`src/flygripper_planner/include/flygripper_planner/visualizer.hpp`](../src/flygripper_planner/include/flygripper_planner/visualizer.hpp) | 规划结果可视化的接口/数据结构声明；定义 Color、Visualizer、MsgType |
| [`src/flygripper_planner/launch/door.launch`](../src/flygripper_planner/launch/door.launch) | ROS 启动文件：“door”模块；节点包 flygripper_planner、rviz |
| [`src/flygripper_planner/launch/goal.launch`](../src/flygripper_planner/launch/goal.launch) | ROS 启动文件：“goal”模块；节点包 flygripper_planner |
| [`src/flygripper_planner/launch/grasp.launch`](../src/flygripper_planner/launch/grasp.launch) | ROS 启动文件：“grasp”模块；节点包 flygripper_planner、rviz |
| [`src/flygripper_planner/launch/real.launch`](../src/flygripper_planner/launch/real.launch) | ROS 启动文件：“real”模块；节点包 flygripper_planner、rviz |
| [`src/flygripper_planner/launch/test.launch`](../src/flygripper_planner/launch/test.launch) | ROS 启动文件：“test”模块；节点包 flygripper_planner |
| [`src/flygripper_planner/package.xml`](../src/flygripper_planner/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_planner/params/bak/grasp.yaml`](../src/flygripper_planner/params/bak/grasp.yaml) | “grasp”模块配置参数；顶层键：piece_length、allocation_speed、allocation_acc、body_in_odom、actuator_max |
| [`src/flygripper_planner/params/bak/joy.yaml`](../src/flygripper_planner/params/bak/joy.yaml) | “joy”模块配置参数；顶层键：max_velocity、update_duration |
| [`src/flygripper_planner/params/bak/offline.yaml`](../src/flygripper_planner/params/bak/offline.yaml) | “offline”模块配置参数；顶层键：waypoint_height、initial_vel、initial_acc、initial_jerk、terminal_vel |
| [`src/flygripper_planner/params/bak/online.yaml`](../src/flygripper_planner/params/bak/online.yaml) | “online”模块配置参数；顶层键：waypoint_height、terminal_vel、terminal_acc、terminal_jerk、allocation_speed |
| [`src/flygripper_planner/params/command.yaml`](../src/flygripper_planner/params/command.yaml) | “command”模块配置参数；顶层键：piece_length、morph_shrink_pos、morph_expand_pos、waypoints_0、waypoints_1 |
| [`src/flygripper_planner/params/command_grasp_bottle.yaml`](../src/flygripper_planner/params/command_grasp_bottle.yaml) | “command_grasp_bottle”模块配置参数；顶层键：piece_length、morph_shrink_pos、morph_expand_pos、waypoints_0、waypoints_1 |
| [`src/flygripper_planner/params/command_grasp_stick.yaml`](../src/flygripper_planner/params/command_grasp_stick.yaml) | “command_grasp_stick”模块配置参数；顶层键：piece_length、morph_shrink_pos、morph_expand_pos、waypoints_0、waypoints_1 |
| [`src/flygripper_planner/params/command_grasp_tissue.yaml`](../src/flygripper_planner/params/command_grasp_tissue.yaml) | “command_grasp_tissue”模块配置参数；顶层键：piece_length、morph_shrink_pos、morph_expand_pos、waypoints_0、waypoints_1 |
| [`src/flygripper_planner/params/command_raw.yaml`](../src/flygripper_planner/params/command_raw.yaml) | “command_raw”模块配置参数；顶层键：piece_length、morph_shrink_pos、morph_expand_pos、waypoints_0、waypoints_1 |
| [`src/flygripper_planner/params/command_sim.yaml`](../src/flygripper_planner/params/command_sim.yaml) | “command_sim”模块配置参数；顶层键：piece_length、morph_shrink_pos、morph_expand_pos、waypoints_0、waypoints_1 |
| [`src/flygripper_planner/params/door.yaml`](../src/flygripper_planner/params/door.yaml) | “door”模块配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、M |
| [`src/flygripper_planner/params/goal.yaml`](../src/flygripper_planner/params/goal.yaml) | “goal”模块配置参数；顶层键：segment_per_distance、max_iteration、min_piece_num、K、M |
| [`src/flygripper_planner/params/real.yaml`](../src/flygripper_planner/params/real.yaml) | “real”模块配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、rhoT |
| [`src/flygripper_planner/params/real_door.yaml`](../src/flygripper_planner/params/real_door.yaml) | “real_door”模块配置参数；顶层键：K、rhoT、rhoV、rhoA、rhoDYaw |
| [`src/flygripper_planner/params/real_grasp_bottle.yaml`](../src/flygripper_planner/params/real_grasp_bottle.yaml) | “real_grasp_bottle”模块配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、rhoT |
| [`src/flygripper_planner/params/real_grasp_tissue.yaml`](../src/flygripper_planner/params/real_grasp_tissue.yaml) | “real_grasp_tissue”模块配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、rhoT |
| [`src/flygripper_planner/params/real_home.yaml`](../src/flygripper_planner/params/real_home.yaml) | “real_home”模块配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、rhoT |
| [`src/flygripper_planner/params/real_human_robot_contact.yaml`](../src/flygripper_planner/params/real_human_robot_contact.yaml) | “real_human_robot_contact”模块配置参数；顶层键：K、rhoT、rhoV、rhoA、rhoDYaw |
| [`src/flygripper_planner/params/real_perching.yaml`](../src/flygripper_planner/params/real_perching.yaml) | 遥控器输入配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、rhoT |
| [`src/flygripper_planner/params/real_sim.yaml`](../src/flygripper_planner/params/real_sim.yaml) | “real_sim”模块配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、rhoT |
| [`src/flygripper_planner/params/real_test.yaml`](../src/flygripper_planner/params/real_test.yaml) | “real_test”模块配置参数；顶层键：arc_angle、arc_radius、arc_point_num、K、rhoT |
| [`src/flygripper_planner/src/door.cpp`](../src/flygripper_planner/src/door.cpp) | “door”模块的实现源码 |
| [`src/flygripper_planner/src/goal.cpp`](../src/flygripper_planner/src/goal.cpp) | “goal”模块的实现源码 |
| [`src/flygripper_planner/src/grasp.cpp`](../src/flygripper_planner/src/grasp.cpp) | “grasp”模块的实现源码 |
| [`src/flygripper_planner/src/grasp_raw.cpp`](../src/flygripper_planner/src/grasp_raw.cpp) | “grasp_raw”模块的实现源码 |
| [`src/flygripper_planner/src/joy_test.cpp`](../src/flygripper_planner/src/joy_test.cpp) | “joy_test”模块的实现源码 |
| [`src/flygripper_planner/src/online_test.cpp`](../src/flygripper_planner/src/online_test.cpp) | “online_test”模块的实现源码 |
| [`src/flygripper_planner/src/real.cpp`](../src/flygripper_planner/src/real.cpp) | “real”模块的实现源码 |

## src/flygripper_teleop（9 个）

手柄/遥控操作映射。

| 文件 | 作用概况 |
|---|---|
| [`src/flygripper_teleop/CMakeLists.txt`](../src/flygripper_teleop/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/flygripper_teleop/README.md`](../src/flygripper_teleop/README.md) | 该目录的说明文档 |
| [`src/flygripper_teleop/cfg/Parameters.cfg`](../src/flygripper_teleop/cfg/Parameters.cfg) | “Parameters”模块动态参数定义或配置 |
| [`src/flygripper_teleop/include/flygripper_teleop/teleop_joy.hpp`](../src/flygripper_teleop/include/flygripper_teleop/teleop_joy.hpp) | “teleop_joy”模块的接口/数据结构声明；定义 TeleopJoy |
| [`src/flygripper_teleop/launch/teleop_joy.launch`](../src/flygripper_teleop/launch/teleop_joy.launch) | ROS 启动文件：“teleop_joy”模块；节点包 joy、flygripper_teleop |
| [`src/flygripper_teleop/package.xml`](../src/flygripper_teleop/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/flygripper_teleop/src/teleop_joy_bldc.cpp`](../src/flygripper_teleop/src/teleop_joy_bldc.cpp) | “teleop_joy_bldc”模块的实现源码 |
| [`src/flygripper_teleop/src/teleop_joy_node.cpp`](../src/flygripper_teleop/src/teleop_joy_node.cpp) | “teleop_joy_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/flygripper_teleop/src/teleop_joy_test.cpp`](../src/flygripper_teleop/src/teleop_joy_test.cpp) | “teleop_joy_test”模块的实现源码 |

## src/planning/DecompROS（46 个）

目标预测与轨迹规划相关包及 DecompROS 依赖。

| 文件 | 作用概况 |
|---|---|
| [`src/planning/DecompROS/LICENSE`](../src/planning/DecompROS/LICENSE) | 该子项目的许可证文本 |
| [`src/planning/DecompROS/catkin_simple/CMakeLists.txt`](../src/planning/DecompROS/catkin_simple/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/planning/DecompROS/catkin_simple/cmake/catkin_simple-extras.cmake.em`](../src/planning/DecompROS/catkin_simple/cmake/catkin_simple-extras.cmake.em) | CMake 构建/模板辅助文件 |
| [`src/planning/DecompROS/catkin_simple/package.xml`](../src/planning/DecompROS/catkin_simple/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/include/bar/hello.h`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/include/bar/hello.h) | “hello”模块的接口/数据结构声明 |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/msg/HeaderString.msg`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/msg/HeaderString.msg) | ROS 消息：HeaderString；字段包括 header、data |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/package.xml`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/src/hello.cpp`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/bar/src/hello.cpp) | “hello”模块相关测试源码 |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/baz/include/baz/world.h`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/baz/include/baz/world.h) | “world”模块的接口/数据结构声明 |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/baz/package.xml`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/baz/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/foo/package.xml`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/foo/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/foo/src/main.cpp`](../src/planning/DecompROS/catkin_simple/test/scenarios/hello_world/foo/src/main.cpp) | “main”模块相关测试源码 |
| [`src/planning/DecompROS/decomp_ros_msgs/CMakeLists.txt`](../src/planning/DecompROS/decomp_ros_msgs/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/planning/DecompROS/decomp_ros_msgs/msg/Ellipsoid.msg`](../src/planning/DecompROS/decomp_ros_msgs/msg/Ellipsoid.msg) | ROS 消息：Ellipsoid；字段包括 d、E |
| [`src/planning/DecompROS/decomp_ros_msgs/msg/EllipsoidArray.msg`](../src/planning/DecompROS/decomp_ros_msgs/msg/EllipsoidArray.msg) | ROS 消息：EllipsoidArray；字段包括 header、ellipsoids |
| [`src/planning/DecompROS/decomp_ros_msgs/msg/Polyhedron.msg`](../src/planning/DecompROS/decomp_ros_msgs/msg/Polyhedron.msg) | ROS 消息：Polyhedron；字段包括 points、vector |
| [`src/planning/DecompROS/decomp_ros_msgs/msg/PolyhedronArray.msg`](../src/planning/DecompROS/decomp_ros_msgs/msg/PolyhedronArray.msg) | ROS 消息：PolyhedronArray；字段包括 header、polyhedrons |
| [`src/planning/DecompROS/decomp_ros_msgs/package.xml`](../src/planning/DecompROS/decomp_ros_msgs/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/DecompROS/decomp_ros_utils/CMakeLists.txt`](../src/planning/DecompROS/decomp_ros_utils/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/planning/DecompROS/decomp_ros_utils/icons/classes/EllipsoidArray.png`](../src/planning/DecompROS/decomp_ros_utils/icons/classes/EllipsoidArray.png) | ““EllipsoidArray”模块”可视化/碰撞资源（PNG） |
| [`src/planning/DecompROS/decomp_ros_utils/icons/classes/PolyhedronArray.png`](../src/planning/DecompROS/decomp_ros_utils/icons/classes/PolyhedronArray.png) | ““PolyhedronArray”模块”可视化/碰撞资源（PNG） |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_basis/data_type.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_basis/data_type.h) | “data_type”模块的接口/数据结构声明 |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_basis/data_utils.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_basis/data_utils.h) | “data_utils”模块的接口/数据结构声明；定义 T、TF |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_geometry/ellipsoid.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_geometry/ellipsoid.h) | “ellipsoid”模块的接口/数据结构声明；定义 Ellipsoid |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_geometry/geometric_utils.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_geometry/geometric_utils.h) | “geometric_utils”模块的接口/数据结构声明 |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_geometry/polyhedron.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_geometry/polyhedron.h) | “polyhedron”模块的接口/数据结构声明；定义 template、Hyperplane |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_ros_utils/data_ros_utils.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_ros_utils/data_ros_utils.h) | “data_ros_utils”模块的接口/数据结构声明 |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_util/decomp_base.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_util/decomp_base.h) | “decomp_base”模块的接口/数据结构声明；定义 DecompBase |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_util/ellipsoid_decomp.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_util/ellipsoid_decomp.h) | “ellipsoid_decomp”模块的接口/数据结构声明；定义 EllipsoidDecomp |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_util/iterative_decomp.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_util/iterative_decomp.h) | “iterative_decomp”模块的接口/数据结构声明；定义 IterativeDecomp |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_util/line_segment.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_util/line_segment.h) | “line_segment”模块的接口/数据结构声明；定义 LineSegment |
| [`src/planning/DecompROS/decomp_ros_utils/include/decomp_util/seed_decomp.h`](../src/planning/DecompROS/decomp_ros_utils/include/decomp_util/seed_decomp.h) | “seed_decomp”模块的接口/数据结构声明；定义 SeedDecomp |
| [`src/planning/DecompROS/decomp_ros_utils/package.xml`](../src/planning/DecompROS/decomp_ros_utils/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/DecompROS/decomp_ros_utils/plugin_description.xml`](../src/planning/DecompROS/decomp_ros_utils/plugin_description.xml) | “plugin_description”模块的 XML 配置或插件描述 |
| [`src/planning/DecompROS/decomp_ros_utils/src/bound_visual.cpp`](../src/planning/DecompROS/decomp_ros_utils/src/bound_visual.cpp) | “bound_visual”模块的实现源码 |
| [`src/planning/DecompROS/decomp_ros_utils/src/bound_visual.h`](../src/planning/DecompROS/decomp_ros_utils/src/bound_visual.h) | “bound_visual”模块的接口/数据结构声明；定义 BoundVisual |
| [`src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_display.cpp`](../src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_display.cpp) | “ellipsoid_array_display”模块的实现源码 |
| [`src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_display.h`](../src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_display.h) | “ellipsoid_array_display”模块的接口/数据结构声明；定义 EllipsoidArrayVisual、EllipsoidArrayDisplay |
| [`src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_visual.cpp`](../src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_visual.cpp) | “ellipsoid_array_visual”模块的实现源码 |
| [`src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_visual.h`](../src/planning/DecompROS/decomp_ros_utils/src/ellipsoid_array_visual.h) | “ellipsoid_array_visual”模块的接口/数据结构声明；定义 EllipsoidArrayVisual |
| [`src/planning/DecompROS/decomp_ros_utils/src/mesh_visual.cpp`](../src/planning/DecompROS/decomp_ros_utils/src/mesh_visual.cpp) | “mesh_visual”模块的实现源码 |
| [`src/planning/DecompROS/decomp_ros_utils/src/mesh_visual.h`](../src/planning/DecompROS/decomp_ros_utils/src/mesh_visual.h) | “mesh_visual”模块的接口/数据结构声明；定义 MeshVisual |
| [`src/planning/DecompROS/decomp_ros_utils/src/polyhedron_array_display.cpp`](../src/planning/DecompROS/decomp_ros_utils/src/polyhedron_array_display.cpp) | “polyhedron_array_display”模块的实现源码 |
| [`src/planning/DecompROS/decomp_ros_utils/src/polyhedron_array_display.h`](../src/planning/DecompROS/decomp_ros_utils/src/polyhedron_array_display.h) | “polyhedron_array_display”模块的接口/数据结构声明；定义 PolyhedronArrayDisplay |
| [`src/planning/DecompROS/decomp_ros_utils/src/vector_visual.cpp`](../src/planning/DecompROS/decomp_ros_utils/src/vector_visual.cpp) | “vector_visual”模块的实现源码 |
| [`src/planning/DecompROS/decomp_ros_utils/src/vector_visual.h`](../src/planning/DecompROS/decomp_ros_utils/src/vector_visual.h) | “vector_visual”模块的接口/数据结构声明；定义 VectorVisual |

## src/planning/planning（25 个）

目标预测与轨迹规划相关包及 DecompROS 依赖。

| 文件 | 作用概况 |
|---|---|
| [`src/planning/planning/CMakeLists.txt`](../src/planning/planning/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/planning/planning/include/env/env.hpp`](../src/planning/planning/include/env/env.hpp) | 规划环境的接口/数据结构声明；定义 hash、Node、NodeComparator |
| [`src/planning/planning/include/env/env_0510.hpp`](../src/planning/planning/include/env/env_0510.hpp) | 规划环境的接口/数据结构声明；定义 hash、Node、NodeComparator |
| [`src/planning/planning/include/env/free_space_grid.hpp`](../src/planning/planning/include/env/free_space_grid.hpp) | 自由空间网格的接口/数据结构声明；定义 FreeSpaceGrid |
| [`src/planning/planning/include/prediction/lbfgs.hpp`](../src/planning/planning/include/prediction/lbfgs.hpp) | L-BFGS 优化的接口/数据结构声明；定义 lbfgs_parameter_t、callback_data_t、iteration_data_t |
| [`src/planning/planning/include/prediction/prediction.hpp`](../src/planning/planning/include/prediction/prediction.hpp) | 目标状态预测的接口/数据结构声明；定义 TargetState、Node、NodeComparator |
| [`src/planning/planning/include/visualization/visualization.hpp`](../src/planning/planning/include/visualization/visualization.hpp) | “visualization”模块的接口/数据结构声明；定义 Visualization、ROTATION、CENTER |
| [`src/planning/planning/include/wr_msg/wr_msg.hpp`](../src/planning/planning/include/wr_msg/wr_msg.hpp) | “wr_msg”模块的接口/数据结构声明；定义 MSG、FILENAME |
| [`src/planning/planning/launch/debug.launch`](../src/planning/planning/launch/debug.launch) | ROS 启动文件：“debug”模块；节点包 nodelet |
| [`src/planning/planning/launch/fake_car_target.launch`](../src/planning/planning/launch/fake_car_target.launch) | ROS 启动文件：“fake_car_target”模块；节点包 mockamap、nodelet |
| [`src/planning/planning/launch/fake_target.launch`](../src/planning/planning/launch/fake_target.launch) | ROS 启动文件：“fake_target”模块；节点包 mockamap、nodelet |
| [`src/planning/planning/launch/grasping.launch`](../src/planning/planning/launch/grasping.launch) | ROS 启动文件：“grasping”模块；包含其他 launch |
| [`src/planning/planning/launch/log.launch`](../src/planning/planning/launch/log.launch) | ROS 启动文件：“log”模块；节点包 planning |
| [`src/planning/planning/launch/planning.launch`](../src/planning/planning/launch/planning.launch) | ROS 启动文件：“planning”模块；节点包 nodelet、planning |
| [`src/planning/planning/launch/simulation1.launch`](../src/planning/planning/launch/simulation1.launch) | ROS 启动文件：IMU；节点包 mapping、odom_visualization、target_ekf；包含其他 launch |
| [`src/planning/planning/launch/simulation2.launch`](../src/planning/planning/launch/simulation2.launch) | ROS 启动文件：IMU；节点包 mapping、odom_visualization、target_ekf；包含其他 launch |
| [`src/planning/planning/launch/simulation_landing.launch`](../src/planning/planning/launch/simulation_landing.launch) | ROS 启动文件：IMU；节点包 mapping、odom_visualization、target_ekf；包含其他 launch |
| [`src/planning/planning/nodelet_plugin.xml`](../src/planning/planning/nodelet_plugin.xml) | “nodelet_plugin”模块的 XML 配置或插件描述 |
| [`src/planning/planning/package.xml`](../src/planning/planning/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/planning/src/planning_nodelet.cpp`](../src/planning/planning/src/planning_nodelet.cpp) | “planning_nodelet”模块的 ROS 节点/Nodelet 实现 |
| [`src/planning/planning/src/planning_nodelet_0510.cpp`](../src/planning/planning/src/planning_nodelet_0510.cpp) | “planning_nodelet_0510”模块的 ROS 节点/Nodelet 实现 |
| [`src/planning/planning/src/play_bag_node.cpp`](../src/planning/planning/src/play_bag_node.cpp) | “play_bag_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/planning/planning/src/test_node.cpp`](../src/planning/planning/src/test_node.cpp) | “test_node”模块相关测试源码 |
| [`src/planning/planning/src/traj_server.cpp`](../src/planning/planning/src/traj_server.cpp) | “traj_server”模块的实现源码 |
| [`src/planning/planning/src/trigger_rc.cpp`](../src/planning/planning/src/trigger_rc.cpp) | 遥控器输入的实现源码 |

## src/planning/poly_planner（23 个）

目标预测与轨迹规划相关包及 DecompROS 依赖。

| 文件 | 作用概况 |
|---|---|
| [`src/planning/poly_planner/CMakeLists.txt`](../src/planning/poly_planner/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/planning/poly_planner/config/flygripper.rviz`](../src/planning/poly_planner/config/flygripper.rviz) | “flygripper”模块的 RViz 显示布局 |
| [`src/planning/poly_planner/config/flygripper_simple.rviz`](../src/planning/poly_planner/config/flygripper_simple.rviz) | “flygripper_simple”模块的 RViz 显示布局 |
| [`src/planning/poly_planner/config/params.yaml`](../src/planning/poly_planner/config/params.yaml) | “params”模块配置参数；顶层键：OdomFrame、BoxMin、BoxMax、Round、DiagWidth |
| [`src/planning/poly_planner/include/fsto/config.h`](../src/planning/poly_planner/include/fsto/config.h) | “config”模块的接口/数据结构声明；定义 Config |
| [`src/planning/poly_planner/include/fsto/fsto.h`](../src/planning/poly_planner/include/fsto/fsto.h) | “fsto”模块的接口/数据结构声明；定义 Visualization、MavGlobalPlanner |
| [`src/planning/poly_planner/include/fsto/point.h`](../src/planning/poly_planner/include/fsto/point.h) | “point”模块的接口/数据结构声明；定义 MavGlobalPlanner |
| [`src/planning/poly_planner/include/gcopter/backup/geo_utils.hpp`](../src/planning/poly_planner/include/gcopter/backup/geo_utils.hpp) | “geo_utils”模块的接口/数据结构声明；定义 filterLess |
| [`src/planning/poly_planner/include/gcopter/backup/minco.hpp`](../src/planning/poly_planner/include/gcopter/backup/minco.hpp) | MINCO 多项式轨迹的接口/数据结构声明；定义 is、BandedSystem、MINCO_S2NU |
| [`src/planning/poly_planner/include/gcopter/gcopter.hpp`](../src/planning/poly_planner/include/gcopter/gcopter.hpp) | “gcopter”模块的接口/数据结构声明；定义 is、BandedSystem、MinJerkOpt |
| [`src/planning/poly_planner/include/gcopter/geoutils.hpp`](../src/planning/poly_planner/include/gcopter/geoutils.hpp) | “geoutils”模块的接口/数据结构声明；定义 filterLess |
| [`src/planning/poly_planner/include/gcopter/lbfgs.hpp`](../src/planning/poly_planner/include/gcopter/lbfgs.hpp) | L-BFGS 优化的接口/数据结构声明；定义 lbfgs_parameter_t、callback_data_t、iteration_data_t |
| [`src/planning/poly_planner/include/gcopter/quickhull.hpp`](../src/planning/poly_planner/include/gcopter/quickhull.hpp) | Quickhull 凸包的接口/数据结构声明；定义 Vector3、Plane、Ray |
| [`src/planning/poly_planner/include/gcopter/sdlp.hpp`](../src/planning/poly_planner/include/gcopter/sdlp.hpp) | 线性规划求解的接口/数据结构声明 |
| [`src/planning/poly_planner/launch/eight.launch`](../src/planning/poly_planner/launch/eight.launch) | ROS 启动文件：“eight”模块；节点包 poly_planner、rviz |
| [`src/planning/poly_planner/launch/point.launch`](../src/planning/poly_planner/launch/point.launch) | ROS 启动文件：“point”模块；节点包 poly_planner、rviz |
| [`src/planning/poly_planner/launch/rviz.launch`](../src/planning/poly_planner/launch/rviz.launch) | ROS 启动文件：“rviz”模块；节点包 rviz |
| [`src/planning/poly_planner/package.xml`](../src/planning/poly_planner/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/poly_planner/src/fsto.cpp`](../src/planning/poly_planner/src/fsto.cpp) | “fsto”模块的实现源码 |
| [`src/planning/poly_planner/src/fsto_node.cpp`](../src/planning/poly_planner/src/fsto_node.cpp) | “fsto_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/planning/poly_planner/src/point.cpp`](../src/planning/poly_planner/src/point.cpp) | “point”模块的实现源码 |
| [`src/planning/poly_planner/src/point_node.cpp`](../src/planning/poly_planner/src/point_node.cpp) | “point_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/planning/poly_planner/src/test.cpp`](../src/planning/poly_planner/src/test.cpp) | “test”模块相关测试源码 |

## src/planning/traj_opt（12 个）

目标预测与轨迹规划相关包及 DecompROS 依赖。

| 文件 | 作用概况 |
|---|---|
| [`src/planning/traj_opt/CMakeLists.txt`](../src/planning/traj_opt/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/planning/traj_opt/include/traj_opt/geoutils.hpp`](../src/planning/traj_opt/include/traj_opt/geoutils.hpp) | “geoutils”模块的接口/数据结构声明；定义 filterLess |
| [`src/planning/traj_opt/include/traj_opt/lbfgs_raw.hpp`](../src/planning/traj_opt/include/traj_opt/lbfgs_raw.hpp) | L-BFGS 优化的接口/数据结构声明；定义 lbfgs_parameter_t、callback_data_t、iteration_data_t |
| [`src/planning/traj_opt/include/traj_opt/minco.hpp`](../src/planning/traj_opt/include/traj_opt/minco.hpp) | MINCO 多项式轨迹的接口/数据结构声明；定义 is、BandedSystem、MinJerkOpt |
| [`src/planning/traj_opt/include/traj_opt/poly_traj_utils.hpp`](../src/planning/traj_opt/include/traj_opt/poly_traj_utils.hpp) | “poly_traj_utils”模块的接口/数据结构声明；定义 Piece、Trajectory |
| [`src/planning/traj_opt/include/traj_opt/quickhull.hpp`](../src/planning/traj_opt/include/traj_opt/quickhull.hpp) | Quickhull 凸包的接口/数据结构声明；定义 Vector3、Plane、Ray |
| [`src/planning/traj_opt/include/traj_opt/root_finder.hpp`](../src/planning/traj_opt/include/traj_opt/root_finder.hpp) | 求根工具的接口/数据结构声明 |
| [`src/planning/traj_opt/include/traj_opt/sdlp.hpp`](../src/planning/traj_opt/include/traj_opt/sdlp.hpp) | 线性规划求解的接口/数据结构声明 |
| [`src/planning/traj_opt/include/traj_opt/traj_opt.h`](../src/planning/traj_opt/include/traj_opt/traj_opt.h) | 轨迹优化的接口/数据结构声明；定义 TrajOpt |
| [`src/planning/traj_opt/package.xml`](../src/planning/traj_opt/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/planning/traj_opt/src/traj_opt.cc`](../src/planning/traj_opt/src/traj_opt.cc) | 轨迹优化的实现源码 |
| [`src/planning/traj_opt/src/traj_opt_fake.cc`](../src/planning/traj_opt/src/traj_opt_fake.cc) | 轨迹优化的实现源码 |

## src/uav_simulator/local_sensing（22 个）

无人机和传感器仿真。

| 文件 | 作用概况 |
|---|---|
| [`src/uav_simulator/local_sensing/CMakeLists.txt`](../src/uav_simulator/local_sensing/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/uav_simulator/local_sensing/CMakeModules/FindCUDA.cmake`](../src/uav_simulator/local_sensing/CMakeModules/FindCUDA.cmake) | CMake 构建/模板辅助文件 |
| [`src/uav_simulator/local_sensing/CMakeModules/FindCUDA/make2cmake.cmake`](../src/uav_simulator/local_sensing/CMakeModules/FindCUDA/make2cmake.cmake) | CMake 构建/模板辅助文件 |
| [`src/uav_simulator/local_sensing/CMakeModules/FindCUDA/parse_cubin.cmake`](../src/uav_simulator/local_sensing/CMakeModules/FindCUDA/parse_cubin.cmake) | CMake 构建/模板辅助文件 |
| [`src/uav_simulator/local_sensing/CMakeModules/FindCUDA/run_nvcc.cmake`](../src/uav_simulator/local_sensing/CMakeModules/FindCUDA/run_nvcc.cmake) | CMake 构建/模板辅助文件 |
| [`src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake`](../src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake) | CMake 构建/模板辅助文件 |
| [`src/uav_simulator/local_sensing/cfg/local_sensing_node.cfg`](../src/uav_simulator/local_sensing/cfg/local_sensing_node.cfg) | 局部传感器仿真动态参数定义或配置 |
| [`src/uav_simulator/local_sensing/package.xml`](../src/uav_simulator/local_sensing/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/uav_simulator/local_sensing/params/camera.yaml`](../src/uav_simulator/local_sensing/params/camera.yaml) | “camera”模块配置参数；顶层键：cam_width、cam_height、cam_fx、cam_fy、cam_cx |
| [`src/uav_simulator/local_sensing/src/AlignError.h`](../src/uav_simulator/local_sensing/src/AlignError.h) | “AlignError”模块的接口/数据结构声明；定义 AlignError |
| [`src/uav_simulator/local_sensing/src/ceres_extensions.h`](../src/uav_simulator/local_sensing/src/ceres_extensions.h) | “ceres_extensions”模块的接口/数据结构声明；定义 EigenQuaternionParameterization |
| [`src/uav_simulator/local_sensing/src/csv_convert.py`](../src/uav_simulator/local_sensing/src/csv_convert.py) | “csv_convert”模块的 Python 实现或测试 |
| [`src/uav_simulator/local_sensing/src/cuda_exception.cuh`](../src/uav_simulator/local_sensing/src/cuda_exception.cuh) | “cuda_exception”模块的接口/数据结构声明；定义 CudaException |
| [`src/uav_simulator/local_sensing/src/depth_render.cu`](../src/uav_simulator/local_sensing/src/depth_render.cu) | 深度图渲染的实现源码 |
| [`src/uav_simulator/local_sensing/src/depth_render.cuh`](../src/uav_simulator/local_sensing/src/depth_render.cuh) | 深度图渲染的接口/数据结构声明；定义 Parameter、DepthRender |
| [`src/uav_simulator/local_sensing/src/device_image.cuh`](../src/uav_simulator/local_sensing/src/device_image.cuh) | “device_image”模块的接口/数据结构声明；定义 Size、DeviceImage |
| [`src/uav_simulator/local_sensing/src/empty.cpp`](../src/uav_simulator/local_sensing/src/empty.cpp) | “empty”模块的实现源码 |
| [`src/uav_simulator/local_sensing/src/empty.h`](../src/uav_simulator/local_sensing/src/empty.h) | “empty”模块的接口/数据结构声明 |
| [`src/uav_simulator/local_sensing/src/euroc.cpp`](../src/uav_simulator/local_sensing/src/euroc.cpp) | “euroc”模块的实现源码 |
| [`src/uav_simulator/local_sensing/src/helper_math.h`](../src/uav_simulator/local_sensing/src/helper_math.h) | “helper_math”模块的接口/数据结构声明 |
| [`src/uav_simulator/local_sensing/src/pcl_render_node.cpp`](../src/uav_simulator/local_sensing/src/pcl_render_node.cpp) | “pcl_render_node”模块的 ROS 节点/Nodelet 实现 |
| [`src/uav_simulator/local_sensing/src/pointcloud_render_node.cpp`](../src/uav_simulator/local_sensing/src/pointcloud_render_node.cpp) | 点云渲染的 ROS 节点/Nodelet 实现 |

## src/uav_simulator/mockamap（8 个）

无人机和传感器仿真。

| 文件 | 作用概况 |
|---|---|
| [`src/uav_simulator/mockamap/CMakeLists.txt`](../src/uav_simulator/mockamap/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/uav_simulator/mockamap/include/maps.hpp`](../src/uav_simulator/mockamap/include/maps.hpp) | “maps”模块的接口/数据结构声明；定义 Maps、BasicInfo、MazePoint |
| [`src/uav_simulator/mockamap/include/perlinnoise.hpp`](../src/uav_simulator/mockamap/include/perlinnoise.hpp) | “perlinnoise”模块的接口/数据结构声明；定义 PerlinNoise |
| [`src/uav_simulator/mockamap/package.xml`](../src/uav_simulator/mockamap/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/uav_simulator/mockamap/src/ces_randommap.cpp`](../src/uav_simulator/mockamap/src/ces_randommap.cpp) | “ces_randommap”模块的实现源码 |
| [`src/uav_simulator/mockamap/src/maps.cpp`](../src/uav_simulator/mockamap/src/maps.cpp) | “maps”模块的实现源码 |
| [`src/uav_simulator/mockamap/src/mockamap.cpp`](../src/uav_simulator/mockamap/src/mockamap.cpp) | 模拟地图的实现源码 |
| [`src/uav_simulator/mockamap/src/perlinnoise.cpp`](../src/uav_simulator/mockamap/src/perlinnoise.cpp) | “perlinnoise”模块的实现源码 |

## src/uav_simulator/odom_vis（28 个）

无人机和传感器仿真。

| 文件 | 作用概况 |
|---|---|
| [`src/uav_simulator/odom_vis/odom_visualization/CMakeLists.txt`](../src/uav_simulator/odom_vis/odom_visualization/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/uav_simulator/odom_vis/odom_visualization/Makefile`](../src/uav_simulator/odom_vis/odom_visualization/Makefile) | make 构建入口 |
| [`src/uav_simulator/odom_vis/odom_visualization/bin/odom_visualization`](../src/uav_simulator/odom_vis/odom_visualization/bin/odom_visualization) | 仓库内保存的里程计可视化可执行程序 |
| [`src/uav_simulator/odom_vis/odom_visualization/bin/odom_visualization_vicon45`](../src/uav_simulator/odom_vis/odom_visualization/bin/odom_visualization_vicon45) | 仓库内保存的里程计可视化可执行程序 |
| [`src/uav_simulator/odom_vis/odom_visualization/launch/real_vis.launch`](../src/uav_simulator/odom_vis/odom_visualization/launch/real_vis.launch) | ROS 启动文件：“real_vis”模块；节点包 odom_visualization |
| [`src/uav_simulator/odom_vis/odom_visualization/launch/sim_vis.launch`](../src/uav_simulator/odom_vis/odom_visualization/launch/sim_vis.launch) | ROS 启动文件：“sim_vis”模块；节点包 odom_visualization |
| [`src/uav_simulator/odom_vis/odom_visualization/launch/vins_vis.launch`](../src/uav_simulator/odom_vis/odom_visualization/launch/vins_vis.launch) | ROS 启动文件：“vins_vis”模块；节点包 odom_visualization |
| [`src/uav_simulator/odom_vis/odom_visualization/mainpage.dox`](../src/uav_simulator/odom_vis/odom_visualization/mainpage.dox) | Doxygen 项目说明 |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/bottle.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/bottle.dae) | “HIARM_DAE”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_1_base.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_1_base.dae) | “HIARM_DAE”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_1_rotate.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_1_rotate.dae) | “HIARM_DAE”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_2.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_2.dae) | “HIARM_DAE”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_3_base.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_3_base.dae) | “HIARM_DAE”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_3_rotate.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_3_rotate.dae) | “HIARM_DAE”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_4.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/HIARM_DAE/module_4.dae) | “HIARM_DAE”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/Mini_car_02.STL`](../src/uav_simulator/odom_vis/odom_visualization/meshes/Mini_car_02.STL) | “Mini_car_02.STL”可视化/碰撞资源（STL） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/WheeledRobot.stl`](../src/uav_simulator/odom_vis/odom_visualization/meshes/WheeledRobot.stl) | “WheeledRobot.stl”可视化/碰撞资源（STL） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/car.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/car.dae) | “car.dae”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/f250.dae`](../src/uav_simulator/odom_vis/odom_visualization/meshes/f250.dae) | “f250.dae”可视化/碰撞资源（DAE） |
| [`src/uav_simulator/odom_vis/odom_visualization/meshes/hummingbird.mesh`](../src/uav_simulator/odom_vis/odom_visualization/meshes/hummingbird.mesh) | “hummingbird.mesh”可视化/碰撞资源（MESH） |
| [`src/uav_simulator/odom_vis/odom_visualization/package.xml`](../src/uav_simulator/odom_vis/odom_visualization/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/uav_simulator/odom_vis/odom_visualization/src/odom_visualization.cpp`](../src/uav_simulator/odom_vis/odom_visualization/src/odom_visualization.cpp) | 里程计可视化、里程计的实现源码 |
| [`src/uav_simulator/odom_vis/odom_visualization/src/odom_visualization_car.cpp`](../src/uav_simulator/odom_vis/odom_visualization/src/odom_visualization_car.cpp) | 里程计可视化、里程计的实现源码 |
| [`src/uav_simulator/odom_vis/pose_utils/CMakeLists.txt`](../src/uav_simulator/odom_vis/pose_utils/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/uav_simulator/odom_vis/pose_utils/Makefile`](../src/uav_simulator/odom_vis/pose_utils/Makefile) | make 构建入口 |
| [`src/uav_simulator/odom_vis/pose_utils/include/pose_utils.h`](../src/uav_simulator/odom_vis/pose_utils/include/pose_utils.h) | 位姿工具的接口/数据结构声明 |
| [`src/uav_simulator/odom_vis/pose_utils/package.xml`](../src/uav_simulator/odom_vis/pose_utils/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/uav_simulator/odom_vis/pose_utils/src/pose_utils.cpp`](../src/uav_simulator/odom_vis/pose_utils/src/pose_utils.cpp) | 位姿工具的实现源码 |

## src/uav_simulator/so3_controller（6 个）

无人机和传感器仿真。

| 文件 | 作用概况 |
|---|---|
| [`src/uav_simulator/so3_controller/CMakeLists.txt`](../src/uav_simulator/so3_controller/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/uav_simulator/so3_controller/config/so3_controller.yaml`](../src/uav_simulator/so3_controller/config/so3_controller.yaml) | SO(3) 姿态控制器配置参数；顶层键：g、mass |
| [`src/uav_simulator/so3_controller/include/so3_controller/so3_controller.hpp`](../src/uav_simulator/so3_controller/include/so3_controller/so3_controller.hpp) | SO(3) 姿态控制器的接口/数据结构声明；定义 SO3Controller |
| [`src/uav_simulator/so3_controller/nodelet_plugin.xml`](../src/uav_simulator/so3_controller/nodelet_plugin.xml) | “nodelet_plugin”模块的 XML 配置或插件描述 |
| [`src/uav_simulator/so3_controller/package.xml`](../src/uav_simulator/so3_controller/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/uav_simulator/so3_controller/src/so3_controller_nodelet.cpp`](../src/uav_simulator/so3_controller/src/so3_controller_nodelet.cpp) | SO(3) 姿态控制器的 ROS 节点/Nodelet 实现 |

## src/uav_simulator/so3_quadrotor（6 个）

无人机和传感器仿真。

| 文件 | 作用概况 |
|---|---|
| [`src/uav_simulator/so3_quadrotor/CMakeLists.txt`](../src/uav_simulator/so3_quadrotor/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/uav_simulator/so3_quadrotor/include/so3_quadrotor/geometry_utils.hpp`](../src/uav_simulator/so3_quadrotor/include/so3_quadrotor/geometry_utils.hpp) | “geometry_utils”模块的接口/数据结构声明 |
| [`src/uav_simulator/so3_quadrotor/include/so3_quadrotor/quadrotor_dynamics.hpp`](../src/uav_simulator/so3_quadrotor/include/so3_quadrotor/quadrotor_dynamics.hpp) | “quadrotor_dynamics”模块的接口/数据结构声明；定义 Config、Control、Cmd |
| [`src/uav_simulator/so3_quadrotor/nodelet_plugin.xml`](../src/uav_simulator/so3_quadrotor/nodelet_plugin.xml) | “nodelet_plugin”模块的 XML 配置或插件描述 |
| [`src/uav_simulator/so3_quadrotor/package.xml`](../src/uav_simulator/so3_quadrotor/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/uav_simulator/so3_quadrotor/src/so3_quadrotor_nodelet.cpp`](../src/uav_simulator/so3_quadrotor/src/so3_quadrotor_nodelet.cpp) | SO(3) 四旋翼动力学的 ROS 节点/Nodelet 实现 |

## src/uav_simulator/uav_simulator（6 个）

无人机和传感器仿真。

| 文件 | 作用概况 |
|---|---|
| [`src/uav_simulator/uav_simulator/CMakeLists.txt`](../src/uav_simulator/uav_simulator/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/uav_simulator/uav_simulator/config/mockamap.yaml`](../src/uav_simulator/uav_simulator/config/mockamap.yaml) | 模拟地图配置参数；顶层键：seed、update_freq、resolution、x_length、y_length |
| [`src/uav_simulator/uav_simulator/config/so3_controller.yaml`](../src/uav_simulator/uav_simulator/config/so3_controller.yaml) | SO(3) 姿态控制器配置参数；顶层键：g、mass |
| [`src/uav_simulator/uav_simulator/config/so3_quadrotor.yaml`](../src/uav_simulator/uav_simulator/config/so3_quadrotor.yaml) | SO(3) 四旋翼动力学配置参数；顶层键：init_x、init_y、init_z、init_yaw、simulation_rate |
| [`src/uav_simulator/uav_simulator/launch/uav_simulator.launch`](../src/uav_simulator/uav_simulator/launch/uav_simulator.launch) | ROS 启动文件：IMU；节点包 nodelet、local_sensing_node |
| [`src/uav_simulator/uav_simulator/package.xml`](../src/uav_simulator/uav_simulator/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |

## src/uav_simulator/uav_utils（1 个）

无人机和传感器仿真。

| 文件 | 作用概况 |
|---|---|
| [`src/uav_simulator/uav_utils/CMakeLists.txt`](../src/uav_simulator/uav_utils/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |

## src/utils/fake_mavros（4 个）

仿真 MAVROS、消息及 RViz 插件。

| 文件 | 作用概况 |
|---|---|
| [`src/utils/fake_mavros/CMakeLists.txt`](../src/utils/fake_mavros/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/utils/fake_mavros/launch/apm.launch`](../src/utils/fake_mavros/launch/apm.launch) | ROS 启动文件：“apm”模块；节点包 joy、fake_mavros |
| [`src/utils/fake_mavros/package.xml`](../src/utils/fake_mavros/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/utils/fake_mavros/src/mavros_node.cpp`](../src/utils/fake_mavros/src/mavros_node.cpp) | “mavros_node”模块的 ROS 节点/Nodelet 实现 |

## src/utils/quadrotor_msgs（11 个）

仿真 MAVROS、消息及 RViz 插件。

| 文件 | 作用概况 |
|---|---|
| [`src/utils/quadrotor_msgs/CMakeLists.txt`](../src/utils/quadrotor_msgs/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/utils/quadrotor_msgs/msg/AuxCommand.msg`](../src/utils/quadrotor_msgs/msg/AuxCommand.msg) | ROS 消息：AuxCommand；字段包括 current_yaw、kf_correction、pitch、enable_motors、use_external_yaw |
| [`src/utils/quadrotor_msgs/msg/GraspState.msg`](../src/utils/quadrotor_msgs/msg/GraspState.msg) | ROS 消息：GraspState；字段包括 header、graspState、graspDegree、replan_stamp |
| [`src/utils/quadrotor_msgs/msg/OccMap3d.msg`](../src/utils/quadrotor_msgs/msg/OccMap3d.msg) | ROS 消息：OccMap3d；字段包括 header、resolution、size_x、size_y、size_z、offset_x 等 |
| [`src/utils/quadrotor_msgs/msg/PolyTraj.msg`](../src/utils/quadrotor_msgs/msg/PolyTraj.msg) | ROS 消息：PolyTraj；字段包括 drone_id、traj_id、start_time、hover、yaw、hover_p 等 |
| [`src/utils/quadrotor_msgs/msg/PositionCommand.msg`](../src/utils/quadrotor_msgs/msg/PositionCommand.msg) | ROS 消息：PositionCommand；字段包括 header、position、velocity、acceleration、jerk、yaw 等 |
| [`src/utils/quadrotor_msgs/msg/Px4ctrlDebug.msg`](../src/utils/quadrotor_msgs/msg/Px4ctrlDebug.msg) | ROS 消息：Px4ctrlDebug；字段包括 header、des_v_x、des_v_y、des_v_z、fb_a_x、fb_a_y 等 |
| [`src/utils/quadrotor_msgs/msg/ReplanState.msg`](../src/utils/quadrotor_msgs/msg/ReplanState.msg) | ROS 消息：ReplanState；字段包括 header、state、iniState、target、occmap、path 等 |
| [`src/utils/quadrotor_msgs/msg/SO3Command.msg`](../src/utils/quadrotor_msgs/msg/SO3Command.msg) | ROS 消息：SO3Command；字段包括 header、force、orientation、kR、kOm、aux |
| [`src/utils/quadrotor_msgs/msg/TakeoffLand.msg`](../src/utils/quadrotor_msgs/msg/TakeoffLand.msg) | ROS 消息：TakeoffLand；字段包括 takeoff_land_cmd |
| [`src/utils/quadrotor_msgs/package.xml`](../src/utils/quadrotor_msgs/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |

## src/utils/rviz_plugins（14 个）

仿真 MAVROS、消息及 RViz 插件。

| 文件 | 作用概况 |
|---|---|
| [`src/utils/rviz_plugins/CMakeLists.txt`](../src/utils/rviz_plugins/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/utils/rviz_plugins/config/rviz_config.rviz`](../src/utils/rviz_plugins/config/rviz_config.rviz) | “rviz_config”模块的 RViz 显示布局 |
| [`src/utils/rviz_plugins/package.xml`](../src/utils/rviz_plugins/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/utils/rviz_plugins/plugin_description.xml`](../src/utils/rviz_plugins/plugin_description.xml) | “plugin_description”模块的 XML 配置或插件描述 |
| [`src/utils/rviz_plugins/src/aerialmap_display.cpp`](../src/utils/rviz_plugins/src/aerialmap_display.cpp) | “aerialmap_display”模块的实现源码 |
| [`src/utils/rviz_plugins/src/aerialmap_display.h`](../src/utils/rviz_plugins/src/aerialmap_display.h) | “aerialmap_display”模块的接口/数据结构声明；定义 ManualObject、FloatProperty、IntProperty |
| [`src/utils/rviz_plugins/src/gamelikeinput.cpp`](../src/utils/rviz_plugins/src/gamelikeinput.cpp) | “gamelikeinput”模块的实现源码 |
| [`src/utils/rviz_plugins/src/gamelikeinput.hpp`](../src/utils/rviz_plugins/src/gamelikeinput.hpp) | “gamelikeinput”模块的接口/数据结构声明；定义 Arrow、StringProperty、GameLikeInput |
| [`src/utils/rviz_plugins/src/goal_tool.cpp`](../src/utils/rviz_plugins/src/goal_tool.cpp) | “goal_tool”模块的实现源码 |
| [`src/utils/rviz_plugins/src/goal_tool.h`](../src/utils/rviz_plugins/src/goal_tool.h) | “goal_tool”模块的接口/数据结构声明；定义 Arrow、DisplayContext、StringProperty |
| [`src/utils/rviz_plugins/src/pose_tool.cpp`](../src/utils/rviz_plugins/src/pose_tool.cpp) | “pose_tool”模块的实现源码 |
| [`src/utils/rviz_plugins/src/pose_tool.h`](../src/utils/rviz_plugins/src/pose_tool.h) | “pose_tool”模块的接口/数据结构声明；定义 Arrow、DisplayContext、Pose3DTool |
| [`src/utils/rviz_plugins/src/probmap_display.cpp`](../src/utils/rviz_plugins/src/probmap_display.cpp) | “probmap_display”模块的实现源码 |
| [`src/utils/rviz_plugins/src/probmap_display.h`](../src/utils/rviz_plugins/src/probmap_display.h) | “probmap_display”模块的接口/数据结构声明；定义 ManualObject、FloatProperty、IntProperty |

## src/vrpn_client_ros（9 个）

VRPN 动捕数据接入。

| 文件 | 作用概况 |
|---|---|
| [`src/vrpn_client_ros/CHANGELOG.rst`](../src/vrpn_client_ros/CHANGELOG.rst) | 该目录的说明文档 |
| [`src/vrpn_client_ros/CMakeLists.txt`](../src/vrpn_client_ros/CMakeLists.txt) | CMake/catkin 构建规则：声明目标、依赖和安装方式 |
| [`src/vrpn_client_ros/LICENSE`](../src/vrpn_client_ros/LICENSE) | 该子项目的许可证文本 |
| [`src/vrpn_client_ros/include/vrpn_client_ros/vrpn_client_ros.h`](../src/vrpn_client_ros/include/vrpn_client_ros/vrpn_client_ros.h) | VRPN 客户端的接口/数据结构声明；定义 VrpnTrackerRos、VrpnClientRos |
| [`src/vrpn_client_ros/launch/sample.launch`](../src/vrpn_client_ros/launch/sample.launch) | ROS 启动文件：“sample”模块；节点包 vrpn_client_ros |
| [`src/vrpn_client_ros/package.xml`](../src/vrpn_client_ros/package.xml) | ROS 包清单：包名、版本、维护者和构建/运行依赖 |
| [`src/vrpn_client_ros/src/vrpn_client_node.cpp`](../src/vrpn_client_ros/src/vrpn_client_node.cpp) | VRPN 客户端的 ROS 节点/Nodelet 实现 |
| [`src/vrpn_client_ros/src/vrpn_client_ros.cpp`](../src/vrpn_client_ros/src/vrpn_client_ros.cpp) | VRPN 客户端的实现源码 |
| [`src/vrpn_client_ros/src/vrpn_tracker_node.cpp`](../src/vrpn_client_ros/src/vrpn_tracker_node.cpp) | “vrpn_tracker_node”模块的 ROS 节点/Nodelet 实现 |

## 生成文件与本地状态

- `build/`、`build_catkin/`：catkin/CMake 编译中间文件。
- `devel/`：编译后的 ROS 环境、可执行文件和生成消息。
- `logs_catkin/`、`docker/logs/`：编译和容器运行日志。
- `.venv/`：uv/Python 虚拟环境及第三方包。
- `.git/`：Git 对象、引用和工作树元数据。
- `.runtime_captures/`、`output/`、`.playwright-cli/`：运行抓取与工具输出。
- `__pycache__/`、`.ipynb_checkpoints/`：Python 与 Notebook 缓存。

本导览不修改上述文件。当前工作区已有未提交修改和新增文件；具体状态以 `git status --short` 为准。
