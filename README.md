# HIARM

基于 **ROS 1 Noetic / Gazebo Classic** 的可变形飞行抓取机器人工作区，包含机器人模型、执行器驱动、状态估计、飞行控制、轨迹规划及仿真插件。机器人名称为 `flygripper`。

当前仓库提供两套入口：

- **Docker 仿真**：在 Linux 容器中运行物理与控制程序，通过 WebGL 或 noVNC 查看，适合当前 Apple Silicon Mac 环境。
- **原生 ROS 工作区脚本**：通过 `run.sh`、`ctrl.sh`、`plan.sh` 分别启动模型、控制和规划；需要先准备 Linux/ROS 环境并编译工作区。

本页提供项目概览、快速开始及代码修改说明。环境变量、启动流程、场景参数、日志和历史运行记录统一放在 **[Docker 仿真使用手册](docker/README.md)**。

## 目录

- [快速开始](#快速开始)
- [显示方式与运行模式](#显示方式与运行模式)
- [项目目录](#项目目录)
- [原生 ROS 启动流程](#原生-ros-启动流程)
- [主要话题与参数](#主要话题与参数)
- [当前仿真适配的修改范围](#当前仿真适配的修改范围)
- [结果判读与模型边界](#结果判读与模型边界)
- [修改后如何生效](#修改后如何生效)

## 快速开始

### 1. 准备环境

当前 Docker 配置指定 `linux/arm64`，镜像提供 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 和编译依赖。先启动 Docker Desktop，再在 **Mac 终端、仓库根目录**执行下面的命令；宿主机不需要安装 ROS。

### 2. 启动 WebGL 仿真

```bash
HIARM_DESKTOP=false HIARM_GUI=false ./docker/sim.sh start
```

打开 **<http://localhost:8080>**。首次启动会编译工作区，等待页面显示“仿真实时连接”，再点击“起飞并执行任务”。

`start` 启动后台服务，任务由网页按钮触发。首次编译进度可用 `./docker/sim.sh logs` 查看。

### 3. 使用 noVNC 桌面

```bash
HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh start
```

打开 **<http://localhost:6080/vnc.html?autoconnect=true&resize=scale>**。

桌面提供 Gazebo、RViz 和任务栏。窗口最小化后可通过任务栏或 `Alt+Tab` 恢复；双击标题栏可以最大化。WebGL 页面仍可同时使用。

### 4. 自动执行任务

以下命令会等待工作区编译完成，再等待 ROS 就绪并执行任务，按需选择：

```bash
# 短演示：起飞至 1 m，再飞向 (0.6, 0, 1) m
./docker/sim.sh demo

# 完整演示：重置场景，执行十阶段、四物体抓取搬运
./docker/sim.sh mission

# 完整演示，同时保留 noVNC、Gazebo GUI 和 RViz
HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh mission
```

`demo` 和 `mission` 都会重新创建容器并重置本轮仿真，复用已有镜像；首次准备环境或修改 Dockerfile 后，先执行 `./docker/sim.sh build`。按 Ctrl+C 或脚本以非零状态退出时会停止并移除容器；正常退出后保留容器。四次抓取是否通过，查看下文的结果字段。

命令前的环境变量只对该次调用生效，后续任务若需要桌面，应再次带上开关。

### 5. 管理服务

```bash
./docker/sim.sh logs       # 查看容器启动输出
./docker/sim.sh shell      # 进入容器并加载工作区环境
./docker/sim.sh restart    # 重置现有容器中的仿真，沿用已有环境配置
./docker/sim.sh stop       # 停止并移除容器，保留构建卷和镜像
```

修改环境变量后使用 `start` 应用；`restart` 不会更新容器环境。仅刷新浏览器不会重置物理仿真。

**不要在 `root@容器:/ws#` 内执行 `docker/sim.sh`**：它是宿主机管理入口，容器内没有 Docker CLI。

## 显示方式与运行模式

### 显示方式

| 方式 | 浏览器看到什么 | 三维渲染位置 | 适用场景 |
|---|---|---|---|
| WebGL，8080 | 根据模型与实时姿态绘制的网页场景 | Mac 浏览器，可使用本机 GPU | 演示、观察运动、执行集成任务 |
| noVNC，6080 | 容器 Linux 桌面上的 Gazebo/RViz 窗口 | 容器内，当前采用 CPU 软件渲染 | 查看原软件界面、调试 ROS 数据 |

两者使用同一次 Gazebo 仿真。`HIARM_GUI=false` 关闭自动启动的 `gzclient`，`HIARM_DESKTOP=false` 关闭桌面及自动启动的 RViz；**`gzserver`、控制器和物理计算仍然运行**。WebGL 加速显示，不直接加速物理积分。

RViz 默认显示机器人、里程计和参考路径；参考路径需要任务产生数据后才出现。它不会自动显示 Gazebo 世界中的易拉罐和支撑柱。

### 任务模式

| 模式 | 配置与场景 | 任务内容 |
|---|---|---|
| `demo` | `demo-plan.yaml`、`basic.world` | 起飞、悬停、单段轨迹 |
| `original`，默认 | 原 `real_sim.yaml` 加 `grasp-plan.yaml` 覆盖、`matched.world` | 十阶段四物体演示 |

完整任务依次抓取和释放易拉罐、球体、瓶子、方块，另有初始到位和最后收拢动作。

`original` 表示使用原十阶段任务作为基础。当前匹配场景、接近路径、速度、搬运航向和部分抓取逻辑已有仿真适配。

## 项目目录

| 路径 | 职责 |
|---|---|
| [docker/](docker/README.md) | 容器环境、统一启动、WebGL、noVNC、RViz、仿真任务及报告 |
| `src/flygripper_description/` | Xacro/URDF、网格及物体模型 |
| `src/flygripper_bringup/` | 模型、Gazebo 和硬件执行器的启动组合 |
| `src/flygripper_actuator/` | 执行器、舵机和电机接口 |
| `src/flygripper_common/`、`src/flygripper_msgs/` | 公共类型、运动学映射及 ROS 消息 |
| `src/flygripper_estimator/` | 状态估计相关节点和配置 |
| `src/flygripper_flight/` | 飞行状态机、数据接口和控制调用 |
| `src/flygripper_controller/` | 控制器及参数 |
| `src/flygripper_planner/` | 多阶段规划器及轨迹生成库 |
| `src/flygripper_sim/` | Gazebo 世界、启动文件及旋翼/夹爪/抓取等插件 |
| `src/flygripper_teleop/` | 遥控相关功能 |
| `src/ekf_pose/`、`src/vrpn_client_ros/` | 外部定位及相关估计接口 |
| `src/utils/fake_mavros/`、`src/utils/rviz_plugins/` | 模拟飞控接口及 RViz 插件 |

## 原生 ROS 启动流程

在已安装 ROS Noetic、Gazebo 和项目依赖的 Linux 主机上，先在仓库根目录编译：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release
```

随后分别在三个终端中、仓库根目录执行：

```bash
# 终端 1：Gazebo、机器人模型和模拟 MAVROS
./run.sh flygripper

# 终端 2：重置仿真世界并启动飞行控制
./ctrl.sh flygripper

# 终端 3：启动 real 多阶段规划器
./plan.sh flygripper
```

三个脚本会自行加载 `devel/setup.bash`，默认使用 `flygripper` 和 `USE_SIM=true`。Gazebo 窗口由 `run.sh` 中的 `GUI=true` 控制。

实际启动关系：

1. `run.sh` 调用 `flygripper_bringup/run.launch` 加载模型和 Gazebo，再由 `run.sh` 的仿真分支单独启动 `fake_mavros/apm.launch`。原生 `urdf.launch` 中 RViz 节点目前被注释，不会默认打开。
2. `ctrl.sh` 的仿真分支调用 `reset_world.launch`，随后启动 `flygripper_flight/flight.launch`；当前脚本没有单独启动估计器或摇杆节点。
3. `plan.sh` 当前启动 `flygripper_planner/real.launch`；其他规划示例入口保留为注释。

这条流程使用 `basic.world` 和原 `real_sim.yaml`。三个脚本启动节点后，还需要遥控模式输入触发飞行；Docker 的虚拟遥控和任务编排由 `docker/start.sh`、`mission.py` 提供。两套入口分别使用，避免重复启动节点。

仿真控制由项目内的 `GeometricController` 执行，`fake_mavros` 提供模拟飞控接口；当前流程不启动 PX4 SITL。

### 其他工具

- [fake_ctrl.sh](fake_ctrl.sh)：另一套模拟飞控、估计器和控制启动组合，不是 Docker 必需步骤。
- [bag.sh](bag.sh)：按脚本中指定的话题记录 ROS bag；当前清单偏向 MAVROS/硬件接口，仿真记录前应核对话题。
- [rename.sh](rename.sh)：文件重命名辅助脚本，使用前检查作用范围。
- `collect_data.py`、`collect_step_data.py`、`collect_continuous_data.py`：数据采集脚本，按具体实验检查参数后使用。

在已经加载 ROS 和工作区环境的终端中，可展开 Xacro：

```bash
rosrun xacro xacro src/flygripper_description/urdf/flygripper.xacro \
  namespace:=flygripper use_sim:=true > flygripper.urdf
```

## 主要话题与参数

### 仿真与硬件接口

以下按当前 `flight.launch` 和 Docker 配置整理，`flygripper` 为默认命名空间：

| 数据 | 当前仿真话题 | 原生硬件分支的接口 |
|---|---|---|
| IMU | `/flygripper/imu` | `/mavros/imu/data` |
| 里程计 | `/flygripper/odom` | `/ekf/ekf_odom` |
| 遥控输入 | `/mavros/rc/in`，Docker 由虚拟遥控发布 | `/mavros/rc/in` |
| 飞控状态 | `/mavros/state`，模拟节点提供 | `/mavros/state` |
| 旋翼控制 | `/flygripper/rotor_command` | `/mavros/actuator_control` |
| 旋翼转速 | `/flygripper/rotor_speed` | `/mavros/esc_telemetry` |
| 执行器状态/指令 | `/flygripper/actuator_state`、`/flygripper/actuator_command` | 由对应驱动与命名空间配置决定 |

其他仿真接口包括 `/gazebo/link_states`、`/flygripper/reference`、`/flygripper/trajectory` 和 `/flygripper/grasp_status`。抓取状态是连接/释放事件，不是持续状态心跳。

## 当前仿真适配的修改范围

Docker 运行涉及以下源码适配：

| 类别 | 主要文件/内容 | 原因与影响 |
|---|---|---|
| 编译入口 | `src/CMakeLists.txt` | 将 XSym 占位文本替换为标准 catkin 入口 |
| 编译依赖 | `poly_planner`、Gazebo 插件、`fake_mavros` 的 CMake | 明确生成消息与编译目标之间的依赖 |
| 文件筛选 | actuator、catkin_simple 的 CMake | 避免 AppleDouble 元数据参与编译；与 Docker 源码同步过滤部分重复 |
| RViz 依赖 | `rviz_plugins` 的 CMake/package.xml | 移除源码未使用的 `multi_map_server` 依赖声明；保留地图显示代码 |
| 模型加载 | `flygripper.xacro`、`flygripper_base.xacro` | 使用相对包含路径，设置 `namespace` 和 `use_sim` 的默认值 |
| 旋翼初始化 | `gazebo_rotor_plugin.hpp` | 读取时间常数后再构造滤波器 |
| 夹爪控制 | `gazebo_reconfigurable_plugin.hpp`、Xacro | 初始化更新时间、限制目标变化速度、按最落后关节反馈到位状态 |
| 抓取连接 | `gazebo_grasp_plugin.hpp`、Xacro | 使用固定连接、修正锚点坐标、排除同连杆伪双侧接触、增加按指令释放策略 |
| 接触诊断 | 抓取插件 | 记录易拉罐、旋翼和支撑柱碰撞，辅助定位抖动 |

上述插件和 Xacro 修改同时影响 Docker 与原生 ROS 启动的 `flygripper` 仿真。

### 夹爪反馈的含义

`0`（展开）和 `5`（闭合）是统一执行器控制坐标。滑动和转动关节分别通过比例及死区映射得到自己的目标位置。当前 `commandRateLimit=1.0` 表示参考值每秒最多变化 1 个控制单位；0→5 的参考变化需要约 5 秒，到位由实际反馈判断。网页当前标注的 `rad` 不适用于这个统一坐标。

单值反馈代表跟踪最终目标误差最大的关节，避免只看第一根滑轨就宣布全机构到位。`effort` 是选中关节控制量的映射，不是实测夹持力。

### 抓取逻辑的边界

抓取采用固定关节连接。`commandedRelease=true` 时，收到张爪指令并满足位置及释放条件后解除连接。

全机接触日志属于诊断功能，可与抓取行为调整分开维护。详细参数和历史结果见 [Docker 手册](docker/README.md#8-任务场景与抓取参数)。

## 结果判读与模型边界

完整演示结果保存到 `docker/logs/mission.json`：

| 字段 | 表示什么 |
|---|---|
| `passed` | 十阶段规划器报告完成 |
| `physical_grasp_verified` | 至少出现过一次连接事件 |
| `all_transfers_verified` | 四个物体的连接、搬运和计划释放全部通过 |
| `can_motion_validation.stable` | 指定易拉罐抓取时间窗内的稳定性判据通过 |
| `grasp_and_stability_verified` | 四次搬运与上述稳定性同时通过 |

读取 `grasp_and_stability_verified` 判断四次搬运及易拉罐稳定性是否全部通过。当前 `mission.py` 即使该字段为 false，也可能正常退出；命令退出码 0 只表示脚本执行结束。短演示结果单独保存在 `docker/logs/verification.json`。

当前抓取采用**Gazebo 碰撞接触触发、固定约束保持**的模型。连接后禁用被抓物体的碰撞，释放时恢复。报告检查连接、搬运和释放时序，未检查最终落点。稳定性检查当前未检测采样中断，详见 [报告说明](docker/README.md#101-结果字段)。

历史三轮四物体搬运记录及稳定性指标见 [历史结果](docker/README.md#11-历史结果和适用边界)。2026-10-10 新增的工作区编译等待逻辑尚未运行仿真验证。

运行日志和截图通常被 Git 忽略，新克隆未必包含历史证据；固定名称的报告可能被下一轮覆盖。

## 修改后如何生效

Docker 使用独立的源码、构建和产物卷。Mac 的 `src` 只读挂载到 `/host-src`，再由 `build.sh` 同步至实际编译的 `/ws/src`。已有构建成功标记时，重启会复用卷内源码和产物。

修改 C++、URDF 或包内配置后，推荐：

```bash
./docker/sim.sh stop
./docker/sim.sh build
./docker/sim.sh start
```

需要桌面时，在最后一条命令前加上 `HIARM_DESKTOP=true HIARM_GUI=true`。

- Dockerfile / Compose：用 `start` 构建并应用配置。
- `docker/` 下启动脚本、world 和任务 YAML：通常需要重启对应进程或仿真。
- WebGL 页面样式和脚本：刷新网页。
- 场景与任务生成器：先重新生成输出文件，再重启；启动脚本不会自动运行生成器。

提交时建议分开审核编译修复、旋翼修复、Docker 基础运行、抓取行为、第三方依赖、WebGL 和桌面功能。不要将 `.DS_Store`、构建产物、运行日志或浏览器截图一并提交。Three.js 本地依赖及许可证见 [第三方资源说明](docker/README.md#threejs-来源与许可)。
