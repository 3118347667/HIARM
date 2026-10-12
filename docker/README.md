# HIARM Docker 仿真使用手册

本手册说明 Docker 仿真的启动命令、参数、显示方式、抓取场景和结果报告。项目概览见 [根目录 README](../README.md)，历史运行数据集中在第 11 节。

## 目录

- [1. 快速启动](#1-快速启动)
- [2. 三类模式及其关系](#2-三类模式及其关系)
- [3. 启动参数和环境变量](#3-启动参数和环境变量)
- [4. 命令及重启语义](#4-命令及重启语义)
- [5. 完整启动流程](#5-完整启动流程)
- [6. Compose 配置和数据保存](#6-compose-配置和数据保存)
- [7. WebGL、noVNC 与 RViz](#7-webglnovnc-与-rviz)
- [8. 任务、场景与抓取参数](#8-任务场景与抓取参数)
- [9. 修改文件后如何生效](#9-修改文件后如何生效)
- [10. 报告、日志与故障排查](#10-报告日志与故障排查)
- [11. 历史结果和适用边界](#11-历史结果和适用边界)
- [12. 文件索引与第三方资源](#12-文件索引与第三方资源)

## 1. 快速启动

### 1.1 环境与命令位置

当前配置面向 Apple Silicon Mac，使用 Docker Desktop 的 Linux ARM64 容器。镜像提供 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 和构建工具。Docker Desktop 需要处于运行状态；本机不需要安装 ROS。建议给 Docker 分配约 8 GB 内存，实际需求随图形窗口和任务负载变化。

除特别注明，本文命令均在 **Mac 终端、仓库根目录**执行：

```bash
cd /Users/lzf/Documents/HIARM
```

不要在 `root@容器:/ws#` 内运行 `./docker/sim.sh`：它是宿主机 Docker 管理脚本，容器中没有 Docker CLI。容器内运行的是 `start.sh`、ROS 节点和任务脚本。

### 1.2 日常使用：WebGL

```bash
HIARM_DESKTOP=false HIARM_GUI=false ./docker/sim.sh start
```

打开 <http://localhost:8080>，等待“仿真实时连接”，再点击“起飞并执行任务”。`start` 启动服务，并不自动执行完整飞行任务。首次启动会编译工作区，需要等待。

### 1.3 使用 noVNC、Gazebo GUI 和 RViz

```bash
HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh start
```

打开 <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>。

启动脚本打印两个地址及 noVNC 开关状态；首次编译期间可用 `./docker/sim.sh logs` 查看进度。8080 是 WebGL，6080 是 noVNC，两种页面可以同时使用。

### 1.4 一条命令重新运行四物体任务

首次准备环境或修改 Dockerfile 后，先执行 `./docker/sim.sh build`。随后任选一条任务命令：

```bash
# WebGL 模式
HIARM_DESKTOP=false HIARM_GUI=false ./docker/sim.sh mission

# 或：同时开启桌面
HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh mission
```

`mission` 会重新创建容器并重置仿真，等待工作区编译成功和 ROS 就绪，然后自动起飞并执行任务；无需再点击网页启动按钮。

### 1.5 短飞行检查

```bash
./docker/sim.sh demo
```

该命令重置仿真，先起飞并悬停在 `(0,0,1)` m，再飞向 `(0.6,0,1)` m。结果保存在 `docker/logs/verification.json`。完整任务结果保存在 `docker/logs/mission.json`，字段含义见 [第 10 节](#10-报告日志与故障排查)。

## 2. 三类模式及其关系

### 2.1 显示模式：在哪里画图

| `HIARM_DESKTOP` | `HIARM_GUI` | 启动内容 | 用途 |
|---|---|---|---|
| `false` | `false` | Gazebo 服务端、ROS、WebGL | 默认，适合演示，容器没有图形窗口 |
| `true` | `false` | 上述服务，加 noVNC 桌面、任务栏、RViz | 查看 ROS 数据，不自动打开 Gazebo GUI |
| `true` | `true` | 上述服务，加 Gazebo GUI | 完整桌面调试 |
| `false` | `true` | Gazebo GUI 在虚拟显示中运行，但无 noVNC 入口 | 通常无须使用，会增加图形开销 |

**两个开关都为 false 时，Gazebo 仍然运行。** `gzserver` 负责物理、碰撞和仿真插件；`gzclient` 只是 Gazebo 图形窗口。Xvfb 虚拟显示也始终保留，供插件初始化使用。

桌面快捷入口可以手动打开 Gazebo/RViz，因此 `HIARM_GUI=false` 表示“不自动启动 gzclient”，并非禁止用户随后打开。

### 2.2 任务模式：加载什么世界和规划配置

| `HIARM_MISSION` | 世界文件 | 参数加载顺序 | 内容 |
|---|---|---|---|
| `original`（默认） | `docker/matched.world` | `real_sim.yaml`，再覆盖 `grasp-plan.yaml` | 十阶段、四物体抓取搬运 |
| `demo` | 包内 `worlds/basic.world` | `real_sim.yaml`，再覆盖 `demo-plan.yaml` | 起飞至 1 m，再飞向 `(0.6,0,1)` m |

`original` 使用原规划器，保留十阶段终点的 x/y/z 和变形顺序；仿真覆盖会调整接近点、速度、搬运航向和自动变形设置，世界采用匹配后的物体与支撑柱。

任务模式与显示模式独立：完整任务和短演示都能选择 WebGL 或 noVNC。`HIARM_MISSION` 使用 `original` 或 `demo`；它只选择配置，自动执行分别使用 `sim.sh mission`、`sim.sh demo`。

### 2.3 运行中的飞行控制状态

`virtual_rc.py` 以 30 Hz 发布模拟遥控输入，由 ROS 参数选择状态：

| ROS 参数 | 值 | 作用 |
|---|---|---|
| `/hiarm/demo_mode` | `manual`（默认） | 中立输入，等待任务启动；不是桌面显示模式 |
| 同上 | `hover` | 遥控通道 4 置高，进入起飞/悬停流程 |
| 同上 | `trajectory` | 通道 4、5 置高，进入轨迹执行 |
| `/hiarm/morph_high` | `false` / `true` | 控制通道 7 的低/高电平，触发人工等待阶段的变形 |

通道编号从 0 开始。`mission.py` 在适当阶段自动切换这些参数；通常无需手动修改。`HIARM_MISSION=demo` 是短任务配置，`/hiarm/demo_mode` 是运行状态，二者含义不同。

## 3. 启动参数和环境变量

### 3.1 日常开关

定义位置：[compose.yaml](compose.yaml)。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `HIARM_DESKTOP` | `false` | 启动 Openbox、tint2 任务栏、x11vnc、websockify，并自动打开配置好的 RViz |
| `HIARM_GUI` | `false` | 传给 `sim.launch` 的 `gui` 参数，控制自动启动 Gazebo GUI |
| `HIARM_MISSION` | `original` | 选择世界及任务参数覆盖文件 |
| `HIARM_PROXY_URL` | `http://host.docker.internal:7897` | 容器及镜像构建中的 HTTP/HTTPS 代理；设为空字符串关闭代理 |

布尔开关使用小写 `true` / `false`；脚本按字符串 `true` 判断。

临时赋值只对这一条命令有效：

```bash
HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh start
```

后续执行 `mission` 时，也要带上这些临时开关，否则会按照当次环境和 Compose 默认值创建容器。

若希望长期默认开启，可将 `compose.yaml` 对应行改成：

```yaml
environment:
  HIARM_GUI: ${HIARM_GUI:-true}
  HIARM_DESKTOP: ${HIARM_DESKTOP:-true}
  HIARM_MISSION: ${HIARM_MISSION:-original}
```

`${变量:-默认值}` 表示变量未设置或为空时使用默认值。命令前的显式赋值可以覆盖这里的默认值。修改后使用 `start` 应用；`restart` 不会更新容器环境变量。

若要明确回到 WebGL 模式，无论默认值如何都可执行：

```bash
HIARM_DESKTOP=false HIARM_GUI=false ./docker/sim.sh start
```

### 3.2 镜像中的图形环境

| 参数 | 当前值 | 作用 |
|---|---|---|
| `DISPLAY` | `:1` | 容器程序连接 Xvfb 虚拟显示 |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | 容器 Gazebo/RViz 使用软件 OpenGL；不控制 Mac 浏览器 |
| `QT_X11_NO_MITSHM` | `1` | 避免 Qt 的 X11 共享内存扩展兼容性问题 |
| `GAZEBO_MODEL_DATABASE_URI` | 空字符串 | 禁用远端模型数据库地址；本项目模型由本地提供 |
| `DISABLE_ROS1_EOL_WARNINGS` | `open-window.sh` 中设为 `1` | 避免 RViz 的 ROS 1 到期提示对话框挡住初始界面 |

这些值主要用于容器图形兼容性，不是 Apple GPU 直通开关。

### 3.3 Launch 和内部参数

| 参数 | 来源/当前值 | 作用 |
|---|---|---|
| `gui` | `start.sh` 从 `HIARM_GUI` 传入 | 控制 `gzclient`；launch 文件独立默认值为 true，正常脚本启动会显式覆盖 |
| `mission` | 从 `HIARM_MISSION` 传入 | 选择任务配置 |
| `world` | 按任务自动选择 | Gazebo 加载的 world 路径 |
| `/hiarm/mission`、`/hiarm/world_file` | launch 写入 | 供网页和任务脚本读取当前配置 |
| `/use_sim_time` | Gazebo 启动流程设置 | ROS 使用 `/clock` 仿真时间；任务和虚拟遥控要求它为 true |
| `auto_takeoff` | `true` | 允许进入相应控制模式后自动起飞；不表示容器启动即执行任务 |
| `velocity_teleop` | `false` | 关闭速度遥控方式 |
| 机器人初始 `z` | `0.1` m | Gazebo 模型出生高度，不是飞行目标高度 |

控制器原始增益和 L1 补偿配置仍由项目的飞控 launch/参数文件提供，本目录没有把调试失败的低增益覆盖保留下来。

## 4. 命令及重启语义

[sim.sh](sim.sh) 是宿主机入口，不传参数时等价于 `start`。

| 命令 | 实际行为 | 是否重置仿真 |
|---|---|---|
| `start` | `compose up -d --build`；构建/复用镜像，启动服务并应用配置变化 | 若重新创建容器则重置；配置未变时不保证重置 |
| `restart` | 重启现有 `sim` 容器，继续使用它已有的环境变量 | 是；不会应用新 Compose 环境或新镜像 |
| `mission` | 强制 `original`，重新创建容器，等待编译成功，再执行 `mission.py` | 是，并自动执行完整任务 |
| `demo` | 强制 `demo`，重新创建容器，等待编译成功，再执行 `verify.py` | 是，并自动执行短演示 |
| `build` | 构建镜像，在临时容器内直接编译共享源码 | 不负责重启正在运行的仿真 |
| `stop` | `compose down`，停止并移除容器及项目网络 | 当前仿真结束；保留镜像及命名卷 |
| `logs` | 跟随 Compose 容器标准输出 | 否 |
| `shell` | 进入现有容器，加载工作区环境 | 否 |

`mission` / `demo` 复用已有镜像，没有传入 `--build`。首次准备环境或修改 Dockerfile 后，先用 `build` 构建镜像和工作区，或用 `start` 构建镜像并启动服务。

`mission` / `demo` 在前台执行任务；按 Ctrl+C、容器启动失败或任务脚本以非零状态退出时，会调用 `compose down`。脚本正常退出后保留容器，结束时运行 `./docker/sim.sh stop`。`start` 是后台服务入口，命令返回后容器继续运行。

**当前退出码的含义：** 短演示验收失败会返回非零值；完整任务的抓取或稳定性字段为 false 时，`mission.py` 仍可能返回 0。完整任务结果需读取 `mission.json` 中的 `grasp_and_stability_verified`。

典型选择：

- 只是刷新画面：刷新浏览器，不重置物理仿真。
- 重新从网页执行任务：`restart`，等待连接，再点击启动。
- 一次性重置并自动执行任务：`mission`。
- 修改环境开关或镜像依赖：`start`。
- 从 `demo` 恢复完整任务：`HIARM_MISSION=original ./docker/sim.sh start`，或直接 `mission`；所需显示开关也应一并设置。

## 5. 完整启动流程

### 5.1 宿主机：sim.sh 和 Dockerfile

1. `sim.sh` 切换到仓库根目录，使用固定的 `docker/compose.yaml`。
2. `start` 构建或复用 `hiarm:noetic-arm64` 镜像。Dockerfile 固定 ROS 基础镜像摘要，安装编译工具、ROS/Gazebo/RViz、桌面和软件渲染依赖。
3. APT 使用 USTC Ubuntu 镜像、缓存挂载及重试；ROS 源来自基础镜像。
4. Compose 创建容器、映射端口并挂载源码和构建卷。
5. 容器执行 `bash docker/start.sh`；宿主机打印访问地址。后续初始化仍在容器中继续。

`demo` / `mission` 的后续流程：每隔 2 秒检查容器中的 `/ws/devel/.hiarm-build-ok`。等待期间容器停止则报错退出；标记存在后，加载 `devel/setup.bash` 并执行任务脚本。这个等待没有固定时限，首次编译较慢时可在另一终端查看日志，或按 Ctrl+C 结束。

### 5.2 容器：start.sh

1. 创建 `docker/logs`，加载 `/opt/ros/noetic/setup.bash`。
2. 若缺少 `devel/.hiarm-build-ok`，调用 `build.sh` 编译；已有标记则跳过。
3. 加载 `devel/setup.bash`，使本项目 ROS 包和插件可被找到。
4. 清理失效的 X11 锁/套接字，启动 `Xvfb :1`，检查 OpenGL 初始化。
5. 如果启用桌面，启动窗口管理器、任务栏、VNC 和网页代理。
6. 执行 `roslaunch docker/sim.launch`：启动 Gazebo、生成机器人描述、生成模型、启动模拟飞控、飞行控制器及 `real` 规划器。
7. 轮询 `/use_sim_time` 参数是否可读取，再轮询机器人描述参数，然后启动可视化 TF 桥接；启用桌面时打开 RViz。任务脚本会另外确认 `/use_sim_time=true` 和传感器数据。
8. 启动 `virtual_rc.py` 和 `web_server.py`，网页开始接收实时场景数据。
9. 脚本等待 roslaunch 或网页服务退出；收到退出信号时清理相关子进程。

`build.sh` 删除旧成功标记，初始化配置卷中的 profiles 目录，然后直接编译 `/ws/src` 中的共享源码：

```bash
catkin config --init --extend /opt/ros/noetic \
  --source-space /ws/src --build-space /ws/build --devel-space /ws/devel \
  --log-space /ws/build/logs --merge-devel --no-install \
  -j2 -l2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build -j2 -p1 -l2 --no-status --summarize
python3 docker/merge-compile-commands.py
```

`catkin build` 为各包分别配置和编译；`--merge-devel` 将产物汇集到 `/ws/devel`，现有 `devel/setup.bash` 和插件路径继续使用。`-j2` 限制最多两个编译作业，`-p1` 每次构建一个包，`-l2` 限制调度新作业时的负载。构建成功后才重新生成标记；成功标记不会自动检测源码变化。

构建配置保存在 `/ws/.catkin_tools`，分包日志保存在 `/ws/build/logs`。共享源码中的 `._*.msg` 是 macOS 元数据，会导致消息生成器报编码错误，已从消息目录移除；`.gitignore` 排除新增的 `._*` 文件。进入容器后可以运行 `catkin build -j2 -p1 -l2`，复用已保存的路径和 CMake 配置；从主机执行 `./docker/sim.sh build` 则统一应用上述配置并更新成功标记。

### 5.3 点击任务按钮后的流程

1. 网页通过本机同源接口启动 `mission.py`；终端 `mission` 则直接调用它。
2. 脚本等待仿真时钟和节点就绪，确认加载的是十阶段配置，并检查是否从初始状态开始。
3. 虚拟遥控切到 `hover`，等待机器人在 `(0,0,1)` m 附近稳定悬停。
4. 切到 `trajectory`，由原 `real` 规划器顺序执行十阶段任务。
5. 规划器报告人工变形等待时，脚本发送通道 7 上升沿，代替拨动遥控开关。
6. 记录规划器事件、命名物体连接/释放事件、物体位置和机体运动。
7. 结束后分别评价阶段完成、四次搬运以及易拉罐抓取稳定性。低空坠落、规划失败或超时会中止本轮记录。

飞行控制由项目的 `GeometricController` 计算，Gazebo 旋翼插件施加力和力矩；`fake_mavros` 提供模拟飞控状态与服务。该流程不启动 PX4 SITL。

## 6. Compose 配置和数据保存

| 配置 | 当前值 | 含义 |
|---|---|---|
| 项目/服务 | `hiarm` / `sim` | Compose 项目名与服务名 |
| 容器/镜像 | `hiarm-sim` / `hiarm:noetic-arm64` | `docker exec` 操作的容器及构建镜像 |
| `platform` | `linux/arm64` | 使用 ARM64 Linux 环境 |
| `build.context` | `.`，相对 compose 文件目录 | 使用 `docker` 目录构建镜像，Dockerfile 不把整个源码复制进镜像 |
| `init` | `true` | 使用容器 init 处理子进程回收和信号 |
| `shm_size` | `1gb` | 容器 `/dev/shm` 上限，不是 Docker 总内存配置 |
| `working_dir` | `/ws` | 容器中的工作区根目录 |
| `command` | `bash docker/start.sh` | 容器实际启动入口 |

### 6.1 端口

| 宿主机地址 | 容器端口 | 服务 |
|---|---|---|
| `127.0.0.1:8080` | 8080 | WebGL 静态资源、实时状态和固定的控制接口 |
| `127.0.0.1:6080` | 6080 | websockify/noVNC，仅启用桌面时有服务 |
| 不对外映射 | 5900 | x11vnc，仅供容器内 websockify 连接 |

HTTP 控制接口只提供任务启动、暂停和恢复等固定操作，并检查本机同源 JSON 请求；没有任意 shell 或 ROS 发布接口。noVNC 当前无密码，端口只绑定本机，不应直接改成公开网络监听。

### 6.2 挂载关系

| 挂载 | 用途 |
|---|---|
| 仓库根目录 → `/ws`（可写） | 源码、脚本、配置、网页和日志与主机共享；容器 `/ws/src` 直接对应主机 `src` |
| 命名卷 `catkin_build` → `/ws/build` | 各包独立的构建缓存及 `logs/` 编译日志 |
| 命名卷 `catkin_devel` → `/ws/devel` | 合并的编译产物、环境设置及构建成功标记 |
| 命名卷 `catkin_config` → `/ws/.catkin_tools` | catkin_tools 工作区与 profile 配置 |
| 命名卷 `vscode_server` → `/root/.vscode-server` | VS Code Server、容器端插件及其用户数据 |
| 命名卷 `codex_data` → `/root/.codex` | Codex CLI、账号登录状态、配置及会话 |

主机和容器直接编辑同一份文件，不再需要复制源码。容器内对 `/ws/src` 的修改或删除也会立即作用于主机仓库，可在主机使用 Git 查看差异。编译产物和工具配置保存在上述命名卷内，避免写入主机源码目录。

从旧配置迁移时，先导出旧容器的 `/ws/src` 并合并其中独有的修改，再重新创建容器。旧的 `hiarm_source` 卷不再被挂载。迁移到 `catkin build` 后使用新的 `hiarm_catkin_build`、`hiarm_catkin_devel` 和 `hiarm_catkin_config` 卷；旧 `hiarm_build`、`hiarm_devel` 保留，不与新工具混用缓存。

`stop` 和任务重新创建容器不会清空命名卷。日志在宿主机 `docker/logs` 中，但固定文件名可能被下一轮覆盖；要保留某轮结果，请先复制到单独目录。不要把删除卷当作普通重启步骤。

### 6.3 使用 VS Code 在容器中开发

1. 在主机 VS Code 中安装 **Dev Containers**（`ms-vscode-remote.remote-containers`）。
2. 执行 `./docker/sim.sh start` 启动容器。
3. 打开 VS Code 命令面板，执行 **Dev Containers: Attach to Running Container...**，选择 `hiarm-sim`，打开 `/ws`。
4. 在扩展面板中，将需要在容器中运行的 Python、C/C++ 等插件安装到容器。主题和图标等界面插件继续使用本机安装。

容器终端中编译：

```bash
cd /ws
bash docker/build.sh
source devel/setup.bash
```

容器端插件安装在 `hiarm_vscode_server` 命名卷中。`stop`、`demo`、`mission` 重新创建容器后会保留插件；重新连接容器即可继续开发。删除该卷会清除容器端 VS Code 数据。Mac 插件目录不直接挂载到容器，插件由 VS Code 安装适用于 Linux ARM64 的版本。

本项目使用的开发插件：

| 插件 ID | 用途 |
|---|---|
| `ms-vscode.cpptools` | C++ 控制器、Gazebo 插件的补全、跳转和调试界面 |
| `ms-python.python`、`ms-python.vscode-pylance` | Python 解释器选择、补全和类型分析 |
| `ms-python.debugpy@2024.2.0` | 支持容器 Python 3.8 的断点调试；保留此版本 |
| `Ranch-Hand-Robotics.rde-ros-1` | ROS 1 工作区、消息文件、节点和 launch 开发 |
| `redhat.vscode-xml` | launch、Xacro、world、package.xml 编辑 |
| `redhat.vscode-yaml` | Compose、控制参数和任务计划编辑 |
| `twxs.cmake` | CMakeLists.txt 语法和补全；编译使用 `docker/build.sh` |
| `timonwong.shellcheck` | Shell 脚本静态检查 |
| `deyuf.urdf-studio` | URDF/Xacro 模型预览 |
| `mhutchie.git-graph` | 查看分支和提交历史 |

Python 解释器选择 `/usr/bin/python3`，补全搜索路径包含 `/opt/ros/noetic/lib/python3/dist-packages` 和 `/ws/devel/lib/python3/dist-packages`。ROS 插件的 `ros.rosSetupScript` 设为 `/ws/devel/setup.bash`，加载当前工作区。Python Debugger 的版本依据其[官方兼容表](https://github.com/microsoft/vscode-python-debugger#limited-support-for-deprecated-python-versions)选择。

#### C++ 代码跳转

`build.sh` 启用 `CMAKE_EXPORT_COMPILE_COMMANDS=ON`。各包生成 `/ws/build/<包名>/compile_commands.json`，构建后由 `merge-compile-commands.py` 合并为 `/ws/build/compile_commands.json`，包含实际使用的头文件路径、宏和编译选项。

在 **Preferences: Open Remote Settings (JSON)** 中配置：

```json
{
  "C_Cpp.default.compileCommands": ["/ws/build/compile_commands.json"],
  "C_Cpp.default.compilerPath": "/usr/bin/g++",
  "C_Cpp.default.intelliSenseMode": "linux-gcc-arm64"
}
```

在容器窗口中打开 `/ws`，对符号使用 F12 跳转定义。新增源文件或修改 CMake 后执行 `bash docker/build.sh` 更新数据库；单独执行 `catkin build` 后，再运行 `python3 docker/merge-compile-commands.py`。数据库保存在构建卷中，不提交到 Git。

没有加入 CMake 目标的 `.cpp`（例如控制器的 `test.cpp`），以及带有 `CATKIN_IGNORE` 的包，不会出现在编译数据库中。C/C++ 扩展对这些文件使用 `/ws/.vscode/c_cpp_properties.json` 中的后备配置。该文件是本地编辑器设置，不提交到 Git。

后备 `includePath` 应填写实际的头文件根目录，包括各包的 `include`、`devel/include`、`/opt/ros/noetic/include`、`/opt/ros/noetic/include/xmlrpcpp` 和 `/usr/include/eigen3`。第三方库路径可参照同类文件的编译数据库记录：执行器需要 `external/feetech_servo`，RViz 需要 Qt5 和 OGRE，Gazebo 需要 Gazebo、Ignition、SDFormat 等目录。不要只依赖 `/usr/include/**`。Qt 文件还需要 `compilerArgs: ["-fPIC"]` 和 `defines: ["QT_NO_KEYWORDS"]`；工作区后备语言标准使用 C++17。

容器镜像包含 `ros-noetic-vrpn` 开发依赖；已有容器可运行 `apt-get install -y --no-install-recommends ros-noetic-vrpn` 补齐 VRPN 头文件。

### 6.4 在容器中使用 Codex

容器终端安装 CLI（默认版本 `0.151.0`，可用脚本第一个参数指定版本）：

```bash
bash /ws/docker/install-codex.sh
codex --version
```

首次复用 Mac 当前账号，在 **Mac 终端**执行：

```bash
docker exec hiarm-sim install -d -m 700 /root/.codex
docker cp "$HOME/.codex/auth.json" hiarm-sim:/root/.codex/auth.json
docker exec hiarm-sim chmod 600 /root/.codex/auth.json
docker exec hiarm-sim codex login status
```

这是官方支持的[向容器复制登录缓存](https://learn.chatgpt.com/docs/auth#fallback-authenticate-locally-and-copy-your-auth-cache)方式。凭据保存在 `hiarm_codex_data` 卷中，不放进仓库或镜像。CLI 与容器端 Codex 扩展共享该登录状态；平时重建容器无需再次复制。

在容器终端进入项目后使用：

```bash
cd /ws
codex
```

VS Code 中将 **Codex**（`openai.chatgpt`）安装到容器，重新加载容器窗口后打开 Codex 面板。CLI 与 VS Code 插件分别更新：CLI 执行安装脚本，插件在扩展面板更新。

当前 Docker 默认配置会阻止 bubblewrap 创建命名空间，表现为能对话但执行命令时报 `No permissions to create a new namespace`。安装脚本首次创建 `/root/.codex/config.toml`，使用文件登录缓存并启用 Landlock 沙箱：

```toml
cli_auth_credentials_store = "file"

[features]
use_legacy_landlock = true
```

已有配置时，安装脚本保留原内容；需将上述设置合并进去。本次已在容器中设置并验证只读命令可执行、沙箱外写入被拒绝。该兼容选项已被 Codex 标记为弃用，升级后若移除此选项，需要重新检查沙箱兼容性。

当前验证通过的是 CLI `0.151.0`。扩展 `26.51007.21434` 自带的 `0.162.0-alpha.17.2` 后端可以登录和对话，但执行代码命令仍报 `filesystem-restricted execution requires bubblewrap to isolate app-server sockets`；Landlock 兼容设置不能解决这个新版后端问题。当前在 VS Code 的容器终端中运行 `codex` 进行代码操作。

### 6.5 使用 Mac 主机代理

Compose 默认将容器的 `HTTP_PROXY`、`HTTPS_PROXY` 及小写同名变量设为 `http://host.docker.internal:7897`。`host.docker.internal` 指向 Mac 主机；容器中的 `127.0.0.1` 指向容器自身。

`NO_PROXY`/`no_proxy` 排除 localhost、`127.0.0.1`、`::1`、`hiarm-sim`、`sim` 和 `host.docker.internal`。容器主机名固定为 `hiarm-sim`，使 ROS 发布的本地节点地址绕过代理。

更换代理端口或关闭代理后，用 `start` 重新创建服务使环境变量生效：

```bash
HIARM_PROXY_URL=http://host.docker.internal:7890 ./docker/sim.sh start
HIARM_PROXY_URL= ./docker/sim.sh start
```

这些临时赋值只影响本次命令；后续 `demo`/`mission` 重建容器时也要传入相同值。主机代理软件需要保持运行。

容器中检查外网和本地 ROS：

```bash
curl -I https://github.com
source /ws/devel/setup.bash
rosnode ping -c 1 /gazebo
```

Compose 的构建参数覆盖 Dockerfile 中的下载命令；`docker pull` 和 `FROM` 镜像拉取使用 Docker Desktop 自身的网络/代理设置。

## 7. WebGL、noVNC 与 RViz

### 7.1 两种显示方式

| 对比 | WebGL | noVNC |
|---|---|---|
| 传输内容 | 模型资源及实时姿态数据 | 容器桌面图像与输入事件 |
| 画面生成位置 | Mac 浏览器，支持本机 GPU | 容器内 Gazebo/RViz，当前为 CPU 软件渲染 |
| 显示对象 | 本项目定制网页场景 | 完整 Linux 桌面和原软件窗口 |
| 适用场景 | 演示、观察、执行集成任务 | 模型检查、ROS 数据调试、使用原软件菜单 |

两者共享同一次 Gazebo 仿真。关闭 GUI 可以减少软件渲染负载；WebGL 加速画面绘制，不会直接把 ODE 物理或控制器计算搬到 Mac GPU。

WebGL FPS、Gazebo 实时因子、ROS 仿真时间是不同指标。网页 60 FPS 不代表物理仿真每秒推进 1 秒。`graphics.log` 是容器 Mesa 信息，不是浏览器 GPU 信息；浏览器渲染器见网页“渲染与连接信息”。

### 7.2 WebGL 使用及范围

页面支持拖动旋转、滚轮缩放、右键平移、跟随飞行器、任务全景、启动任务和暂停/恢复。刷新网页不重置仿真；连接中断后会重连。

当前暂停按钮只暂停 Gazebo 物理，任务的墙钟超时仍继续计时：起飞悬停限时 240 秒，进入轨迹任务后限时 1200 秒。暂停耗尽剩余时间后，恢复物理不会恢复任务脚本，需要重启任务。网页夹爪数值是 0～5 的统一执行器坐标，当前界面的 `rad` 单位标注不准确。

机器人视觉来自 `robot_description` 经 Gazebo URDF→SDF 转换后的模型，固定关节合并后的名称与 `/gazebo/link_states` 对齐。场景模型取自当前 world。DAE/STL 网格、纹理和 Three.js 均由本地服务提供，无运行时 CDN。

查看器在服务启动时读取 `basic.world` 或 `matched.world` 的视觉模型，运行中更新已有连杆的位姿。修改世界模型后需重启服务重新加载。

### 7.3 noVNC 窗口与分辨率

- 底部任务栏显示已打开的窗口，点击可恢复最小化窗口。
- 左侧快捷入口打开或恢复 Gazebo、RViz，也可打开终端。
- `Alt+Tab` 切换窗口；桌面右键菜单可访问应用和窗口列表。
- 双击标题栏或点击最大化按钮，扩大应用窗口。
- noVNC 控制栏可全屏；链接中的 `resize=scale` 对桌面进行本地缩放。

当前 `start.sh` 的虚拟桌面分辨率为 **1920×1080、24 位色深**：

```bash
Xvfb :1 -screen 0 1920x1080x24 +extension GLX +render -noreset
```

修改该行后执行 `restart` 生效，仿真会重置。本地缩放只改变显示大小，提高 Xvfb 分辨率才增加桌面实际像素，也增加图形开销。RViz 的初始窗口大小另由 `hiarm.rviz` 的 `Window Geometry` 控制，当前为 1200×790。

### 7.4 RViz 配置

启用桌面时自动加载 [hiarm.rviz](hiarm.rviz)。

| 配置 | 当前值 | 作用 |
|---|---|---|
| Fixed Frame | `world` | 显示数据的公共参考坐标系 |
| Robot Description | `/flygripper/robot_description` | 读取机器人 URDF |
| TF Prefix | `flygripper` | 将 URDF 连杆映射到可视化 TF |
| RobotModel | 启用 | 显示机器人视觉模型 |
| Path | `/flygripper/reference` | 显示参考路径，产生轨迹后才有数据 |
| Odometry | `/flygripper/odom` | 显示实际位姿历史箭头 |
| 相机 Target Frame | `flygripper/odom` | 跟随机器人 |
| 相机 Distance | `2.5` m | 初始观察距离，可在 Views 或用滚轮修改 |
| Frame Rate | `30` | RViz 请求的显示频率，不保证实际达到 |

`rviz_bridge.py` 将 Gazebo 实际连杆世界位姿发布为约 30 Hz 的可视化 TF，并补齐被 Gazebo 合并的固定连杆。它不改变物理状态或飞行控制。RViz 不会自动显示 Gazebo 世界中的易拉罐和支撑柱，这些场景物体应在 Gazebo GUI 或 WebGL 中查看。

启用 `HIARM_DESKTOP=true` 后，在 Mac 终端手动打开或恢复配置好的窗口：

```bash
docker exec -d hiarm-sim bash /ws/docker/open-window.sh rviz
docker exec -d hiarm-sim bash /ws/docker/open-window.sh gazebo
```

已存在的窗口会被恢复，不重新加载配置；修改 `hiarm.rviz` 后需在 RViz 打开该配置或关闭窗口后重新启动。

## 8. 任务、场景与抓取参数

### 8.1 十阶段任务

| 代码阶段 | 网页编号 | 动作 |
|---|---|---|
| 0 | 01 | 飞向起点并展开 |
| 1 / 2 | 02 / 03 | 抓取易拉罐 / 搬运并释放 |
| 3 / 4 | 04 / 05 | 抓取球体 / 搬运并释放 |
| 5 / 6 | 06 / 07 | 抓取瓶子 / 搬运并释放 |
| 7 / 8 | 08 / 09 | 抓取方块 / 搬运并释放 |
| 9 | 10 | 最后收拢动作 |

当前场景包含四个抓取物体及支撑柱。网页最后一阶段沿用“垃圾箱”名称，该阶段执行到位与收拢动作，场景中没有垃圾桶模型。

### 8.2 规划字段

基础文件为 `src/flygripper_planner/flygripper_planner/params/real_sim.yaml`；以下字段由原 `real.cpp` 解释，仿真覆盖文件只覆盖部分值。

| 字段 | 含义 |
|---|---|
| `task_num` | 阶段总数；完整任务为 10，短演示为 1 |
| `task_N` | 从 0 编号的阶段配置 |
| `plan_type` | `0` POINT、`1` DOOR、`2` GRASP；枚举取自源码，各类型读取的附加参数不同 |
| `plan_point_num`、`plan_point_N` | POINT 航点数量及坐标；`x/y/z` 单位 m，`yaw` 单位 rad |
| `plan_terminal_vel` | 阶段终点速度，单位 m/s |
| `plan_max_vel` | 规划速度上限，m/s |
| `plan_max_acc` | 规划加速度上限，m/s² |
| `plan_max_dyaw` | 规划航向角速度上限，rad/s |
| `plan_tolerance` | 位置到位容差，单位 m；负值按轨迹末段在 `morph_duration` 内的位移自动计算，最小为 0.01 m |
| `morph_action` | `0` 不变形、`1` 展开、`2` 收拢 |
| `morph_trigger` | `0` 规划开始时触发、`1` 到达目标条件后触发 |
| `morph_auto` | true 自动触发；false 等待遥控触发，完整演示脚本负责发送该触发 |
| `morph_tolerance` | 执行器到位容差，使用执行器反馈坐标 |
| `morph_expand_pos` / `morph_shrink_pos` | 当前为 `0` / `5`，是执行器控制坐标，不是每个关节都旋转 5 rad |
| `morph_duration` | 当 `plan_tolerance` 为负值时，用于计算轨迹末段的提前变形距离；不直接规定夹爪动作时长 |

速度和加速度字段约束规划轨迹，实际运动由控制器和物理仿真决定。

`make_grasp_plan.py` 生成的主要覆盖如下：

| 设置 | 当前值/行为 | 用途 |
|---|---|---|
| 抓取前对准点 | `x -= 0.35*cos(yaw)`、`y -= 0.35*sin(yaw)`，z 不变 | 沿目标航向反向退 0.35 m，先对齐再进入夹持位置 |
| 抓取段速度/加速度 | 0.2 m/s、0.3 m/s² | 降低接近冲击 |
| 抓取段到位容差 | 0.008 m | 精确接近目标 |
| 搬运速度/加速度/航向速率 | 0.2 m/s、0.2 m/s²、0.2 rad/s | 平缓带载运动 |
| 搬运航向 | 保持取物时航向 | 避免带载大幅转向 |
| 易拉罐提起航点 | 原位置上方 0.10、0.20 m | 先竖直离开支撑，再横向搬运 |

十阶段终点 x/y/z 保留，附加航点、速度和搬运航向已变化。

短演示 `demo-plan.yaml` 的目标为 `(0.6,0,1)` m、yaw=0；速度 0.3 m/s、加速度 0.4 m/s²、航向速率 0.5 rad/s、规划容差 0.08 m、终点速度为零，不执行抓取。`verify.py` 的运行验收另要求位置误差小于 0.15 m、速度小于 0.25 m/s，并维持约 3 秒仿真时间；规划容差与验收容差不是同一参数。

### 8.3 坐标匹配和碰撞模型

`match_scene.py` 根据脚本内 `OBJECTS` 表中的目标里程计位置、航向、机体系夹持偏移和模型几何中心，生成 `matched.world` 与 `scene-matching.json`。它不会自动读取 `real_sim.yaml` 的航点；修改取物终点时，需要同步更新 `OBJECTS` 表。

计算关系：世界抓取中心 = 目标里程计位置 + 按 yaw 旋转后的机体系偏移；模型放置原点还需要扣除模型内部几何中心偏移。不能直接把世界模型原点当成罐体中心。

| 物体 | 机体系夹持偏移 x/y/z（m） | 世界抓取中心 x/y/z（m，约） | 支撑柱宽度 |
|---|---|---|---|
| 易拉罐 | 0.065 / -0.065 / 0.06 | 7.8914 / 0.1703 / 1.0000 | 0.01 m |
| 球体 | 0.09 / -0.09 / -0.01 | 6.2801 / 0.2299 / 0.5000 | 0.18 m |
| 瓶子 | 0.09 / -0.09 / -0.01 | 6.0401 / -0.8301 / 0.9200 | 0.18 m |
| 方块 | 0.075 / -0.075 / -0.01 | 5.1998 / 2.1841 / 0.4400 | 0.18 m |

易拉罐保留视觉网格、质量和惯量，碰撞体改为半径 0.0335 m、高 0.1239 m 的圆柱。接触参数 `kp=100000`、`kd=100` 分别控制接触弹性/阻尼；`max_vel=0.1` 为接触穿透修正速度限制，`min_depth=0.001` 为接触穿透容忍深度。它们不是飞行控制器增益。

当前易拉罐高度和细支撑柱用于避开旋转部件碰撞体。修改抓取坐标时必须考虑整个闭合过程的空间间隙，不能只检查闭爪终态中心是否重合。

### 8.4 夹爪及连接规则

参数主要位于 `src/flygripper_description/urdf/flygripper_base.xacro`。

| 参数 | 当前值 | 作用 |
|---|---|---|
| `commandRateLimit` | 1.0 控制单位/s | 平滑执行器指令；0→5 的参考变化需约 5 秒，实际到位可能更慢 |
| `updateRate`（抓取插件） | 100 Hz | 处理抓取/释放判定的更新频率 |
| `graspForcesAngleTolerance` | 120° | 不同夹爪连杆对物体施加的接触力夹角至少为 120° |
| `graspCountThreshold` / `graspCountMax` | 2 / 4 | 选定接触对后累积有效判定，计数大于 2 时连接，计数上限为 4 |
| `disableCollisionsOnAttach` | true | 连接后暂时禁用被抓物体碰撞，释放时恢复 |
| `commandedRelease` | true | 张爪指令及实际张开后才允许按释放规则解除连接 |
| `releaseStartPosition` | 4.5 | 判断已经开始张开的执行器位置阈值 |
| `fullyOpenPosition` | 0.5 | 接近全开时允许解除连接，避免距离条件不敏感导致不释放 |
| `releaseDistanceTolerance` | 0.01 m | 原接触距离变化的释放判据之一，不等价于通用夹爪开口宽度 |
| `releaseToleranceMode` | `+` | 仅当接触点到物体连杆原点的距离比连接时增大超过阈值，才满足距离释放条件 |

执行器反馈使用映射到统一控制坐标后、与最终指令误差最大的关节。抓取判定排除同一连杆多个碰撞子部件的伪“双侧接触”，由 Gazebo 碰撞接触触发固定关节连接。

当前释放条件为：指令位置小于 0.1，且实际反馈位置小于 4.5，且（满足距离释放条件或反馈位置小于 0.5）。张爪指令有效期间不建立新的抓取连接。

连接后物体由固定约束保持，碰撞关闭，释放时恢复碰撞。该模型用于演示取放流程；夹持力、摩擦滑脱和投放落点不在报告判据中。

### 8.5 物理步长和计算负载

场景物理配置来自 `src/flygripper_sim/flygripper_gazebo/worlds/basic.world`，`match_scene.py` 生成匹配世界时保留这些设置：

| 字段 | 当前值 | 作用 |
|---|---|---|
| `physics type` | `ode` | 使用 ODE 物理引擎 |
| `solver/type` | `quick` | 使用该引擎的 quick 求解器 |
| `solver/iters` | `1000` | 求解器迭代次数设置，影响计算量及约束求解 |
| `max_step_size` | `0.002` s | 每个物理步最多推进 2 ms 仿真时间 |
| `real_time_update_rate` | `500` Hz | 目标物理更新频率，实际受 CPU 负载影响 |
| `real_time_factor` | `1` | 目标为一秒墙上时间推进一秒仿真时间，不保证始终达到 |
| `gravity` | `0 0 -9.7936` m/s² | 世界坐标中的重力加速度 |

这些字段写在 world 文件中，不是 `HIARM_*` 环境变量。提高显示帧率不会改变物理步长；缩短步长也不保证解决碰撞干涉。修改基础世界后，需要重新生成 `matched.world` 并重启才能影响完整任务。

## 9. 修改文件后如何生效

| 修改内容 | 生效方式 |
|---|---|
| `compose.yaml` 环境、端口、挂载 | 用 `start` 应用，按需带上显示开关 |
| `Dockerfile` 依赖 | `start` 重建镜像并应用；单独 `restart` 不换镜像 |
| `docker/start.sh`、桌面菜单/任务栏、启动中的 Python 服务 | 通常 `restart`；会重置仿真 |
| `web/app.js`、页面样式 | 刷新网页；若仍用缓存可强制刷新 |
| `docker/hiarm.rviz` | RViz 内重新加载配置，或关闭 RViz 后从快捷入口打开 |
| `docker/sim.launch`、现成 YAML/world | 重启仿真后重新加载 |
| 主机或容器内的 `src` C++ 文件 | 两端立即看到修改；执行 `build` 后重启，加载新编译产物 |
| `src` 中的 URDF、launch 或参数 | 重启仿真后重新加载；新增消息、修改 CMake 或包依赖时还需构建 |
| `match_scene.py` / `make_grasp_plan.py` | 先重新生成输出文件，再重启；启动脚本不会自动运行生成器 |

修改项目源码后的完整操作：

```bash
./docker/sim.sh stop
./docker/sim.sh build
HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh start
```

停止运行后再编译，避免边运行边替换插件。源码变化不会自动清除构建标记；`sim.sh start` 内部的 `compose up --build` 只构建镜像，已有标记时仍跳过 catkin 编译。

重新生成场景和抓取计划（已有运行容器时）：

```bash
python3 docker/match_scene.py
docker exec hiarm-sim bash -c 'source devel/setup.bash && python3 docker/make_grasp_plan.py'
HIARM_DESKTOP=true HIARM_GUI=true ./docker/sim.sh mission
```

`match_scene.py` 可在 Mac 用标准库 Python 运行；`make_grasp_plan.py` 使用容器内的 PyYAML。不要只手改生成文件而忘记生成器，下次生成会覆盖它。C++ 修改仍需重新编译。

## 10. 报告、日志与故障排查

### 10.1 结果字段

| `mission.json` 字段 | 含义 |
|---|---|
| `state`、`stage`、`completed` | 运行状态、当前阶段、已完成阶段列表 |
| `passed` | 十阶段规划器报告全部完成；不单独代表抓取成功 |
| `physical_grasp_verified` | 至少出现过一次连接事件 |
| `all_transfers_verified` | 四个命名物体的连接、搬运、计划释放全部通过 |
| `grasp_validation` | 逐物体连接/释放时间、搬运距离及评价 |
| `can_motion_validation.stable` | 易拉罐指定抓取时间窗的稳定性判据通过 |
| `grasp_and_stability_verified` | 四次搬运与易拉罐稳定性同时通过 |
| `object_samples` | 约 10 Hz 的物体及机器人位置记录 |
| `grasp_events` / `grasp_plugin_events` | ROS 布尔事件 / 带物体名称的插件日志 |
| `support_contacts`、`can_contacts` | 支撑柱及易拉罐相关接触诊断 |
| `input_sha256` | 本轮世界、任务、机器人描述和插件摘要，用于区分配置版本 |
| `sim_time` | 仿真秒，不等于墙上时间 |

`/flygripper/grasp_status` 是事件主题，不是连续心跳。未收到事件时，不能把关爪动作当成抓住了物体。

四次搬运的逐物体判据：名称日志和 ROS 事件时间对应；在计划张爪前连接并保持；连接期间物体首末位置的直线距离超过 0.15 m、采样中与机器人距离均小于 0.4 m；释放位于张爪开始到张爪完成后 0.2 s 的窗口。接触去抖允许连接发生在取物阶段或紧随其后的搬运阶段。

易拉罐稳定性窗口为开始闭爪至搬运阶段开始规划后 6 秒，要求数值有限、最大倾角 ≤10°、采样角速度模峰值 ≤1 rad/s，且没有记录到支撑柱或易拉罐与旋转部件碰撞。

当前 `complete` 只检查记录的最后时间是否到达窗口终点，尚未检查窗口首尾覆盖和采样间隔。因此缺失中间采样时仍可能得到 `stable=true`。这些指标按约 50 Hz 的现有采样计算。

独立重算已有记录：

```bash
python3 docker/grasp_report.py docker/logs/mission.json
python3 docker/shake_report.py docker/logs/mission.json docker/logs/motion.jsonl
```

### 10.2 日志位置

| 文件/目录 | 用途 |
|---|---|
| `logs/simulation.log` | Gazebo、飞控、规划器和接触诊断 |
| `logs/mission.json`、`logs/motion.jsonl` | 最新任务报告和约 50 Hz 机体运动 |
| `logs/mission.log` | 网页启动的任务输出；终端启动任务主要输出到该终端 |
| `logs/verification.json` | 短演示结果 |
| `logs/catkin-build.log` | 首次自动编译日志；手动 build 的输出默认在终端 |
| `logs/web.log`、`logs/virtual-rc.log` | 网页服务及虚拟遥控 |
| `logs/rviz.log`、`logs/rviz-bridge.log` | RViz 和 TF 桥接 |
| `logs/xvfb.log`、`logs/graphics.log` | 虚拟显示和容器 OpenGL |
| `logs/novnc.log`、`logs/vnc.log` | noVNC 代理、VNC 服务 |
| `logs/openbox.log`、`logs/panel.log` | 窗口管理器、任务栏 |
| `logs/grasp-tuning/`、`logs/shake-diagnosis/` | 历史抓取调整和稳定性记录 |

```bash
./docker/sim.sh logs
tail -n 100 docker/logs/simulation.log
./docker/sim.sh shell
```

Compose 日志主要是容器启动输出。ROS/Gazebo 的详细输出已重定向到 `simulation.log`，不能只看 `sim.sh logs` 就判断节点没有错误。

### 10.3 常见问题

| 现象 | 原因与处理 |
|---|---|
| 一直显示 Waiting for workspace build | 任务正在等待构建成功标记；另开终端运行 `./docker/sim.sh logs` 查看编译进度。容器停止时等待会退出 |
| demo 提示 ROS master 尚未启动或仿真时钟未启用 | 验收脚本会在初始化 ROS 节点前等待 `/use_sim_time=true`，最多 180 秒；超时则检查 simulation.log 和容器编译/启动日志 |
| 容器里运行 sim.sh 提示 docker 找不到 | 回到 Mac 终端执行；容器内无需运行 Docker CLI |
| 开了 noVNC 仍看到 WebGL | 打开 6080 的 `vnc.html`，8080 不会变成桌面 |
| noVNC 连接失败 | 确认当次创建容器时 `HIARM_DESKTOP=true`；查看 novnc/vnc 日志，等待初始化 |
| 最小化后找不到窗口 | 点击底部任务栏，或 Alt+Tab、桌面右键窗口列表 |
| 桌面小、窗口小 | 最大化应用窗口，noVNC 本地缩放/全屏；必要时修改 Xvfb 分辨率后重启 |
| RViz 空白 | 从快捷入口加载 hiarm.rviz；检查 bridge 日志、Fixed Frame、模型参数及 TF 前缀 |
| RViz 没有路径 | 初始待机尚未生成参考路径，启动任务后才可能显示 |
| RViz 看不到罐子和支撑柱 | 当前 RViz 只配置机器人和 ROS 轨迹；完整世界请用 Gazebo/WebGL |
| 修改源码后运行效果无变化 | 文件已共享；C++ 需构建后重启，模型/参数需重启加载。旧容器需用 start 重新创建以应用新挂载 |
| 任务完成后不能再次启动 | 先 restart；start 在配置未变时不保证清空已有仿真 |
| 暂停后恢复，任务却显示超时 | 暂停期间任务超时仍按墙钟计时；用 mission 重启整轮任务 |
| mission 命令成功退出，但没有完成四次抓取 | 当前退出码没有纳入抓取/稳定性评价；检查 mission.json 中的 all_transfers_verified 和 grasp_and_stability_verified |
| 修改开关后 restart 无效 | restart 沿用旧容器环境；改用 start 应用配置 |
| 任务变慢但网页仍流畅 | 网页帧率与物理实时因子不同；关闭 Gazebo GUI/RViz 降低 CPU 负担 |
| 抓取时抖动或漏抓 | 保存本轮报告/运动/接触日志，检查旋转部件间隙、坐标和连接事件，不能仅凭阶段完成判断成功 |

## 11. 历史结果和适用边界

以下为 2026-10-07 至 2026-10-08 保存的运行记录。日志和截图由 `.gitignore` 排除，新克隆需要自行运行任务生成结果。2026-10-10 新增的工作区编译等待逻辑尚未运行仿真验证。

### 11.1 ARM64 初始短演示

2026-10-07，MacBook Air M5 / 16 GB，Ubuntu 20.04、ROS Noetic、Gazebo 11.15.1、Mesa llvmpipe 环境：完整工作区编译及短演示通过，重启后再次通过。目标 `(0.6,0,1)` m，末位置约 `(0.599559,0.000114,1.000325)` m，误差约 0.560 mm；这是无噪声仿真结果，不代表实机精度。开启 GUI 的一次 5 秒采样实时因子约 0.874，不是固定性能保证。

保留的兼容性修复包括：替换损坏的 XSym 顶层 CMake、排除 AppleDouble 和旧构建缓存、补齐消息生成依赖、移除 RViz 插件未使用依赖、先读取旋翼时间常数再构造滤波器、处理虚拟显示残留锁。该阶段只验证短飞行，没有验证完整抓取。

历史证据：`logs/rebuild-filter.log`、`logs/default-controller-state.json`、`logs/simulation.jpg`；`logs/verification.json` 为可被后续运行覆盖的最新短演示报告。

### 11.2 WebGL 接入及早期任务

2026-10-08 的浏览器检查曾记录 Apple M5 Metal 渲染器、约 60.8 FPS、17 个视觉部件和 18 个连杆姿态，模型加载错误为零；这些是当时场景与采样快照。桌面和 390×844 窄屏布局曾通过检查。

早期未经匹配的 basic.world 中，十阶段动作完成但没有抓取事件；坐标匹配后的早期运行又出现球体搬运成功、瓶子提前释放、易拉罐和方块无连接。因此“10/10 阶段完成”从未被当作“4/4 抓取成功”。这些失败已由后续调整替代，不应误读成当前配置结论。

历史证据：`logs/original-mission-webgl.json`、`logs/webgl-browser.json`、`logs/webgl-api-check.json`、`logs/matched-attempt-1.json`、`logs/matched-mission.json`。暂停/恢复、同源限制（403）及重复启动拒绝（409）曾实际检查。网页截图位于仓库 `output/playwright/`。

易拉罐不可见曾由 DAE 的 `transparency=0` 被解释成全透明导致。`web/app.js` 仅对该罐体网格设置不透明并保留贴图；修复后确认罐体、红色纹理可见。证据：`output/playwright/coke-can-visible.png`。这属于可视化修复，不改变物理碰撞。

### 11.3 早期四物体搬运通过

接近轨迹、夹爪限速、全关节反馈、双侧接触和释放逻辑调整后，两轮达到 10/10 阶段、4/4 搬运，但当时尚未加入抖动检查。第二轮记录如下：

| 物体 | 连接时刻（仿真秒） | 释放时刻 | 连接期间位移 |
|---|---:|---:|---:|
| 易拉罐 | 49.916 | 67.668 | 2.552 m |
| 球体 | 87.596 | 100.436 | 1.142 m |
| 瓶子 | 134.288 | 154.728 | 1.868 m |
| 方块 | 175.394 | 200.692 | 2.339 m |

该轮总计约 213.24 仿真秒。证据：`logs/grasp-tuning/success-1.json`、`success-2.json`。不能用这两轮的搬运通过证明没有抖动。

### 11.4 易拉罐抖动原因及修复

用户失败记录中没有易拉罐连接事件，不能直接归因于成功挂载后的载荷增加。无罐对照的闭爪角速度峰值约 0.153 rad/s，而有罐时约 2.5 rad/s。

扩展接触诊断后，记录到罐体与 `left_up_rotor_link`、`right_up_rotor_link` 反复碰撞；仅抬高罐体后又记录到旋转部件与支撑柱碰撞。这里指此前为演示调整的匹配场景，不能据此认定原仓库或真机也有相同干涉。

最终将易拉罐中心高度偏移从 -0.01 m 调至 +0.06 m、支撑柱宽度改为 0.01 m，并增加竖直提起航点。旋翼碰撞和抓取前物体碰撞没有关闭；飞控增益、夹爪力限及 2 ms 物理步长保持原值。降低驱动力、调整接触参数或缩短步长等未通过的实验没有作为最终配置保留。

| 运行 | 抓取窗最大倾角 | 角速度峰值 | 四次搬运 | 稳定性 |
|---|---:|---:|---|---|
| 修复前复现 | 6.18° | 2.78 rad/s | 非该轮验收结论 | 未通过 |
| 修复后第 1 轮 | 2.53° | 0.696 rad/s | 4/4 | 通过 |
| 第 2 轮 | 2.61° | 0.791 rad/s | 4/4 | 通过 |
| 第 3 轮 | 2.29° | 0.612 rad/s | 4/4 | 通过 |

三轮均为干净启动，完成 10/10 阶段，抓取窗没有记录到旋翼/支撑柱碰撞。第 2 轮清理未启用实验代码，第 3 轮对旋翼碰撞单独限频记录，避免被普通夹爪接触日志遮住。

证据位于 `logs/shake-diagnosis/`：`user-run.json`、`rotor-contact-simulation.log`、`raised-can-simulation.log`、`success-1.json` 至 `success-3.json`、对应 `success-N-motion.jsonl`，以及 `result.json`。碰撞相位和位置误差可解释偶发程度不同，但各因素贡献未单独量化。

### 11.5 桌面与 RViz

补充 tint2 任务栏、快捷入口及右键菜单后，最小化窗口可以找回。增加可视化 TF 桥接、开启 RobotModel 并加载 hiarm.rviz 后，实际观察到机器人且 RViz `Global Status: Ok`。历史截图：`logs/desktop-configured.png`、`logs/rviz-configured.png`。

以上结果均为仿真记录，不证明真实硬件飞行、夹持力、滑脱、接收人递交或垃圾桶投放成功。

## 12. 文件索引与第三方资源

| 文件 | 职责 |
|---|---|
| `Dockerfile` | 系统/ROS/图形依赖和基础环境 |
| `compose.yaml` | 服务、开关、端口、卷与启动命令 |
| `sim.sh` | Mac 端管理入口 |
| `start.sh` | 容器服务编排、显示初始化、子进程清理 |
| `build.sh` | 配置 catkin_tools、用 catkin build 编译共享源码并更新成功标记 |
| `sim.launch` | Gazebo、机器人、模拟飞控、控制器和规划器配置 |
| `virtual_rc.py` | 仿真遥控输入 |
| `mission.py`、`verify.py` | 完整任务执行/记录、短演示执行/记录 |
| `grasp_report.py`、`shake_report.py` | 对已有记录评价搬运与稳定性 |
| `match_scene.py`、`matched.world`、`scene-matching.json` | 场景生成器、输出世界和坐标清单 |
| `make_grasp_plan.py`、`grasp-plan.yaml`、`demo-plan.yaml` | 完整任务覆盖生成器及两种任务覆盖参数 |
| `web_server.py`、`web/` | WebGL 服务、页面和本地模型加载逻辑 |
| `rviz_bridge.py`、`hiarm.rviz` | 可视化 TF 与 RViz 默认显示 |
| `open-window.sh`、`desktop/` | 窗口恢复/启动、任务栏、菜单与应用入口 |
| `logs/` | 运行时证据和日志，不作为源码配置 |

### Three.js 来源与许可

`web/vendor/` 保存 Three.js **0.160.1** 的所需子集，采用 MIT 许可。完整许可文件保留在 [web/vendor/LICENSE](web/vendor/LICENSE)，整理 Markdown 不会删除第三方许可。

原归档地址：`https://registry.npmjs.org/three/-/three-0.160.1.tgz`

归档 SHA-256：

```text
d462f9bb4a53d0a75f5ebba49ec1c69847f166cd1bd6cbfdd98ce0b5b7988609
```

以上来源和摘要迁自原 vendor 说明；本次文档整理没有重新下载归档校验。运行时不访问 CDN。
