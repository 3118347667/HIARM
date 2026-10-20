# dynamic_grasp 真机工作区

此分支于 2026-10-19 从本地 `dynamic_grasp` 工作区导入源码快照，作为与 HIARM 仿真分支无共同提交历史的独立分支。源目录没有 Git 元数据，因此不包含此前的提交历史。

- `src/`：ROS/catkin 软件包。
- `scripts/`：运行与维护脚本。
- `bags/`：录制脚本和 XML 配置；录制生成的 `.bag` 数据未导入。

构建产物、日志和 macOS 元数据未导入。`src/CMakeLists.txt` 保留了源工作区的符号链接，目标为 `/opt/ros/one/share/catkin/cmake/toplevel.cmake`；构建时需要对应的 ROS 环境。

Radxa Zero 3W 的 Docker 编译与板端工作区上传说明见 [docker/ros-one-jammy/README.md](docker/ros-one-jammy/README.md)。当前分支的 17 个选定包已在本机 ARM64 容器中编译成功；真机运行效果仍需验证。
