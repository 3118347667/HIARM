FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.ustc.edu.cn/ubuntu-ports|g' /etc/apt/sources.list \
    && python3 -c "from pathlib import Path; Path('/etc/apt/apt.conf.d/docker-clean').unlink(missing_ok=True)" \
    && printf 'Acquire::Retries "5";\n' > /etc/apt/apt.conf.d/80-retries
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && for attempt in 1 2 3 4 5; do apt-get install -y --no-install-recommends \
    build-essential python3-catkin-tools python3-numpy python3-scipy \
    ros-noetic-gazebo-ros-pkgs ros-noetic-rviz ros-noetic-xacro \
    ros-noetic-mavros-msgs ros-noetic-realtime-tools ros-noetic-joy \
    ros-noetic-tf ros-noetic-tf2-ros ros-noetic-robot-state-publisher \
    libeigen3-dev qtbase5-dev xvfb x11vnc novnc websockify openbox \
    xterm tint2 wmctrl x11-xserver-utils adwaita-icon-theme mesa-utils curl ca-certificates && break; \
    if [ "$attempt" = 5 ]; then exit 1; fi; sleep 2; done
ENV DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV PATH="/root/.codex/bin:${PATH}"

# 新增依赖放在后面，复用已有的 ROS/Gazebo 安装层。
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends zsh git fzf ros-noetic-vrpn

# Shell 插件单独一层，修改插件时无需重新安装 apt 软件包。
RUN git clone --depth=1 https://github.com/ohmyzsh/ohmyzsh.git /root/.oh-my-zsh \
    && git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions.git \
        /root/.oh-my-zsh/custom/plugins/zsh-autosuggestions \
    && git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git \
        /root/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting \
    && git clone --depth=1 https://github.com/zsh-users/zsh-completions.git \
        /root/.oh-my-zsh/custom/plugins/zsh-completions \
    && find /root/.oh-my-zsh -type d -name .git -prune -exec rm -rf '{}' + \
    && chsh -s /usr/bin/zsh root
COPY zshrc /root/.zshrc
ENV SHELL=/usr/bin/zsh

# 开发工具追加在已有安装层之后，复用 ROS/Gazebo 和 Shell 工具缓存。
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && printf '%s\n' 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros/ubuntu focal main' \
        > /etc/apt/sources.list.d/ros1-snapshots.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-rqt-graph ros-noetic-rqt-plot ros-noetic-rqt-console \
        ros-noetic-rqt-reconfigure ros-noetic-rqt-image-view ros-noetic-rqt-topic \
        gdb vim-tiny nano less ripgrep procps psmisc iproute2 iputils-ping

WORKDIR /ws
CMD ["bash"]
