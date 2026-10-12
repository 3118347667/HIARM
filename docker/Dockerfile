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
    ros-noetic-tf ros-noetic-tf2-ros ros-noetic-robot-state-publisher ros-noetic-vrpn \
    libeigen3-dev qtbase5-dev xvfb x11vnc novnc websockify openbox \
    xterm tint2 wmctrl x11-xserver-utils adwaita-icon-theme mesa-utils curl ca-certificates && break; \
    if [ "$attempt" = 5 ]; then exit 1; fi; sleep 2; done
ENV DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV PATH="/root/.codex/bin:${PATH}"
WORKDIR /ws
CMD ["bash"]
