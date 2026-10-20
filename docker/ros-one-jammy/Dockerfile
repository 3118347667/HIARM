FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl build-essential cmake git lsb-release rsync file \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN install -d /etc/apt/keyrings /etc/ros/rosdep/sources.list.d \
    && sed -i 's|http://ports.ubuntu.com|https://ports.ubuntu.com|g' /etc/apt/sources.list \
    && printf 'Acquire::Retries "5";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\n' \
       > /etc/apt/apt.conf.d/80-retries \
    && curl -fsSL https://ros.packages.techfak.net/gpg.key \
       -o /etc/apt/keyrings/ros-one-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net jammy main" \
       > /etc/apt/sources.list.d/ros-one.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep python3-catkin-tools \
    ros-one-ros-base ros-one-rviz ros-one-mavros-msgs \
    && echo 'yaml https://ros.packages.techfak.net/ros-one.yaml one' \
       > /etc/ros/rosdep/sources.list.d/1-ros-one.list \
    && rosdep init \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    qtbase5-dev ros-one-realtime-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=one
WORKDIR /home/rock/dynamic_grasp

# Keep VRPN in the last filesystem layer so changes to it reuse the ROS base.
# This is the VRPN 07.35 commit used for the board's client-only static build.
RUN set -eux; \
    apt-get update -o APT::Update::Error-Mode=any; \
    apt-get install -y --no-install-recommends libusb-1.0-0-dev; \
    git clone --depth 1 --branch version_07.35 https://github.com/vrpn/vrpn.git /tmp/vrpn-source; \
    test "$(git -C /tmp/vrpn-source rev-parse HEAD)" = 9493113de3ace0a8dd1de1e3ff44a4db4cebcdb3; \
    git -C /tmp/vrpn-source submodule update --init --recursive; \
    test "$(git -C /tmp/vrpn-source/submodules/hidapi rev-parse HEAD)" = 8b26e4962175a82205ebc499fdc2682e581e5fbe; \
    test "$(git -C /tmp/vrpn-source/submodules/jsoncpp rev-parse HEAD)" = 00b0a1b99216a3d220a25a78a496e7d0b0c4251c; \
    cmake -S /tmp/vrpn-source -B /tmp/vrpn-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DVRPN_BUILD_CLIENT_LIBRARY=ON \
      -DVRPN_BUILD_SERVER_LIBRARY=OFF \
      -DVRPN_BUILD_CLIENTS=OFF \
      -DVRPN_BUILD_SERVERS=OFF \
      -DVRPN_BUILD_PYTHON=OFF \
      -DVRPN_BUILD_PYTHON_HANDCODED_2X=OFF \
      -DVRPN_BUILD_PYTHON_HANDCODED_3X=OFF \
      -DVRPN_USE_HID=ON \
      -DVRPN_USE_LOCAL_HIDAPI=ON \
      -DVRPN_USE_LIBUSB_1_0=ON \
      -DVRPN_USE_LOCAL_JSONCPP=ON \
      -DVRPN_USE_JSONNET=ON \
      -DVRPN_USE_I2CDEV=ON; \
    cmake --build /tmp/vrpn-build --parallel 4; \
    cmake --install /tmp/vrpn-build; \
    install -d /usr/local/lib/cmake/VRPN; \
    install -m 644 /tmp/vrpn-source/cmake/FindVRPN.cmake \
      /tmp/vrpn-source/cmake/Findquatlib.cmake /usr/local/lib/cmake/VRPN/; \
    printf '%s\n' \
      'set(_robot_vrpn_module_path "${CMAKE_MODULE_PATH}")' \
      'list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")' \
      'include("${CMAKE_CURRENT_LIST_DIR}/FindVRPN.cmake")' \
      'set(CMAKE_MODULE_PATH "${_robot_vrpn_module_path}")' \
      'unset(_robot_vrpn_module_path)' \
      > /usr/local/lib/cmake/VRPN/VRPNConfig.cmake; \
    rm -rf /tmp/vrpn-source /tmp/vrpn-build /var/lib/apt/lists/*

CMD ["bash"]
