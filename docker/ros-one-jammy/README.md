# Radxa Zero 3W: ROS One ARM64 build

The board image discussed in `codex://threads/01a142fe-a527-7ae2-8549-b3ad609360e2`
uses Ubuntu 22.04 ARM64, ROS One at `/opt/ros/one`, and the active workspace
at `/home/rock/dynamic_grasp`. This Docker environment matches those paths.
It builds the 17 selected onboard packages with `catkin build`; simulation
sensing and RViz-only packages are outside that selection.

## Build on Mac

Install Docker Desktop. In the repository root, run:

```bash
./scripts/build_ros_one_arm64.sh
```

The Dockerfile builds the VRPN 07.35 client library from its pinned upstream
commit and submodules in the last filesystem layer. It installs the CMake find
modules needed by `vrpn_client_ros`. No Radxa disk image, extracted libraries,
or `7zz` is needed. The first build needs network access to GitHub and the
Ubuntu/ROS package repositories; later builds reuse the Docker layer cache.

Normal builds reuse the Docker image and `build_catkin/` cache. The resulting
ARM64 programs and catkin environment are in `devel/`. If dependencies change,
update the Dockerfile; `REFRESH_DEPS=1` can check/install them for one build,
but packages installed in a temporary container will not persist. The default
parallelism is two jobs; set `CATKIN_JOBS=4` if the host has enough memory.
The build script only builds locally. Upload separately with one of the
commands below.

If the container cannot reach the ROS One package server through the host's
proxy, use a host-accessible address:

```bash
CONTAINER_PROXY=http://host.docker.internal:7897 ./scripts/build_ros_one_arm64.sh
```

On Windows, use Docker Desktop Linux containers and run these Bash scripts in
WSL 2. An x86 Windows host may run the ARM64 build through emulation and take
longer.

## Upload to the board

Upload an existing build over ADB (one authorized device is selected
automatically):

```bash
./scripts/push_ros_one_arm64_adb.sh
```

If multiple ADB devices are connected, pass the serial as an argument or set
`BOARD_ADB_SERIAL`. The ADB path requires a root ADB shell, checks the archive
checksum, and extracts files as `rock` to preserve ownership and permissions.

Upload over SSH:

```bash
./scripts/push_ros_one_arm64.sh
```

The board IP is assigned dynamically. Set `BOARD_SSH_TARGET=rock@NEW_IP` or
pass `rock@NEW_IP` as an argument to override the default SSH target.

Both upload scripts check the local EKF architecture, then upload `src/`, `scripts/`,
`devel/`, and `params/` to the board's `/home/rock/dynamic_grasp` workspace.
The local `scripts/.wifi_credentials` file is excluded from both uploads.
They verify that the workspace exists before copying, and check four key
ARM64 files and missing dynamic libraries afterwards. They do not delete
board-only files or start hardware. Stop running ROS nodes before updating the
active workspace, then restart them after checking the transfer. Catkin's
generated setup files use this same `/home/rock/dynamic_grasp` path.

The Docker image is a development environment, not an exact copy of every
library on the board. Successful compilation and upload still require board
runtime verification.
