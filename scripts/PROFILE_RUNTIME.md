# Runtime profiling

Build the workspace normally, then enable profiling before starting the real
moving-target grasping experiment:

```bash
export DYNAMIC_GRASP_PROFILE=1
```

Start the same VINS, perception, mapping, planning, and flight launch files used
for the paper experiments. The instrumented modules print cumulative statistics
every 50--100 samples, or every 1000 control samples, in this format:

```text
[PROFILE] module=trajectory_optimization samples=100 mean_ms=... p95_ms=... max_ms=... rate_hz=...
```

At the same time, sample process-level CPU utilization and resident memory:

```bash
source devel/setup.bash
python3 scripts/profile_system.py \
  --duration 180 \
  --interval 1 \
  --output bags/runtime_resources.csv
```

If the onboard kernel exposes NPU/GPU load as percentages in a sysfs or debugfs
file, pass that file to the same sampler:

```bash
python3 scripts/profile_system.py \
  --duration 180 \
  --accelerator-path /path/to/accelerator/load \
  --output bags/runtime_resources.csv
```

After the trial, convert the latest cumulative timing records from the ROS log
directory into a table-ready CSV. The script also calculates the timing margin
for the configured 30 Hz, 20 Hz, and 333 Hz loops:

```bash
python3 scripts/summarize_runtime.py \
  ~/.ros/log/latest \
  --output bags/runtime_timing.csv
```

Use at least three representative complete grasping trials after all modules
have reached steady state. Report the cumulative mean, P95, and maximum values
from the final trial log, and report process resources from
`runtime_resources.summary.json`. Mapping and planning share the
`/drone0/manager` process, so their CPU and memory must be reported together.

To measure the geometric controller without arming or spinning the vehicle,
load the same deployed parameter files and run the isolated benchmark. It calls
the production `prepare()` and `getCommand()` methods directly, does not start
the flight FSM, and publishes no actuator or MAVROS command:

```bash
roscore
# In another shell:
rosparam load src/flygripper_flight/params/flygripper.yaml /
rosparam load src/flygripper_controller/params/flygripper.yaml /
source devel/setup.bash
DYNAMIC_GRASP_PROFILE=1 rosrun flygripper_controller flygripper_controller_benchmark \
  --warmup 1000 --iterations 10000 --frequency 333
```

The benchmark reports `controller_prepare`,
`trajectory_tracking_control` (the same `getCommand()` branch instrumented in
the flight FSM), and `controller_total`. Run it with props secured and motors
physically prevented from spinning; disarming alone is not a substitute for
the normal hardware safety procedure.

Unset the environment variable to return to the normal no-profiling path:

```bash
unset DYNAMIC_GRASP_PROFILE
```
