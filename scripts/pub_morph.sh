#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/devel/setup.bash" || exit 1
rostopic pub /flygripper/actuator_command flygripper_msgs/ActuatorCommand "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
mode: 0
reference_position: -12.0
reference_velocity: 0.0
reference_effort: 0.0"
