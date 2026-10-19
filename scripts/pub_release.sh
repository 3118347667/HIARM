#!/bin/bash

source devel/setup.bash
rostopic pub /flygripper/actuator_command flygripper_msgs/ActuatorCommand "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
mode: 0
reference_position: 0.0
reference_velocity: 0.0
reference_effort: 0.0"
