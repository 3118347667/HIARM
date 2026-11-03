# PREPARE snapshots the powered-flight throttle before the grasp approach.
# START is accepted only for the same prepared mission; the mandatory gradual
# support test precedes zero thrust. Servo feedback is optional and explicitly
# disabled for hardware whose feedback is known to be unreliable.
std_msgs/Header header
uint8 PREPARE=0
uint8 START=1
uint8 CANCEL=2
uint64 mission_id
uint8 command
float64 closed_position
float64 open_position
float64 closed_tolerance
float64 open_tolerance
float64 motors_off_duration
float64 spinup_duration
float64 recovery_duration
float64 retreat_distance
float64 retreat_duration
float64 settle_duration
float64 max_settle_speed
float64 position_tolerance
float64 timeout
bool use_actuator_feedback
float64 gripper_open_duration
float64 unload_duration
float64 support_verify_duration
float64 final_unload_duration
float64 test_throttle_ratio
float64 max_height_drop
float64 max_downward_speed
float64 min_vertical_acceleration
float64 sensor_max_age
