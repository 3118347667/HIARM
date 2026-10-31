# Published by the flight control loop, never latched. Active describes local
# trajectory sampling and command publication, not FCU receipt or physical motion.
uint8 FSM_MANUAL=0
uint8 FSM_GUIDED=1
uint8 FSM_TRAJ=2
uint8 FSM_WAIT_GUIDED=3
uint8 FSM_HANDOVER=4
uint8 REF_TAKEOFF=0
uint8 REF_HOVER=1
uint8 REF_VEL=2
uint8 REF_ADMITTANCE=3
uint8 REF_TRAJ=4

std_msgs/Header header
bool trajectory_active
bool fcu_state_fresh
bool fcu_armed
bool fcu_guided_nogps
uint8 fsm_state
uint8 reference_type
# Exact ID and start stamp of the trajectory consumed by this control loop.
uint32 trajectory_id
time trajectory_start
