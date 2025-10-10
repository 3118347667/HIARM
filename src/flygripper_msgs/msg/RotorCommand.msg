std_msgs/Header header

# Control mode


uint8 MODE_ATTITUDE=0
uint8 MODE_BODY_RATE=1
uint8 MODE_SINGLE_ROTOR=2
uint8 MODE_BODY_RATE_CUSTOMIZED=3
uint8 mode

# Collective mass-normalized thrust [m/s^2]
float64 collective_thrust

# Attitude in body frame [quaternion]
geometry_msgs/Quaternion attitude

# Angular rates in body frame [rad/s]
geometry_msgs/Vector3 bodyrates

# Single rotor velocities [rad/s]
float64[] rotor_velocities