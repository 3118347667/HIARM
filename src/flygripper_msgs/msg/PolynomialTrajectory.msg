std_msgs/Header header

# the trajectory id, starts from "1".
uint32 id

# the polynomial pieces for x, y, z
flygripper_msgs/Polynomial[] xyz_pieces

# the polynomial pieces for heading
uint8 HAEDING_NONE = 0
uint8 HAEDING_FORWARD = 1
uint8 HEADING_BACKWARD = 2
uint8 HEADING_POLY = 3
uint8 heading_mode
flygripper_msgs/Polynomial[] heading_pieces