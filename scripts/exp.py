import math
# x= 5.82
# y= 0.143
# yaw = 0.0
#
# arc_radius = 0.39
# arc_angle = 1.57
#
# ori_x = x - arc_radius * math.sin(yaw)
# ori_y = y + arc_radius * math.cos(yaw)
# x2 = ori_x + arc_radius * math.sin(arc_angle)
# y2 = ori_y - arc_radius * math.cos(arc_angle)
# yaw2 = yaw + arc_angle
# print(x2, y2)

x2 = 6.404
y2 = 0.3676
yaw2 = 1.681
distance=0.05
x3 = x2 - distance*math.cos(yaw2)
y3 = y2 - distance*math.sin(yaw2)
print(x3, y3)
