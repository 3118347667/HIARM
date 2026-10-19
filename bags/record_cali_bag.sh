# rosbag record /mavros/imu/data /camera/color/image_raw /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw -O $1
rosbag record /camera/color/image_raw -O $1
