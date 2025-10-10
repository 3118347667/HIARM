# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;tf;gazebo_ros;nav_msgs;sensor_msgs;flygripper_msgs;mavros_msgs;flygripper_common".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgazebo_rotor_plugin;-lgazebo_template_plugin;-lgazebo_odometry_plugin;-lgazebo_imu_plugin;-lgazebo_reconfigurable_plugin;-lgazebo_grasp_plugin;-lgazebo_interactive_plugin".split(';') if "-lgazebo_rotor_plugin;-lgazebo_template_plugin;-lgazebo_odometry_plugin;-lgazebo_imu_plugin;-lgazebo_reconfigurable_plugin;-lgazebo_grasp_plugin;-lgazebo_interactive_plugin" != "" else []
PROJECT_NAME = "flygripper_gazebo_plugins"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
