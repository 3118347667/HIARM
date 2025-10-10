
# Dependencies
```shell
sudo apt install ros-noetic-realtime-tools
```

## Simulation Workflow
1. `run.sh ${mav_name}` — Launch the simulation environment  
2. `ctrl.sh ${mav_name}` — Run localization, control, and remote control (supports joystick control for forward/backward, lateral, and vertical motion)  
3. `plan.sh ${mav_name}` — Run the planning algorithm (supports autonomous trajectory planning and deformation triggered by remote control)

---

# Introduction
1. `flygripper_bringup run.launch`
   - `flygripper_bringup urdf.launch`
       - `flygripper_description load_robot_urdf.launch` (loads robot URDF depending on `use_sim`; simulation includes Gazebo-related plugins)  
       - `flygripper_gazebo load_gazebo.launch` (loads world; simulation-dependent)  
       - `rviz` (optional visualization)  
   - `flygripper_bringup actuator.launch` (motor driver; real hardware only)  
2. `flygripper_estimator estimator.launch` (localization and state estimation — maps `imu`, `odom`, and `actuator_state` to `robot_state`)  
3. `fake_mavros apm.launch` (simulated MAVROS state and RC logic)  
4. `flygripper_flight flight.launch` (flight state machine and controller)  
5. `flygripper_planner/polyplanner` (TBD: trajectory planning and figure-eight tests; to be extended for grasping tasks)

---

## Topic Mapping
- imu:          `/mavros/imu/data_raw`  
- odom:         `/ekf/ekf_odom` (filtered by mocap and IMU via EKF)  
- rc:           `/mavros/rc/in`  
- fc_state:     `/mavros/state`  
- rotor_command:`/mavros/actuator_control`  
- rotor_speed:  `/mavros/esc_telemetry`

---

## Important Parameters
After changing the aircraft model:
1. In `flygripper_description`, define new Xacro files `${mav_name}.xacro` and `${mav_name}_base.xacro` — names must match the URDF model.  
2. In `flygripper_flight`, define new aircraft parameter file `${mav_name}.yaml`; update actuator limits in `fsm.hpp`.  
3. In `flygripper_common/type/actuator_state.hpp`, update kinematic mappings. (Later, implement interface to map `actuator_position` to `rotor_pos`.)  
4. In `flygripper_controller`, adjust controller parameters to fit the new aircraft.

---

# Scripts

## Simulation Workflow
1. `run.sh ${mav_name}` — Launch simulation environment  
2. `ctrl.sh ${mav_name}` — Run localization, control, and remote control  

---

## Xacro to URDF
```shell
rosrun xacro xacro src/flygripper_description/urdf/flygripper.xacro namespace:=flygripper use_sim:=true > flygripper.urdf
```

---

## Other Scripts
- `rename.sh` — Quickly rename files  
- `bag.sh` — Quickly record ROS bags
