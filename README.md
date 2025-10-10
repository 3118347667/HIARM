
# Dependencies
```shell
sudo apt install ros-noetic-realtime-tools
```

## Simulation Workflow
1. `run.sh ${mav_name}` — Launch the simulation environment  
2. `ctrl.sh ${mav_name}` — Run localization, control, and remote control (supports joystick control for forward/backward, lateral, and vertical motion)  
3. `plan.sh ${mav_name}` — Run the planning algorithm (supports autonomous trajectory planning and deformation triggered by remote control)

---

# Current Latent Problems

- **Unresolved Simulation Issue**  
  Remote deformation operation — rapid position changes in simulation may cause sudden noise spikes, which do not occur in real experiments.

- **Unresolved Localization and Morphing Kinematics Mapping**  
  - Need to further consider how localization should be implemented (directly from distributed motion capture markers or via kinematics). Currently, it is tentatively integrated at the flight control point, similar to a simulator-provided odometry.  
  - Morphing kinematics visualization — temporarily static. The kinematic mapping (from encoder readings to joint and rotor transformations) remains to be implemented.

- **Unresolved RPM Control Mapping Identification**  
  - The current DShot-to-RPM mapping mechanism is not yet realizable in simulation. In real experiments, the RPM signal contains significant noise.  
  - The accuracy of this mapping still requires more extensive testing.

---

- **Preliminary Progress**: The state machine for real-world flight is basically completed. The estimator node has been removed — replaced with `ekf_pose`, with other modules merged into the flight node.  

- **Preliminary Progress**: Real-world tests verify the controller performance — model mismatch seems to cause more oscillation than sensor noise (simulation vibration issue does not occur in real flights).  

- **Preliminary Progress**: Yaw control strongly depends on the aircraft dynamics. INDI causes acceleration oscillation; L1 causes angular acceleration oscillation — both perform similarly in the simplified model.  
   - The original L1 lacks compensation for model mismatch — thrust direction changes with mismatch.  
   - The modified version adds compensation for mismatch — angular velocity oscillates, but acceleration remains stable; INDI angular velocity may be preferable.  
   - Simulation yaw control torque coefficient issue fixed; remaining yaw static error significantly affects tilt control, leading to low-frequency oscillations.  

- **Preliminary Progress**: Simulation vibration cause and grasping instability  
   - Future work can reference OpenAI hand simulation methods. For now, collision is simplified and a grasping attachment plugin is added.

- **Preliminary Progress**: Recompilation and build-time issues  
   - Common library converted to dynamic library — issue not fully resolved. Possible causes: dynamic reconfigure generation or large unseparated `.hpp` files. Modular separation of frequently modified code is in progress.

---

# TODO

## Common
- [ ] Timer malfunction ※  
- [x] Merge static and dynamic parameter config manager; the current API structure is too messy ※※※  

## Flight
- [ ] Add world-frame remote control; needs testing ※※※※※  
- [ ] DShot-RPM mapping: support multiple modes and online least-squares self-update; needs testing ※※※  
  - Gaussian regression lookup table with voltage compensation  
  - Function mapping with voltage compensation  
  - Quadratic mapping without voltage compensation  
- [ ] Implement thrust-attitude and thrust-angular-velocity control modes ... ※※※※※  

## Actuator
- [x] Support input from ROS messages and dynamic parameters  

## Simulator (Deferred)
- [ ] SW to URDF model causes IMU noise (especially acceleration ±1 m/s²), affecting INDI controller performance; questionable in real-world cases (deferred) ※※※  
    - ~~May be due to axis joint collision friction~~  
    - ~~Or heavy rotor inertia causing vibration~~  
    - ~~Or non-orthogonal exported joints causing low-frequency oscillation~~  
    - Caused by joint stiffness;  
      - **Test confirms correlation with URDF-defined joint friction and damping; increasing both reduces IMU noise to ±0.2 m/s²**  
      - **Also related to PD parameters in the simulated line-drive controller**  

- [ ] Gazebo plugin to visualize robot model TF (deferred) ※※※  
    - Prefer a unified robot visualizer suitable for both real and simulated systems.  

- [ ] Add noise model to evaluate controller robustness (deferred) ※  
  - IMU noise, ODOM/MOTOR model time delay  

- [x] Add and modify `gazebo_grasp_fix` plugin (solved grasping vibration issue)  
  Mitigation: simplified collision model and implemented grasping plugin  

- [x] Odom plugin TF duplicate data handling (minor ROS timing issue, now fixed)  
      Cause: identical TF timestamps between frames; fixed.  

## Controller
- [ ] INDI and L1 cannot be activated simultaneously — they are mutually exclusive ※  
- [x] Fixed Yaw control issue !!!  
      Cause: incorrect torque coefficient; yaw axis lacked compensation, thus relied solely on feedback — now resolved.  
- [x] L1 Adaptive Control  
      Integrated into geometric controller; can manually switch between feedforward and feedback linearization modes.  

## Planner
- [ ] Application test ※※※※※  
- [ ] Planning implementation based on Dr. Jia’s method ※※※  
- [x] Minimum Snap or MINCO reference (takeoff, velocity, etc.) in sampler (moderate performance) ※  

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
~~4. In `flygripper_estimator`, modify mapping between `rotor_pos` and aircraft center.~~ (Temporarily handled in flight control; TODO later.)  
5. In `flygripper_controller`, adjust controller parameters to fit the new aircraft.

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
