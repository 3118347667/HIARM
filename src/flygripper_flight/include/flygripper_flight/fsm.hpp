#pragma once
#include <algorithm>
#include <mutex>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/callback_queue.h>
#include "flygripper_flight/runtime_io.hpp"
#include "flygripper_flight/input_buffer.hpp"
#include <ros/ros.h>
#include <mavros_msgs/ExtendedState.h>

#include "flygripper_common/type/actuator_state.hpp"
#include "flygripper_common/type/battery.hpp"
#include "flygripper_common/type/fc_state.hpp"
#include "flygripper_common/type/imu.hpp"
#include "flygripper_common/type/odometry.hpp"
#include "flygripper_common/type/reference.hpp"
#include "flygripper_common/type/robot_command.hpp"
#include "flygripper_common/type/robot_parameter.hpp"
#include "flygripper_common/type/robot_state.hpp"
#include "flygripper_common/type/rotor_speed.hpp"
#include "flygripper_common/type/trajectory.hpp"
#include "flygripper_common/utils/time.hpp"
#include "flygripper_controller/geometric_controller.hpp"
#include "flygripper_flight/external_wrench_estimator.hpp"
#include "flygripper_flight/fsm_parm.hpp"
#include "flygripper_flight/rc_utils.hpp"
#include "flygripper_flight/rpm_map.hpp"
#include "flygripper_flight/sampler.hpp"
#include "flygripper_flight/thrust_acc_map.hpp"
#include "flygripper_msgs/GraspState.h"
#include "flygripper_msgs/FlightTrajectoryStatus.h"
#include "flygripper_msgs/PerchingCommand.h"
#include "flygripper_msgs/PerchingStatus.h"

namespace flygripper_flight {
using namespace flygripper_common;
using flygripper_controller::ControllerBase;
using flygripper_controller::GeometricController;

class Fsm {
 public:
  Fsm(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  virtual ~Fsm();

  void publishZeroCommand();

  void rosSetup(ros::NodeHandle& nh);

  void rosOdomCallback(const ros::MessageEvent<const nav_msgs::Odometry>& event);

  void rosImuCallback(const ros::MessageEvent<const sensor_msgs::Imu>& event);

  void rosActuatorStateCallback(const ros::MessageEvent<const flygripper_msgs::ActuatorState>& event);

  void rosBatteryCallback(const ros::MessageEvent<const sensor_msgs::BatteryState>& event);

  void rosRotorSpeedCallback(const ros::MessageEvent<const mavros_msgs::ESCTelemetry>& event);

  void rosRcCallback(const ros::MessageEvent<const mavros_msgs::RCIn>& event);

  void rosFcStateCallback(const ros::MessageEvent<const mavros_msgs::State>& event);
  void rosExtendedStateCallback(const ros::MessageEvent<const mavros_msgs::ExtendedState>& event);

  void rosTrajCallback(const ros::MessageEvent<const flygripper_msgs::PolynomialTrajectory>& event);
  void rosPerchingCallback(const ros::MessageEvent<const flygripper_msgs::PerchingCommand>& event);

  void controlLoop(const ros::TimerEvent& event);

 private:
  void updateParameter();
  void consumeInputs();
  void publishRuntimeDiagnostics();
  void publishFlightTrajectoryStatus();
  void consumePerchingCommand();
  bool updatePerching();
  bool perchingActive() const;
  void abortPerching(const std::string& reason);
  void publishPerchingStatus();
  void publishPerchingThrust(double throttle);
  double perchingRecoveryThrottle(double computed) const;
  bool perchingActuatorFresh() const;
  void clearPerchingTrajectory();
  void rejectPerchingSupport(const std::string& reason);
  bool publishPerchingEmergencyRecovery();
  InputBuffer<flygripper_msgs::PerchingCommand> perching_input_;
  uint64_t perching_input_count_{0}, perching_last_mission_{0};
  uint64_t perching_last_status_mission_{0};
  uint8_t perching_last_status_phase_{255};
  flygripper_msgs::PerchingCommand perching_config_;
  flygripper_msgs::PerchingStatus perching_status_;
  double perching_prepared_at_{0.0}, perching_started_at_{0.0},
      perching_phase_at_{0.0}, perching_settled_at_{-1.0},
      perching_last_status_{-1.0}, perching_recovery_at_{-1.0};
  bool perching_start_requested_{false};
  Vector<3> perching_position_{Vector<3>::Zero()}, perching_retreat_start_{Vector<3>::Zero()};
  double perching_yaw_{0.0}, perching_fcu_yaw_{0.0};
  double last_powered_throttle_{NAN}, last_powered_throttle_at_{-1.0};
  double perching_rescue_until_{-1.0};
  // A short, cached positive-thrust bridge when FCU state/sensors expire during
  // deliberate unloading. This is never authorization for renewed autonomy.
  double perching_emergency_until_{-1.0};
  struct Inputs {
    InputSample<sensor_msgs::Imu> imu;
    InputSample<nav_msgs::Odometry> odom;
    InputSample<mavros_msgs::RCIn> rc;
    InputSample<mavros_msgs::State> fc;
    InputSample<mavros_msgs::ExtendedState> extended;
    InputSample<sensor_msgs::BatteryState> battery;
    InputSample<mavros_msgs::ESCTelemetry> rotor;
    InputSample<flygripper_msgs::ActuatorState> actuator;
    InputSample<flygripper_msgs::PolynomialTrajectory> trajectory;
  };
  InputBuffer<sensor_msgs::Imu> imu_input_;
  InputBuffer<nav_msgs::Odometry> odom_input_;
  InputBuffer<mavros_msgs::RCIn, 64> rc_input_buffer_;
  InputBuffer<mavros_msgs::State> fc_input_;
  InputBuffer<mavros_msgs::ExtendedState> extended_input_;
  InputBuffer<sensor_msgs::BatteryState> battery_input_;
  InputBuffer<mavros_msgs::ESCTelemetry> rotor_input_;
  InputBuffer<flygripper_msgs::ActuatorState> actuator_input_;
  InputBuffer<flygripper_msgs::PolynomialTrajectory> trajectory_input_;
  Inputs applied_;  // Only the control thread owns decoded state and this snapshot.
  bool rc_release_seen_{false}, rc_traj_release_seen_{false}, rc_snapshot_overflow_{false},
      rc_invalid_seen_{false};
  double input_lock_copy_duration_{0.0}, input_decode_duration_{0.0};
  uint64_t input_snapshot_misses_{0};
  double loop_started_{0.0}, last_loop_started_{0.0};
  double loop_period_{0.0}, input_copy_duration_{0.0}, control_duration_{0.0};
  double last_diagnostic_{0.0}, max_loop_period_{0.0}, max_control_duration_{0.0};
  std::string fault_reason_, last_handover_reason_;
  double parameter_duration_{0.0}, compute_duration_{0.0};
  bool handover_from_wait_{false};
  std::string guided_entry_mode_;
  bool guided_entry_verified_{false};
  bool trajectory_sampled_this_cycle_{false};
  bool trajectory_control_published_this_cycle_{false};
  bool trajectory_message_decoded_{false};
  bool awaiting_first_trajectory_{false};
  flygripper_msgs::PolynomialTrajectoryConstPtr active_trajectory_message_;
  flygripper_msgs::PolynomialTrajectoryConstPtr pending_trajectory_message_;
  Trajectory pending_trajectory_;
  bool flight_status_initialized_{false};
  bool last_flight_status_active_{false};
  uint32_t last_flight_status_id_{0};
  ros::Time last_flight_status_start_;
  double last_flight_status_publish_{0.0};
  uint64_t guided_entry_completed_baseline_{0};

  bool updateRobotState();

  void updateFsmStateAndReference();

  enum class FsmState : int {
    STATE_MANUAL = 0,
    STATE_GUIDED = 1,  // hover
    STATE_TRAJ = 2,    // command by traj generated by planner or joystick
    STATE_WAIT_GUIDED = 3,
    STATE_HANDOVER = 4
  };

  enum class ReferenceType : int {
    TYPE_TAKEOFF = 0,
    TYPE_HOVER = 1,
    TYPE_VEL = 2,
    TYPE_ADMITTANCE = 3,
    TYPE_TRAJ = 4
  };

  std::string ReferenceTypeToString(const ReferenceType& ref_type) {
    switch (ref_type) {
      case ReferenceType::TYPE_TAKEOFF:
        return "TAKEOFF";
      case ReferenceType::TYPE_HOVER:
        return "HOVER";
      case ReferenceType::TYPE_VEL:
        return "VEL";
      case ReferenceType::TYPE_ADMITTANCE:
        return "ADMITTANCE";
      case ReferenceType::TYPE_TRAJ:
        return "TRAJ";
      default:
        return "Unknown";
    }
  }

  std::string FsmStateToString(const FsmState& fsm_state) {
    switch (fsm_state) {
      case FsmState::STATE_MANUAL:
        return "MANUAL";
      case FsmState::STATE_GUIDED:
        return "GUIDED";
      case FsmState::STATE_TRAJ:
        return "TRAJ";
      case FsmState::STATE_WAIT_GUIDED:
        return "WAIT_GUIDED";
      case FsmState::STATE_HANDOVER:
        return "HANDOVER";
      default:
        return "Unknown";
    }
  }

  void setFsmState(const FsmState& fsm_state);

  void setReferenceType(const ReferenceType& ref_type);

  void publishTrajTrigger();

  bool requestFcMode(const std::string& mode);
  bool prepareGuidedEntry();
  void beginHandover(const std::string& reason);
  void updateHandover();
  void stopAutonomy();
  bool fcStateFresh() const;

  static constexpr Scalar kVelocityDeadZone = 0.01;
  static constexpr Scalar kMaxVelocityToHoverMode = 3.0;

  //! ros
  // Parameters
  FsmParameterConfig fsm_param_config_;
  FsmParameter::Ptr fsm_param_;

  RobotParameterConfig robot_param_config_;
  RobotParameter::Ptr robot_param_;

  InputCallbackQueues input_queues_;
  ros::CallbackQueue control_queue_;
  std::unique_ptr<ros::AsyncSpinner> control_spinner_;
  std::unique_ptr<ModeWorker> mode_worker_;
  ros::Timer loop_timer_;
  ros::Duration dt_;
  ros::SubscribeOptions rc_options_, fc_state_options_, battery_options_,
      odom_options_, imu_options_, actuator_state_options_,
      rotor_speed_options_, traj_options_, extended_state_options_, perching_options_;
  ros::Subscriber rc_sub_, fc_state_sub_, battery_sub_, odom_sub_, imu_sub_,
      actuator_state_sub_, rotor_speed_sub_, traj_sub_, extended_state_sub_, perching_sub_;
  ros::Publisher rotor_command_pub_, thrust_att_command_pub_,
      actuator_command_pub_, traj_trigger_pub_, ctrl_state_pub_, reference_pub_,
      robot_state_pub_, ext_indi_pub_, ext_l1_pub_, ext_tact_pub_,
      interactive_wrench_pub_, runtime_diagnostic_pub_, flight_trajectory_status_pub_, perching_status_pub_;
  ros::ServiceClient mavros_setmode_srv_, mavros_getparam_srv_;

  //  flygripper_msgs::RotorCommand rotor_command_msg_;
  mavros_msgs::ActuatorControl rotor_command_msg_;
  mavros_msgs::AttitudeTarget thrust_att_command_msg_;
  flygripper_msgs::ActuatorCommand actuator_command_msg_;
  nav_msgs::Path reference_msg_;
  flygripper_msgs::RobotState robot_state_msg_;
  geometry_msgs::WrenchStamped interactive_wrench_msg_;

  //! core
  Sampler::Ptr sampler_;
  std::vector<RpmMap> rpm_maps_;
  ThrustAccMap thrust_acc_map_;
  Scalar now_;
  RobotState robot_state_;
  Odometry odom_;
  Imu imu_;
  // Callback processing times, kept separate from measurement timestamps.
  double odom_receive_stamp_{-1.0};
  double imu_receive_stamp_{-1.0};
  double imu_callback_gap_{-1.0}, odom_callback_gap_{-1.0};
  Battery battery_;
  ActuatorState actuator_state_;
  RotorSpeed rotor_speed_;
  Trajectory trajectory_;
  RobotCommand robot_command_;

  VelocityControlSystem::Input velocity_control_;
  AdmittanceControlSystem::Input external_wrench_;
  ExternalWrenchEstimator<4>::Ptr ext_wrench_est_;

  ReferenceVector references_;
  RcInput rc_input_;
  FcState fc_state_;
  mavros_msgs::ExtendedState extended_state_;
  Scalar extended_state_stamp_{INF};
  TakeoffPlan takeoff_plan_;
  bool takeoff_requested_{false};
  bool attitude_safety_handover_{false};
  bool velocity_teleop_ready_{false};
  Scalar mode_request_start_{0.0};
  Scalar last_mode_request_{-INF};
  RobotCommandMode session_command_mode_{RobotCommandMode::MODE_ATTITUDE};

  FsmState fsm_state_{FsmState::STATE_MANUAL};
  ReferenceType ref_type_{ReferenceType::TYPE_TAKEOFF};

  flygripper_controller::ControllerBase::Ptr controller_;
  //! debug
  Timer tic_toc_;
};
}  // namespace flygripper_flight
