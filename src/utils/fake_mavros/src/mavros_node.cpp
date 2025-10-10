#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Empty.h>
#include <flygripper_msgs/ActuatorCommand.h>

#include <ros/ros.h>

//! function 1: joy -> /mavros/rc/in -> fsm

//! function 2: /mavros/set_mode <-> fsm
//!             /mavros/state ->

//! function 3: joy -> /mavros/cmd/arming

//! function 4: joy -> takeoff

// XBOX
//           LT(5)              RT(4)
//           LB(B6)             RB(B7)
//             1                  3               Y(B4)
//             Λ                  Λ                                  ↑(7)
//             |                  |         X(B3)     B(B1)     ←(6)
//             |                  |
//    0  <—— ——         2  <—— ——                A(B0)
//
enum JoyAxis{
  AXIS_LL=0,
  AXIS_LU=1,
  AXIS_RL=2,
  AXIS_RU=3,
  AXIS_RT=4,
  AXIS_LT=5,
  AXIS_DL=6,
  AXIS_DU=7,
  AXIS_SIZE=8
};

enum JoyButton{
  BUTTON_A=0,
  BUTTON_B=1,
  BUTTON_X=3,
  BUTTON_Y=4,
  BUTTON_LB=6,
  BUTTON_RB=7,
};

// 8 BitDo
//           LT(2)              RT(5)
//           LB(B4)             RB(B5)
//             1                  4               X(B3)
//             Λ                  Λ                                  ↑(7)
//             |                  |         Y(B2)     A(B1)     ←(6)
//             |                  |
//    0  <—— ——         3  <—— ——                B(B0)
//
//enum JoyAxis{
//  AXIS_LL=0,
//  AXIS_LU=1,
//  AXIS_LT=2,
//  AXIS_RL=3,
//  AXIS_RU=4,
//  AXIS_RT=5,
//  AXIS_DL=6,
//  AXIS_DU=7,
//  AXIS_SIZE=8
//};
//
//enum JoyButton{
//  BUTTON_B=0,
//  BUTTON_A=1,
//  BUTTON_Y=2,
//  BUTTON_X=3,
//  BUTTON_LB=4,
//  BUTTON_RB=5,
//};

// TODO: Tango II
//     A(4)       B(5)       C(6)        D(7)
//     3  <—— ——           0  <—— ——
//              |                    |
//              |                    |
//              V                    V
//              2                    1
//

enum RcChannel{
  RC_ROLL=0,
  RC_PITCH=1,
  RC_THROTTLE=2,
  RC_YAW=3,
  RC_GUIDED=4,
  RC_TRAJ=5,
  RC_MORPH_TRI=7,
  RC_MORPH=8,
};

class FakeMavros {
 public:
  FakeMavros(const ros::NodeHandle& nh,
             const ros::NodeHandle& pnh):
      nh_(nh), pnh_(pnh),
      armed_(true), guided_(false), traj_(false), morph_trigger_(false),
      mode_("STABILIZE"){

    pnh.param("morph_teleop", morph_teleop_, false);
    ROS_INFO("morph_teleop: %d", morph_teleop_);

    joy_sub_ = nh_.subscribe("joy", 10, &FakeMavros::joyCallback, this);

    state_pub_ = nh_.advertise<mavros_msgs::State>("/mavros/state", 10, true);
    rc_in_pub_ = nh_.advertise<mavros_msgs::RCIn>("/mavros/rc/in", 10);
    battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("/mavros/battery", 10);

    arming_srv_ = nh_.advertiseService("/mavros/cmd/arming", &FakeMavros::armingCallback, this);
    mode_srv_ = nh_.advertiseService("/mavros/set_mode", &FakeMavros::setModeCallback, this);

    main_loop_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), &FakeMavros::mainLoop, this);

    reset_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    actuator_cmd_pub_ = nh_.advertise<flygripper_msgs::ActuatorCommand>("actuator_command", 10);
  }

  FakeMavros():FakeMavros(ros::NodeHandle(),ros::NodeHandle("~")){}

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    static int last_button_a  = 0; // for reset
    static int last_button_b  = 0; // for morph trigger
    static int last_button_lb = 0; // for auto mode
    static int last_button_rb = 0; // for traj mode

    mavros_msgs::RCIn rc_in;
    rc_in.rssi = 255;
    // Optional: Set a value for RSSI

    rc_in.channels.resize(12, 1060.0);

    // Guided toggle
    if(joy_msg->buttons[BUTTON_LB] != last_button_lb){
      if(joy_msg->buttons[BUTTON_LB] == 0){
        mode_ = (mode_=="STABILIZE") ? "GUIDED_NOGPS":"STABILIZE";
        guided_ = (armed_&&mode_=="GUIDED_NOGPS") && (!guided_);
      }
      last_button_lb = joy_msg->buttons[BUTTON_LB];
    }

    // Plan toggle
    if(joy_msg->buttons[BUTTON_RB] != last_button_rb){
      if(joy_msg->buttons[BUTTON_RB] == 0){
        traj_ = !traj_;
      }
      last_button_rb = joy_msg->buttons[BUTTON_RB];
    }

    // Arm toggle
    if(joy_msg->buttons[BUTTON_A] != last_button_a){
      if(joy_msg->buttons[BUTTON_A] == 0){
        reset_ = true;
      }
      last_button_a = joy_msg->buttons[BUTTON_A];
    }

    // Morph toggle
    if(joy_msg->buttons[BUTTON_B] != last_button_b){
      if(joy_msg->buttons[BUTTON_B] == 0){
        morph_trigger_ = !morph_trigger_;
      }
      last_button_b = joy_msg->buttons[BUTTON_B];
    }

    // Convert joy_msg to rc_in.channels
    rc_in.channels[RC_ROLL]     = 1500 - 440 * joy_msg->axes[AXIS_RL];
    rc_in.channels[RC_PITCH]    = 1500 - 440 * joy_msg->axes[AXIS_RU];
    rc_in.channels[RC_THROTTLE] = 1500 + 440 * joy_msg->axes[AXIS_LU];
    rc_in.channels[RC_YAW]      = 1500 - 440 * joy_msg->axes[AXIS_LL];
    rc_in.channels[RC_GUIDED]   = 1500 + 440 * (mode_=="GUIDED_NOGPS"?1:-1);
    rc_in.channels[RC_TRAJ]     = 1500 + 440 * (traj_?1:-1);
    rc_in.channels[RC_MORPH_TRI]= 1500 + 440 * (morph_trigger_?1:-1);
    rc_in.channels[RC_MORPH]    = 1500 + 440 * joy_msg->axes[AXIS_LT];

    if (morph_teleop_){
      flygripper_msgs::ActuatorCommand actuator_cmd_msg;
      actuator_cmd_msg.header.stamp = ros::Time::now();
      actuator_cmd_msg.mode = flygripper_msgs::ActuatorCommand::MODE_POSITION;
      actuator_cmd_msg.reference_position = (1 - joy_msg->axes[AXIS_LT]) / 2.0 * 12.5; // NOTED: simulation always goes from 0 to 12.5 instead of real from 0 to -12.5
      actuator_cmd_pub_.publish(actuator_cmd_msg);
    }

    // Publish the converted RCIn message
    rc_in_pub_.publish(rc_in);
  }

  bool armingCallback(mavros_msgs::CommandBool::Request &req,
                      mavros_msgs::CommandBool::Response &res)
  {
    ROS_INFO("fake_mavros arming request: %d", int(req.value));
    armed_ = req.value;
    res.success = true;
    res.result = 0;
    return true;
  }

  bool setModeCallback(mavros_msgs::SetMode::Request &req,
                       mavros_msgs::SetMode::Response &res)
  {
    ROS_INFO("fake_mavros set_mode request: %s", req.custom_mode.c_str());
    mode_ = req.custom_mode;
    res.mode_sent = true;
    return true;
  }

  void publishState() {
    auto state_msg = boost::make_shared<mavros_msgs::State>();
    state_msg->header.stamp = ros::Time::now();
    state_msg->connected = true;
    state_msg->armed = armed_;
    state_msg->guided = false;
    state_msg->manual_input = true;
    state_msg->mode = mode_;
    state_msg->system_status = 4; //3 ground //4 takeoff

    state_pub_.publish(state_msg);
  }

  void publishBattery() {
    auto battery_msg = boost::make_shared<sensor_msgs::BatteryState>();
    battery_msg->header.stamp = ros::Time::now();
    battery_msg->voltage = 12.0;
    battery_msg->current = 0.0;
    battery_msg->temperature = 0.0;
    battery_msg->charge = 100.0;
    battery_msg->capacity = 100.0;
    battery_msg->design_capacity = 100.0;
    battery_msg->percentage = 100.0;
    battery_msg->power_supply_status = 0;
    battery_msg->power_supply_health = 0;
    battery_msg->power_supply_technology = 0;
    battery_pub_.publish(battery_msg);
  }

  void mainLoop(const ros::WallTimerEvent& time) {
    if(reset_){
      reset();
      reset_ = false;
    }
    publishState();
    publishBattery();
  }

  void reset() {
    armed_ = true;
    guided_ = false;
    traj_ = false;
    mode_ = "STABILIZE";
    std_srvs::Empty reset_srv;
    if (reset_client_.call(reset_srv)){
      ROS_INFO("fake_mavros reset gazebo world");
    } else {
      ROS_ERROR("fake_mavros failed to reset gazebo world");
    }
  }

 private:
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer arming_srv_, mode_srv_;
  ros::Subscriber joy_sub_;
  ros::Publisher state_pub_, rc_in_pub_, battery_pub_, actuator_cmd_pub_;
  ros::ServiceClient reset_client_;
  ros::WallTimer main_loop_timer_;
  std::string mode_;
  bool armed_;
  bool guided_ = false;
  bool traj_ = false;
  bool reset_ = false;
  bool morph_trigger_ = false;
  bool morph_teleop_ = false;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_mavros_node");
  FakeMavros fake_mavros;
  ros::spin();
  return 0;
}
