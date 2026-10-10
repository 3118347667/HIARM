#pragma once
#include <Eigen/Eigen>
#include <atomic>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>
#include <flygripper_msgs/ActuatorCommand.h>
#include <flygripper_msgs/ActuatorState.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>

#include "flygripper_gazebo_plugins/gazebo_utils.hpp"

namespace gazebo {
// Used in OnContact loop
struct ContactInfo{
  std::string gripper_link_name;
  physics::CollisionPtr gripper_collision;
  physics::CollisionPtr object_collision;
  GzVector3d sum_force;
  GzVector3d contact_position;
  GzVector3d object_position;
  int sum = 0;
};
// Used to gather the ContactInfo
struct ObjectContactInfo{
  // all forces effecting on the object
  std::map<std::string, GzVector3d> applied_forces;

  // contact
  std::map<std::string, int> contact_cnts;
  std::map<std::string, GzVector3d> contact_positions;
  std::map<std::string, GzVector3d> contact_object_positions;
  std::map<std::string, physics::CollisionPtr> contact_gripper_collisions;
  std::map<std::string, physics::CollisionPtr> contact_object_collisions;
};
// Used to deal with attachment
class ObjectAttachInfo{
 public:
  std::string contact_gripper_name;
  std::string attach_gripper_name;
  int grasp_count;
  bool attached;

  // original contact distance
  GzVector3d contact_position;
  GzVector3d contact_object_position;

  // To calculate the current contact distance
  physics::CollisionPtr contact_gripper_collision;
  physics::CollisionPtr contact_object_collision;
 private:
  physics::JointPtr fixed_joint_;
  physics::ModelPtr model_;
  bool disable_colllisions_on_attach_;

 public:
  ObjectAttachInfo() = default;
  ~ObjectAttachInfo() = default;

  ObjectAttachInfo(physics::ModelPtr &model,
                   bool disable_colllisions_on_attach):
      model_(model),
      disable_colllisions_on_attach_(disable_colllisions_on_attach){
    Reset();
    fixed_joint_ = model->GetWorld()->Physics()->CreateJoint("fixed");
  };

  void Reset(){
    contact_gripper_name = "";
    attach_gripper_name = "";
    grasp_count = 0;
    attached = false;
  }

  void AttachObjectToGripper(){
    auto object_link = contact_object_collision->GetLink();
    auto gripper_link = boost::dynamic_pointer_cast<physics::Collision>
        (model_->GetWorld()->EntityByName(attach_gripper_name))->GetLink();
    if (!gripper_link){
      ROS_ERROR("Gripper link %s not found, whose contact gripper is %s",
                attach_gripper_name.c_str(), contact_gripper_name.c_str());
      return;
    }
    // Joint::Load expects the anchor pose in the CHILD frame, not a parent-relative pose.
    GzPose3d joint_origin = GzPose3d::Zero;

    fixed_joint_->Load(gripper_link, object_link, joint_origin);
    fixed_joint_->Init();
    if (disable_colllisions_on_attach_){
      object_link->SetCollideMode("none");
    }
  }

  void DetachObjectFromGripper(){
    auto object_link = contact_object_collision->GetLink();
    if (!fixed_joint_){
      ROS_ERROR("Gripper fixed joint not found");
      return;
    }
    fixed_joint_->Detach();
    if (disable_colllisions_on_attach_){
      object_link->SetCollideMode("all");
    }
  }
};

class GazeboGraspPlugin : public ModelPlugin {
 public:
  GazeboGraspPlugin(){
    node_ = transport::NodePtr(new transport::Node());
  }

  virtual ~GazeboGraspPlugin() {
    // Release filter to make it safe to reload the model with plugin
    if (!filter_name_.empty() && world_)
    {
      physics::ContactManager *contactManager =  world_->Physics()->GetContactManager();
      if (contactManager)
        contactManager->RemoveFilter(filter_name_);
    }
    update_connection_.reset();
    if (node_) node_->Fini();
    node_.reset();
    Reset();
  }

  virtual void Load(physics::ModelPtr model,
                    sdf::ElementPtr sdf){
    ROS_INFO("Loading %s plugin", kPluginName.c_str());

    // Save pointers to the model
    model_ = model;
    sdf_ = sdf;

    // Error message if the model couldn't be found
    if (!model_)
    {
      ROS_ERROR("%s plugin (ns = %s) couldn't get model from pointer",
                kPluginName.c_str(), robot_namespace_.c_str());
      return;
    }
    world_ = model_->GetWorld();
    getSdfParam<bool>(kPluginName, sdf_, "commandedRelease", commanded_release_, false);
    getSdfParam<double>(kPluginName, sdf_, "releaseStartPosition", release_start_position_, 4.5);
    getSdfParam<double>(kPluginName, sdf_, "fullyOpenPosition", fully_open_position_, 0.5);

    // Check that ROS has been initialized
    if(!ros::isInitialized())
    {
      ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin %s. "
                "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)",
                kPluginName.c_str());
      return;
    }

    // Get robot namespace
    getSdfParam<std::string>(kPluginName,
                             sdf_, "robotNamespace", robot_namespace_, model_->GetName());
    double update_rate;
    if (!getSdfParam<double>(kPluginName,
                             sdf_, "updateRate", update_rate)){
      return;
    }
    update_duration_ = 1.0/update_rate;

    std::string disable_collisions_on_attach_str;
    getSdfParam<std::string>(kPluginName,
                             sdf_, "disableCollisionsOnAttach", disable_collisions_on_attach_str, "1");
    if (disable_collisions_on_attach_str == "1" || disable_collisions_on_attach_str == "true")
      disable_collisions_on_attach_ = true;
    else
      disable_collisions_on_attach_ = false;

    if (!getSdfParam<double>(kPluginName,
                             sdf_, "graspForcesAngleTolerance", grasp_forces_angle_tolerance_)){
      return;
    }
    if (grasp_forces_angle_tolerance_ < 90 || grasp_forces_angle_tolerance_ > 180){
      ROS_ERROR("%s plugin error: graspForcesAngleTolerance is %f, "
                "which should be in the range of 90 and 180 degrees.",
                kPluginName.c_str(), grasp_forces_angle_tolerance_);
      return;
    }

    if (!getSdfParam<int>(kPluginName,
                          sdf_, "graspCountThreshold", grasp_count_threshold_)){
      return;
    }
    if (grasp_count_threshold_ < 0){
      ROS_ERROR("%s plugin error: graspCountThreshold is %d, "
                "which should be larger than 0.",
                kPluginName.c_str(), grasp_count_threshold_);
      return;
    }
    if (!getSdfParam<int>(kPluginName,
                          sdf_, "graspCountMax", grasp_count_max_)){
      return;
    }
    if (grasp_count_max_ < grasp_count_threshold_ ){
      ROS_ERROR("%s plugin error: graspCountThreshold is %d, "
                "which should be larger than graspCountThreshold which is %d,"
                "normally twice as many as graspCountThreshold.",
                kPluginName.c_str(), grasp_count_max_, grasp_count_threshold_);
      return;
    }

    if (!getSdfParam<double>(kPluginName,
                             sdf_, "releaseDistanceTolerance", release_distance_tolerance_)){
      return;
    }
    if (release_distance_tolerance_ < 0){
      ROS_WARN("%s plugin warning: releaseDistanceTolerance is %f, "
               "which should be larger than 0. Here is set to %f",
               kPluginName.c_str(), release_distance_tolerance_, -release_distance_tolerance_);
      release_distance_tolerance_ = - release_distance_tolerance_;
    }

    getSdfParam<std::string>(kPluginName,
                             sdf_, "releaseToleranceMode", release_tolerance_mode_, "abs");
    if (release_tolerance_mode_ != "abs" && release_tolerance_mode_ != "-" && release_tolerance_mode_ != "+"){
      ROS_ERROR("%s plugin error: releaseToleranceMode is %s, "
                "which should be abs, - or +.",
                kPluginName.c_str(), release_tolerance_mode_.c_str());
      return;
    }

    // ---------------------------- SETUP of gripper links and collisions -----------------------------------
    std::vector<std::string> gripper_link_names;
    if(!getSdfParams<std::string>(kPluginName, sdf_,
                                  "gripperLink",
                                  gripper_link_names)){
      return;
    };

    std::vector<std::string> collision_names;
    for (std::vector<std::string>::const_iterator gripper_iter =
        gripper_link_names.begin();
         gripper_iter != gripper_link_names.end();
         ++gripper_iter) {
      std::string gripper_link_name = *gripper_iter;
      physics::LinkPtr link = model_->GetLink(gripper_link_name);
      if (!link.get()) {
        ROS_WARN("%s plugin warning: "
                 "gripper link %s is not found, skip",
                 kPluginName.c_str(), gripper_link_name.c_str());
        continue;
      }
      for (unsigned int j = 0; j < link->GetChildCount(); ++j)
      {
        physics::CollisionPtr collision = link->GetCollision(j);
        std::string collision_name = collision->GetScopedName();
        //collision->SetContactsEnabled(true);
        std::map<std::string, physics::CollisionPtr>::iterator collision_iter =
            collision_name_2_collision_.find(collision_name);
        if (collision_iter !=collision_name_2_collision_.end())   //this collision was already added before
        {
          ROS_WARN("%s plugin warning: "
                   "Adding Gazebo collision link element %s multiple times, "
                   "the gazebo grasp handler may not work properly",
                   kPluginName.c_str(), collision_name.c_str());
          continue;
        }
        // Update mapping from collision name to collision object and collision name to gripper link name
        ROS_INFO("%s plugin: Adding Gazebo gripper collision link element %s",
                 kPluginName.c_str(), collision_name.c_str());
        collision_name_2_collision_[collision_name] = collision;
        collision_name_2_gripper_name_[collision_name] = *gripper_iter;
        collision_names.push_back(collision_name);
      }
    }
    node_->Init(world_->Name());
    physics::ContactManager *contactManager = world_->Physics()->GetContactManager();
    contactManager->PublishContacts();  // TODO not sure this is required?
    filter_name_ = model->GetScopedName();
    // Observe all robot contacts for diagnostics; grasp membership remains unchanged.
    for (auto link : model_->GetLinks())
      for (auto collision : link->GetCollisions())
        collision_names.push_back(collision->GetScopedName());
    std::string topic = contactManager->CreateFilter(filter_name_, collision_names);
    if (!contact_sub_)
    {
      ROS_INFO("%s plugin: Subscribing contact manager to topic %s", kPluginName.c_str(), topic.c_str());
      bool latching = false;
      contact_sub_ = node_->Subscribe(topic, &GazeboGraspPlugin::OnContact, this, latching);
    }
    // ------------------------------- END of gripper link and collision -------------------------------------
    last_update_time_ = world_->SimTime().Double();

    // ROS
    // Get parameters/settings for namespace from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);
    if (!getSdfParam<std::string>(kPluginName, sdf_,
                                  "rosGraspTopicName", ros_grasp_topic_name_)){
      return;
    }

    ros_setup_ = false;
    ros_grasp_msg_.data = false;

    // Update connection
    update_connection_ = event::Events::ConnectWorldUpdateBegin
        (boost::bind(&GazeboGraspPlugin::Update, this, _1));

    ROS_INFO("%s plugin loaded in namespace: %s",
             kPluginName.c_str(), robot_namespace_.c_str());
  }

  void CommandCallback(const flygripper_msgs::ActuatorCommand::ConstPtr& msg) {
    opening_requested_ = msg->reference_position < 0.1;
  }

  void StateCallback(const flygripper_msgs::ActuatorState::ConstPtr& msg) {
    measured_actuator_position_ = msg->position;
  }

  void SetupROS(){
    command_sub_ = model_nh_.subscribe("actuator_command", 1, &GazeboGraspPlugin::CommandCallback, this);
    state_sub_ = model_nh_.subscribe("actuator_state", 1, &GazeboGraspPlugin::StateCallback, this);
    ros_grasp_pub_ = model_nh_.advertise<std_msgs::Bool>(
        ros_grasp_topic_name_, 1);
  }

  void PublishGraspMsg(bool attached){
    ros_grasp_msg_.data = attached;
    ros_grasp_pub_.publish(ros_grasp_msg_);
  }

  void OnContact(const ConstContactsPtr &contact_msg){

    for (int i = 0; i < contact_msg->contact_size(); ++i)
    {
      physics::CollisionPtr collision1 =
          boost::dynamic_pointer_cast<physics::Collision>(
              world_->EntityByName(contact_msg->contact(i).collision1()));
      physics::CollisionPtr collision2 =
          boost::dynamic_pointer_cast<physics::Collision>(
              world_->EntityByName(contact_msg->contact(i).collision2()));

      if (collision1 && collision2 &&
          (collision1->GetScopedName().find("support_stage_") != std::string::npos ||
           collision2->GetScopedName().find("support_stage_") != std::string::npos)) {
        double depth = 0.0;
        for (int k=0; k<contact_msg->contact(i).depth_size(); ++k)
          depth = std::max(depth, contact_msg->contact(i).depth(k));
        ROS_WARN_THROTTLE(0.2, "GripperSupportContact %s / %s depth=%g",
                         collision1->GetScopedName().c_str(), collision2->GetScopedName().c_str(), depth);
      }
      if (collision1 && collision2 &&
          (collision1->GetScopedName().find("coke_can") != std::string::npos ||
           collision2->GetScopedName().find("coke_can") != std::string::npos)) {
        // Separate throttle: ordinary finger contacts must not hide rotor impacts.
        if (collision1->GetScopedName().find("_rotor_link") != std::string::npos ||
            collision2->GetScopedName().find("_rotor_link") != std::string::npos) {
          ROS_WARN_THROTTLE(0.1, "CanRotorContact %s / %s",
                           collision1->GetScopedName().c_str(), collision2->GetScopedName().c_str());
        } else {
          ROS_WARN_THROTTLE(0.1, "CanContact %s / %s",
                           collision1->GetScopedName().c_str(), collision2->GetScopedName().c_str());
        }
      }
      if ((collision1 && !collision1->IsStatic()) && (collision2
          && !collision2->IsStatic()))
      {
        std::string name1 = collision1->GetScopedName();
        std::string name2 = collision2->GetScopedName();

        int count = contact_msg->contact(i).position_size();

        // Check to see if the contact arrays all have the same size.
        if ((count != contact_msg->contact(i).normal_size()) ||
            (count != contact_msg->contact(i).wrench_size()) ||
            (count != contact_msg->contact(i).depth_size()))
        {
          ROS_ERROR("%s plugin error: Contact message has invalid array sizes", kPluginName.c_str());
          continue;
        }

        std::string object_collision_name, gripper_collision_name, gripper_link_name;
        physics::CollisionPtr gripper_collision;
        physics::CollisionPtr object_collision;
        physics::Contact contact;
        contact = contact_msg->contact(i);
        if (contact.count < 1)
        {
          ROS_WARN("%s plugin warning: Not enough forces given for contact of %s / %s.",
                   kPluginName.c_str(), name1.c_str(), name2.c_str());
          continue;
        }
        // all force vectors which are part of this contact
        std::vector<GzVector3d> force;

        // find out which part of the colliding entities is the OBJECT, *not* the GRIPPER,
        // and copy all the forces applied to it into the vector 'force'.
        std::map<std::string, std::string>::const_iterator collision_gripper_iter =
            collision_name_2_gripper_name_.find(name2);
        if (collision_gripper_iter != this->collision_name_2_gripper_name_.end())
        {
          // collision 1 is the object
          object_collision_name = name1;
          object_collision = collision1;
          gripper_collision_name = name2;
          gripper_collision = collision2;
          gripper_link_name = collision_gripper_iter->second;
          for (int k = 0; k < contact.count; ++k)
            force.push_back(contact.wrench[k].body1Force);
        }
        else if ((collision_gripper_iter = collision_name_2_gripper_name_.find(name1)) !=
            this->collision_name_2_gripper_name_.end())
        {
          // collision 2 is the object
          object_collision_name = name2;
          object_collision = collision2;
          gripper_collision_name = name1;
          gripper_collision = collision1;
          gripper_link_name = collision_gripper_iter->second;
          for (int k = 0; k < contact.count; ++k)
            force.push_back(contact.wrench[k].body2Force);
        }

        if (!gripper_collision || !object_collision) continue;

        GzVector3d avg_force;
        // compute average/sum of the forces applied on the object
        for (int k = 0; k < force.size(); ++k) avg_force += force[k];
        avg_force /= force.size();

        GzVector3d avg_pos;
        // compute center point (average pose) of all the origin positions of the forces appied
        for (int k = 0; k < contact.count; ++k) avg_pos += contact.positions[k];
        avg_pos /= contact.count;

        GzPose3d gripper_world_pose = gripper_collision->GetLink()->WorldPose();
        GzMatrix4d world2gripper = GzMatrix4d(gripper_world_pose);

        GzMatrix4d world2contact = GzMatrix4d::Identity; // Random orientation is okay as we do not care
        world2contact.SetTranslation(avg_pos);

        GzMatrix4d gripper2world = world2gripper.Inverse();
        GzMatrix4d gripper2contact = gripper2world * world2contact;
        GzVector3d gripper2contact_pos = gripper2contact.Translation();

        // Get the pose of the object and compute it's relative position to the collision surface.
        GzPose3d object_world_pose = object_collision->GetLink()->WorldPose();
        GzMatrix4d world2object = GzMatrix4d(object_world_pose);
        GzMatrix4d gripper2object = gripper2world * world2object;
        GzVector3d gripper2object_pos = gripper2object.Translation();

        {
          boost::mutex::scoped_lock lock(contact_mutex_);
          ContactInfo &contact_info = contacts_[object_collision_name][gripper_collision_name];  //! inserts new entry if doesn't exist
          contact_info.gripper_link_name = gripper_link_name;
          contact_info.gripper_collision = gripper_collision;
          contact_info.object_collision = object_collision;
          contact_info.sum_force += avg_force;
          contact_info.contact_position += gripper2contact_pos;
          contact_info.object_position += gripper2object_pos;
          contact_info.sum++;
        }
      }
    }
  }

  void Update(const common::UpdateInfo& update_info){
    if (!ros_setup_){
      SetupROS();
      ros_setup_ = true;
      ROS_INFO("%s plugin: Setup ROS settings.", kPluginName.c_str());
    }
    auto current_time = update_info.simTime;
    double current_time_sec = current_time.Double();
    double dt = current_time_sec - last_update_time_;

    if (dt < update_duration_){
      return;
    }
    last_update_time_ = current_time_sec;
    // Step 1: copy contacts and do not influence the function OnContact.
    //         contacts[object_collision_name][gripper_collision_name] => ContactInfo
    contact_mutex_.lock();
    std::map<std::string, std::map<std::string, ContactInfo>> contacts(contacts_);
    contacts_.clear(); // clear so it can be filled anew by OnContact().
    contact_mutex_.unlock();

    // Step 2: Iterate contacts and summarize to get object_contacts
    //         object_contacts[object_name] => ObjectContactInfo
    std::map<std::string, ObjectContactInfo> object_contacts;
    UpdateObjectContactInfo(contacts, object_contacts);

    // Step 3: Handle Attachment
    // Iterate object_contacts and handle attachment condition
    std::map<std::string, ObjectContactInfo>::iterator object_contact_iter;
    for (object_contact_iter = object_contacts.begin();
         object_contact_iter != object_contacts.end();
         ++object_contact_iter){

      std::string object_name = object_contact_iter->first;
      ObjectContactInfo& object_contact_info = object_contact_iter->second;

      // object_attaches must be constructed with model and paramters
      if(object_attaches_.find(object_name) == object_attaches_.end()){
        object_attaches_.emplace(
            object_name, ObjectAttachInfo(model_, disable_collisions_on_attach_));
      }

      ObjectAttachInfo& object_attach_info_ = object_attaches_[object_name];
      // If the object is already attached, skip
      if (object_attach_info_.attached || (commanded_release_ && opening_requested_)){
        continue;
      }

      std::set<std::string> target_gripper_names;
      std::map<std::string, std::set<std::string>> target_gripper_pairs;
      // Evaluate the attachment condition, and get target_gripper_names from applied forces from object_contact_info
      // manually assign the min_ratio and min_force
      if (!EvaluateAttachCondition(object_contact_info.applied_forces,
                                   target_gripper_names, target_gripper_pairs,
                                   grasp_forces_angle_tolerance_,
                                   0.3, 1e-3)){
        continue;
      }

      // Update the contact gripper name
      std::string & target_gripper_name = object_attach_info_.contact_gripper_name;
      std::string & opposite_gripper_name = object_attach_info_.attach_gripper_name;
      if (target_gripper_names.find(target_gripper_name) == target_gripper_names.end()){
        // sort to find the most hopeful gripper by contact_cnts among target grippers
        GetMaxContactsGripperNames(target_gripper_names,
                                   object_contact_info.contact_cnts,
                                   target_gripper_name);
        GetMaxContactsGripperNames(target_gripper_pairs[target_gripper_name],
                                   object_contact_info.contact_cnts,
                                   opposite_gripper_name);
        object_attach_info_.grasp_count = 0;
        continue;
      }

      // If the grasp count is less than max count, plus 1
      if (object_attach_info_.grasp_count < grasp_count_max_){
        object_attach_info_.grasp_count++;
      }
      // If the grasp count is more than threshold, set the object attached
      if (object_attach_info_.grasp_count > grasp_count_threshold_){
        ROS_INFO("%s plugin: Object %s has been grasped", kPluginName.c_str(), object_name.c_str());
        object_attach_info_.attached = true;
        PublishGraspMsg(true);
        // Update contact distance
        object_attach_info_.contact_position = object_contact_info.contact_positions[target_gripper_name];
        object_attach_info_.contact_object_position = object_contact_info.contact_object_positions[target_gripper_name];

        object_attach_info_.contact_gripper_collision = object_contact_info.contact_gripper_collisions[target_gripper_name];
        object_attach_info_.contact_object_collision = object_contact_info.contact_object_collisions[target_gripper_name];

        // Attach object link to gripper link
        object_attach_info_.AttachObjectToGripper();
      }
    }

    // Step 4: Handle detachment
    // Iterate object_attaches_ and handle detachment condition
    std::map<std::string, ObjectAttachInfo>::iterator object_attach_iter_;
    for (object_attach_iter_ = object_attaches_.begin();
         object_attach_iter_ != object_attaches_.end();
         ++object_attach_iter_){
      std::string object_name = object_attach_iter_->first;
      ObjectAttachInfo& object_attach_info_ = object_attach_iter_->second;
      // If the object is not attached, skip
      if (!object_attach_info_.attached){
        continue;
      }
      // Calculate current contact distance
      GzPose3d gripper_world_pose = object_attach_info_.contact_gripper_collision->GetLink()->WorldPose();
      GzMatrix4d world2gripper = GzMatrix4d(gripper_world_pose);

      GzMatrix4d gripper2contact = GzMatrix4d::Identity; // Random orientation is okay as we do not care
      gripper2contact.SetTranslation(object_attach_info_.contact_position);

      GzMatrix4d world2contact = world2gripper * gripper2contact;
      GzVector3d contact_world_pos = world2contact.Translation();

      GzPose3d object_world_pose = object_attach_info_.contact_object_collision->GetLink()->WorldPose();
      GzVector3d object_world_pos = object_world_pose.Pos();

      double original_contact_distance = (object_attach_info_.contact_position -
          object_attach_info_.contact_object_position).Length();
      double current_contact_distance = (contact_world_pos - object_world_pos).Length();
      double distance_diff = current_contact_distance - original_contact_distance;
      // For debug
//      ROS_WARN_THROTTLE(1.0,"attach_gripper: %s, max_contact_gripper: %s, "
//                            "distance_diff: %f, tolerance_mode: %s",
//                        object_attach_info_.attach_gripper_name.c_str(),
//                        object_attach_info_.contact_gripper_name.c_str(),
//                        distance_diff, release_tolerance_mode_.c_str());

      // If pass the detach condition of distance tolerance
      bool detach_condition = false;
      detach_condition |= release_tolerance_mode_ == "abs" && std::abs(distance_diff) > release_distance_tolerance_;
      detach_condition |= release_tolerance_mode_ == "+"   && distance_diff > release_distance_tolerance_;
      detach_condition |= release_tolerance_mode_ == "-"   && distance_diff < -release_distance_tolerance_;
      // A latched contact grasp must not be released by transport-induced link motion.
      // Release only during commanded opening; measured near-open position also handles
      // collision pairs whose object-origin distance is insensitive to opening.
      if (commanded_release_) {
        detach_condition = opening_requested_ && measured_actuator_position_ < release_start_position_ &&
            (detach_condition || measured_actuator_position_ < fully_open_position_);
      }
      if (detach_condition){
        ROS_INFO("%s plugin: Object %s has been released", kPluginName.c_str(), object_name.c_str());
        PublishGraspMsg(false);
        // Detach object link from gripper link
        object_attach_info_.DetachObjectFromGripper();
        object_attach_info_.Reset();
      }
    }
  }

  void GetMaxContactsGripperNames(const std::set<std::string>& target_gripper_names,
                                  const std::map<std::string, int>& contact_cnts,
                                  std::string& max_contacts_gripper_name){
    int max_contact_cnts = -1;
    for (auto name : target_gripper_names) {
      std::map<std::string, int>::const_iterator contact_cnt_iter = contact_cnts.find(name);
      if (contact_cnt_iter == contact_cnts.end()) {
        ROS_ERROR("Could not find gripper %s in contact_cnts", name.c_str()); // THIS SHOULD NEVER HAPPEN
        continue;
      }
      int contact_cnt = contact_cnt_iter->second;
      if(contact_cnt > max_contact_cnts){
        max_contact_cnts = contact_cnt;
        max_contacts_gripper_name = name;
      }
    }

  }

  void UpdateObjectContactInfo(const std::map<std::string, std::map<std::string, ContactInfo>>& contacts,
                               std::map<std::string, ObjectContactInfo>& object_contacts){
    std::map<std::string, std::map<std::string, ContactInfo>>::const_iterator
        object_gripper_contact_iter;
    // For all colliding object
    for (object_gripper_contact_iter = contacts.begin();
         object_gripper_contact_iter != contacts.end();
         ++object_gripper_contact_iter){
      std::string object_name = object_gripper_contact_iter->first;
      ObjectContactInfo& object_contact_info = object_contacts[object_name];
      std::map<std::string, ContactInfo>::const_iterator gripper_contact_iter;
      // For all gripper colliding with this object
      for (gripper_contact_iter = object_gripper_contact_iter->second.begin();
           gripper_contact_iter != object_gripper_contact_iter->second.end();
           ++gripper_contact_iter){
        std::string gripper_name = gripper_contact_iter->first;
        const ContactInfo& contact_info = gripper_contact_iter->second;

        // Update force for gripper colliding with this object
        GzVector3d avg_force = contact_info.sum_force / contact_info.sum;
        object_contact_info.applied_forces[gripper_name] = avg_force;

        // Update count for gripper colliding with this object in the sampling period
        int& contact_cnt = object_contact_info.contact_cnts[gripper_name];
        contact_cnt++;

        // Update contact position
        object_contact_info.contact_positions[gripper_name] = contact_info.contact_position / contact_info.sum;
        object_contact_info.contact_object_positions[gripper_name] = contact_info.object_position / contact_info.sum;

        // Update contact collision
        object_contact_info.contact_gripper_collisions[gripper_name] = contact_info.gripper_collision;
        object_contact_info.contact_object_collisions[gripper_name] = contact_info.object_collision;
      }
    }
  }

  bool EvaluateAttachCondition(
      const std::map<std::string, GzVector3d> &gripper_applied_force,
      std::set<std::string> &target_gripper_names,
      std::map<std::string, std::set<std::string>>& target_gripper_pairs,
      double min_angle_diff, double min_ratio, double min_force){
    target_gripper_names.clear();
    std::map<std::string, GzVector3d>::const_iterator iter1, iter2;

    for (iter1 = gripper_applied_force.begin(); iter1 != gripper_applied_force.end(); ++iter1){
      GzVector3d f1 = iter1->second;
      double f1_norm = f1.Length();
      if (f1_norm < min_force) continue;

      for (iter2 = std::next(iter1); iter2 != gripper_applied_force.end(); ++iter2){

        GzVector3d f2 = iter2->second;
        double f2_norm = f2.Length();
        if (f2_norm < min_force) continue;
        double angle_diff = acos(f1.Dot(f2) / (f1_norm * f2_norm)) * 180 / M_PI; // degrees
        std::string name1 = iter2->first;
        std::string name2 = iter1->first;
        // Multiple collision shapes lumped into one link are not opposing fingers.
        if (collision_name_2_collision_.at(name1)->GetLink() ==
            collision_name_2_collision_.at(name2)->GetLink()) continue;

        if (angle_diff >= min_angle_diff){
          double ratio = f1_norm > f2_norm ? f2_norm/f1_norm : f1_norm/f2_norm;
          if (ratio > min_ratio) {
            target_gripper_names.insert(name1);
            target_gripper_names.insert(name2);
            target_gripper_pairs[name1].insert(name2);
            target_gripper_pairs[name2].insert(name1);
          }
        }
      }
    }
    return !target_gripper_names.empty();
  }

  void Reset(){
    if (ros_setup_) {
      ros_grasp_pub_.shutdown();
    }
    ros_setup_ = false;
  }

 private:
  const std::string kPluginName = "GazeboGraspPlugin";

  // Pointer to the gazebo objects
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  sdf::ElementPtr sdf_;

  transport::NodePtr node_; // communication of gazebo
  std::string filter_name_; // the contact scope filter
  transport::SubscriberPtr contact_sub_; // subscriber to contact updates
  boost::mutex contact_mutex_; //mutex protects contacts
  std::map<std::string, physics::CollisionPtr> collision_name_2_collision_;
  std::map<std::string, std::string> collision_name_2_gripper_name_;
  std::map<std::string, std::map<std::string, ContactInfo> > contacts_;
  std::map<std::string, ObjectAttachInfo> object_attaches_;

  double last_update_time_;
  // Parameters
  std::string robot_namespace_;
  double update_duration_;
  bool disable_collisions_on_attach_;
  double grasp_forces_angle_tolerance_;
  int grasp_count_threshold_;
  int grasp_count_max_;
  double release_distance_tolerance_;
  std::string release_tolerance_mode_;

  // Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  // ros
  ros::NodeHandle model_nh_;
  ros::Publisher ros_grasp_pub_;
  ros::Subscriber command_sub_, state_sub_;
  bool commanded_release_ = false;
  double release_start_position_ = 4.5, fully_open_position_ = 0.5;
  std::atomic<bool> opening_requested_{true};
  std::atomic<double> measured_actuator_position_{0.0};
  std::string ros_grasp_topic_name_;
  std_msgs::Bool ros_grasp_msg_;
  bool ros_setup_;
};

} // namespace gazebo {

