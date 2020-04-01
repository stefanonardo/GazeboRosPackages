/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 * https://www.humanbrainproject.eu
 *
 * The Human Brain Project is a European Commission funded project
 * in the frame of the Horizon2020 FET Flagship plan.
 * http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
#ifndef AVATAR_CONTROL_PLUGIN_H
#define AVATAR_CONTROL_PLUGIN_H

#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/JointStates.h>

#include <ros/ros.h>

#include <mutex>

namespace gazebo
{
  // Constant Forces:
  // Applies a constant force to given joints on each Update until cancelled
  static std::map<std::string, double> constantJointForces;

typedef std::map<std::string, physics::JointPtr> JointMap;

class AvatarControlPlugin : public ModelPlugin
{

public:

  AvatarControlPlugin();
  ~AvatarControlPlugin();

  // Load the plugin and initilize all controllers
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Simulation update callback function
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:

  // go through all controller elements in sdf and create controllers accordingly
  void ParseControllers(const sdf::ElementPtr &sdf);

  // get controller type from SDF
  std::string GetControllerType(const sdf::ElementPtr &sdf_ctrl_def);

  // Create controllers
  void CreateControllerModelPositionOnGround(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllerModelRotation(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllerLinkPoseTarget(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllerLinkPositionTarget(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllersLinkVelocities(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllerAllJointsSetPosition(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllerAllJointsPIDPositionTarget(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllersModelPoseTarget(const sdf::ElementPtr &sdf_elem_controller);
  void CreateControllerConstantJointForces(const sdf::ElementPtr &sdf_elem_controller);

  // ROS topic callbacks
  void ModelRotationCB(const geometry_msgs::Quaternion::ConstPtr &msg);
  void LinkPoseTargetCB(const geometry_msgs::Pose::ConstPtr &msg, const physics::LinkPtr &link);
  void LinkPositionTargetCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link);
  void LinkRotationTargetCB(const geometry_msgs::Quaternion::ConstPtr &msg, const physics::LinkPtr &link);
  void LinkLinearVelocityCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link);
  void LinkAngularVelocityCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link);
  void JointSetPositionCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::JointPtr &joint);
  void JointPIDPositionTargetCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::JointPtr &joint);
  void JointStatesPIDPositionTargetCB(const gazebo_msgs::JointStates::ConstPtr &msg);
  void JointPIDParamCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::JointPtr &joint);
  void ModelPoseTargetCB(const geometry_msgs::Pose::ConstPtr &msg);
  void ConstantJointForceCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::JointPtr &joint);

  // set link pose targets
  void SetLinkPositionTarget(double x, double y, double z, const physics::LinkPtr &link);
  void SetLinkRotationTarget(double w, double x, double y, double z, const physics::LinkPtr &link);

  // update link velocities based on pose target
  void UpdateLinkLinearVelocityFromPositionTarget(const physics::LinkPtr &link);
  void UpdateLinkAngularVelocityFromRotationTarget(const physics::LinkPtr &link);

  // update joint positions instantaneously from position targets
  void UpdateJointPositionFromPositionTarget(const physics::JointPtr &joint);

  void UpdateModelVelocitiesFromModelPoseTarget();

  // ROS node handle
  ros::NodeHandle node_handle_;

  // Pointer to the model
  physics::ModelPtr model_;

  // Pointer to the update event connection
  event::ConnectionPtr eventconnection_update_world_;

  // ROS subscribers
  std::vector<ros::Subscriber> subscribers_model_rotation_;
  std::vector<ros::Subscriber> subscribers_link_pose_;
  std::vector<ros::Subscriber> subscribers_link_linear_velocity_;
  std::vector<ros::Subscriber> subscribers_link_angular_velocity_;
  std::vector<ros::Subscriber> subscribers_joint_set_position_;
  std::vector<ros::Subscriber> subscribers_joint_pid_position_target_;
  std::vector<ros::Subscriber> subscribers_joint_pid_params_;
  std::vector<ros::Subscriber> subscribers_model_pose_target_;
  std::vector<ros::Subscriber> subscribers_constant_joint_force_;

  /// \brief keep track of controller update sim-time.
  gazebo::common::Time last_update_time_;

  /// \brief Controller update mutex.
  std::mutex mutex_;

  float rotation_velocity_speed_factor_;
  //TODO: general speed factors for link velocities
  //TODO: maybe better to set as parameter on a per controller basis
  float link_linear_velocity_speed_factor_, link_angular_velocity_speed_factor_;
  float link_pos_target_diff_threshold_, link_rot_target_diff_threshold_;
  // saves parent model rotation updates from topics (applied during OnUpdate() )
  ignition::math::Quaterniond model_target_rotation_;
  std::shared_ptr<ignition::math::Pose3d> model_target_pose_;
  // position targets for links
  std::map<physics::LinkPtr, ignition::math::Vector3d> link_position_targets_;
  // rotation targets for links
  std::map<physics::LinkPtr, ignition::math::Quaterniond> link_rotation_targets_;
  // position targets for joints
  //std::map<physics::JointPtr, double[3]> joint_position_targets_;

  std::shared_ptr<physics::JointController> joint_controller_;
  
  // whether to keep the parent model positioned on ground
  bool keep_model_on_ground_;
  // determines ability to move up when recalculating parent model position on ground
  float step_height_;

};

} // namespace gazebo

#endif
