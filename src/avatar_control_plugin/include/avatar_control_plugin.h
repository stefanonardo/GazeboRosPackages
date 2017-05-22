/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
 * ---LICENSE-END **/
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

#include <ros/ros.h>

#include <mutex>

namespace gazebo
{

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

  // Create controller for parent model, keeping it on ground level without gravity influencing it
  void CreateControllerModelPositionOnGround(const sdf::ElementPtr &sdf_elem_controller);
  // Create controller for parent model rotation
  void CreateControllerModelRotation(const sdf::ElementPtr &sdf_elem_controller);
  // Create controller for link pose
  void CreateControllerLinkPose(const sdf::ElementPtr &sdf_elem_controller);
  // Create controller for link velocity
  void CreateControllerLinkLinearVelocity(const sdf::ElementPtr &sdf_elem_controller);

  // Generic model rotation command callback function (ROS topic)
  void ModelRotationCB(const geometry_msgs::Quaternion::ConstPtr &msg);
  // Generic link pose command callback function (ROS topic)
  void LinkPoseCB(const geometry_msgs::Pose::ConstPtr &msg, const physics::LinkPtr &link);
  // Generic link velocity command callback function (ROS topic)
  void LinkLinearVelocityCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link);

  // update link velocities based on pose target
  void UpdateLinkVelocitiesFromPoseTargets(const physics::LinkPtr &link);
  void UpdateLinkLinearVelocityFromPositionTarget(const physics::LinkPtr &link);
  void UpdateLinkAngularVelocityFromRotationTarget(const physics::LinkPtr &link);

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
  gazebo::math::Quaternion model_target_rotation_;
  // pose targets for links
  std::map<physics::LinkPtr, gazebo::math::Pose> link_pose_targets_;
  
  // whether to keep the parent model positioned on ground
  bool keep_model_on_ground_;
  // determines ability to move up when recalculating parent model position on ground
  float step_height_;

};

} // namespace gazebo

#endif
