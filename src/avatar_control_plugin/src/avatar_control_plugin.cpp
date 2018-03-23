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
/*
 * Desc: Gazebo plugin providing controllers for user interaction
 * This plugin provides ROS topics for user controlled objects.
 * Author: Sandro Weber (webers@in.tum.de)
 */

#include "avatar_control_plugin.h"

#include <boost/bind.hpp>
#include <ros/time.h>

namespace gazebo
{

AvatarControlPlugin::AvatarControlPlugin()
{
}

AvatarControlPlugin::~AvatarControlPlugin()
{
  node_handle_.shutdown();
}

void AvatarControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  // Store the pointer to the model
  this->model_ = parent;

  this->node_handle_ = ros::NodeHandle();
  
  this->model_target_rotation_.Set(1.0, 0.0, 0.0, 0.0);
  this->rotation_velocity_speed_factor_ = 10.0f;

  this->link_linear_velocity_speed_factor_ = 10.0f;
  this->link_angular_velocity_speed_factor_ = 10.0f;
  this->link_pos_target_diff_threshold_ = 0.1f;
  this->link_rot_target_diff_threshold_ = 0.1f;
  
  this->keep_model_on_ground_ = false;
  this->step_height_ = 0.1f;

  ROS_INFO("sdf name %s, sdf description %s", sdf->GetName().c_str(), sdf->GetDescription().c_str());

  // parse through all controllers and create model and link controllers
  this->ParseControllers(sdf);

  // Controller time control.
  this->last_update_time_ = this->model_->GetWorld()->GetSimTime();

  // Listen to the update event. This event is broadcast every simulation iteration.
  eventconnection_update_world_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AvatarControlPlugin::OnUpdate, this, _1));
}

// Called by the world update start event
void AvatarControlPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  gazebo::common::Time curTime = this->model_->GetWorld()->GetSimTime();
  
  // keep model on ground if set
  math::Pose model_world_pose = this->model_->GetWorldPose();
  if (this->keep_model_on_ground_)
  {
    gazebo::math::Vector3 up = gazebo::math::Vector3::UnitZ;
    gazebo::math::Vector3 ground_ray_start = model_world_pose.pos + (up * this->step_height_);
    gazebo::math::Vector3 ground_ray_end = model_world_pose.pos - (up * 100.0f);
    
    gazebo::physics::PhysicsEnginePtr engine = model_->GetWorld()->GetPhysicsEngine();
    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));
    ray->SetPoints(ground_ray_start, ground_ray_end);
    
    double distance = 0.0;
    std::string entity_name = "";
    ray->GetIntersection(distance, entity_name);
    if (!entity_name.empty())
    {
      model_world_pose.pos = ground_ray_start - (distance * up);
    }
  }
  this->model_->SetWorldPose(model_world_pose);
  
  // set angular velocity from target model rotation (this->model_target_rotation_)
  math::Vector3 model_angular_velocity = math::Vector3::Zero;
  math::Quaternion rotation_diff_quat = this->model_target_rotation_ * model_world_pose.rot.GetInverse();

  math::Vector3 rotation_diff_axis;
  double rotation_diff_angle;
  rotation_diff_quat.GetAsAxis(rotation_diff_axis, rotation_diff_angle);
  if (rotation_diff_angle > 0.1f)
  {
    model_angular_velocity = this->rotation_velocity_speed_factor_ * rotation_diff_axis * rotation_diff_angle;
  }
  this->model_->SetAngularVel(model_angular_velocity);

  // update via link pose targets
  for (std::map<physics::LinkPtr, gazebo::math::Pose>::iterator it = link_pose_targets_.begin(); it != link_pose_targets_.end(); ++it) {
    this->UpdateLinkVelocitiesFromPoseTargets(it->first);
  }

  this->last_update_time_ = curTime;
}

///////////////////////////////////////// SDF parser functions ////////////////////////////////////////////

void AvatarControlPlugin::ParseControllers(const sdf::ElementPtr &sdf)
{
  sdf::ElementPtr sdf_elem_controller = sdf->GetElement("controller");
  while (sdf_elem_controller != NULL)
  {
    std::string controller_type = GetControllerType(sdf_elem_controller);
    if (controller_type == "model_position_on_ground")
    {
      this->CreateControllerModelPositionOnGround(sdf_elem_controller);
    }
    else if (controller_type == "model_rotation")
    {
      this->CreateControllerModelRotation(sdf_elem_controller);
    }
    else if (controller_type == "link_velocity")
    {
      this->CreateControllerLinkLinearVelocity(sdf_elem_controller);
    }
    else if (controller_type == "link_pose")
    {
      this->CreateControllerLinkPose(sdf_elem_controller);
    }
    else
    {
      ROS_WARN("Unknown controller type in SDF file: %s", controller_type.c_str());
    }
    
    sdf_elem_controller = sdf_elem_controller->GetNextElement("controller");
  }
}

std::string AvatarControlPlugin::GetControllerType(const sdf::ElementPtr &sdf_elem_controller)
{
  std::string controller_type = "";

  if (sdf_elem_controller != NULL)
  {
    sdf::ParamPtr controller_type_attr = sdf_elem_controller->GetAttribute("type");
    if (controller_type_attr != NULL)
    {
      controller_type = controller_type_attr->GetAsString();
    }
    else
    {
      ROS_WARN("Could not find controller type in SDF file.");
    }
  }
  else
  {
    ROS_WARN("Cannot determine controller type, sdf element not specified");
  }
  
  return controller_type;
}

//////////////////////////////////////// Controller construction //////////////////////////////////////////


void AvatarControlPlugin::CreateControllerModelPositionOnGround(const sdf::ElementPtr &sdf_elem_controller)
{
  sdf::ElementPtr step_height_elem = sdf_elem_controller->GetElement("step_height");
  if (step_height_elem != NULL)
  {
    this->keep_model_on_ground_ = true;
    this->step_height_ = step_height_elem->Get<float>();
  }
  else
  {
    ROS_WARN("ModelPositionOnGround controller has no element <step_height>");
    return;
  }
  
  ROS_INFO("Added new positional controller (keep on ground) for model %s", model_->GetName().c_str());
}

void AvatarControlPlugin::CreateControllerModelRotation(const sdf::ElementPtr &sdf_elem_controller)
{
  std::string topic_name = model_->GetName() + "/cmd_rot";

  model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Quaternion");

  // Add ROS topic for velocity control
  subscribers_model_rotation_.push_back(
      node_handle_.subscribe<geometry_msgs::Quaternion>(
          topic_name, 1, boost::bind(&AvatarControlPlugin::ModelRotationCB, this, _1)
      )
  );

  ROS_INFO("Added new rotation controller for model %s on topic %s", model_->GetName().c_str(), topic_name.c_str());
}

void AvatarControlPlugin::CreateControllerLinkPose(const sdf::ElementPtr &sdf_elem_controller)
{
  sdf::ElementPtr link_name_elem = sdf_elem_controller->GetElement("link_name");
  std::string sdf_link_name = "";
  if (link_name_elem != NULL)
  {
    sdf_link_name = link_name_elem->Get<std::string>();
  }
  else
  {
    ROS_WARN("link pose controller has no element <link_name>");
    return;
  }

  physics::LinkPtr link = model_->GetLink(sdf_link_name);
  if (link == NULL)
  {
    ROS_WARN("no link with name %s found", sdf_link_name.c_str());
    return;
  }

  // generate link topic name using the link name and replace "::" with "/"
  std::string link_name = link->GetName();
  std::string string_old = "::";
  std::string string_new = "/";
  std::string::size_type pos = 0u;
  while ((pos = link_name.find(string_old, pos)) != std::string::npos)
  {
    link_name.replace(pos, string_old.length(), string_new);
    pos += string_new.length();
  }
  std::string topic_name = model_->GetName() + "/" + link_name + "/cmd_pose";

  model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Pose");

  // Add ROS topic for velocity control
  subscribers_link_pose_.push_back(
      node_handle_.subscribe<geometry_msgs::Pose>(
          topic_name, 1, boost::bind(&AvatarControlPlugin::LinkPoseCB, this, _1, link)
      )
  );

  ROS_INFO("Added new pose controller for link %s on topic %s", link->GetName().c_str(), topic_name.c_str());
}

void AvatarControlPlugin::CreateControllerLinkLinearVelocity(const sdf::ElementPtr &sdf_elem_controller)
{
  sdf::ElementPtr link_name_elem = sdf_elem_controller->GetElement("link_name");
  std::string sdf_link_name = "";
  if (link_name_elem != NULL)
  {
    sdf_link_name = link_name_elem->Get<std::string>();
  }
  else
  {
    ROS_WARN("link velocity controller has no element <link_name>");
    return;
  }
  
  physics::LinkPtr link = model_->GetLink(sdf_link_name);
  if (link == NULL)
  {
    ROS_WARN("no link with name %s found", sdf_link_name.c_str());
    return;
  }
  
  // generate link topic name using the link name and replace "::" with "/"
  std::string link_name = link->GetName();
  std::string string_old = "::";
  std::string string_new = "/";
  std::string::size_type pos = 0u;
  while ((pos = link_name.find(string_old, pos)) != std::string::npos)
  {
    link_name.replace(pos, string_old.length(), string_new);
    pos += string_new.length();
  }
  std::string topic_name = model_->GetName() + "/" + link_name + "/cmd_vel";

  model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Vector3");

  // Add ROS topic for velocity control
    subscribers_link_linear_velocity_.push_back(
        node_handle_.subscribe<geometry_msgs::Vector3>(
            topic_name, 1, boost::bind(&AvatarControlPlugin::LinkLinearVelocityCB, this, _1, link)
        )
    );

  ROS_INFO("Added new velocity controller for link %s on topic %s", link->GetName().c_str(), topic_name.c_str());
}

//////////////////////////////////////// ROS topic callback functions //////////////////////////////////////////

void AvatarControlPlugin::ModelRotationCB(const geometry_msgs::Quaternion::ConstPtr &msg)
{
  ROS_DEBUG("ModelRotationCB called! quaternion(xyzw) = %.2f %.2f %.2f %.2f", msg->x, msg->y, msg->z, msg->w);
  this->model_target_rotation_.Set(msg->w, msg->x, msg->y, msg->z);
}

void AvatarControlPlugin::LinkPoseCB(const geometry_msgs::Pose::ConstPtr &msg, const physics::LinkPtr &link)
{
  ROS_DEBUG("LinkPoseCB called! link name = %s, link pos = %.2f %.2f %.2f, link rot = %.2f %.2f %.2f %.2f",
            link->GetName().c_str(),
            msg->position.x, msg->position.y, msg->position.z,
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

  auto target = link_pose_targets_.find(link);
  if (target == link_pose_targets_.end())
  {
    link_pose_targets_.insert(std::make_pair(link, gazebo::math::Pose()));
  }

  link_pose_targets_[link].pos.Set(msg->position.x, msg->position.y, msg->position.z);
  link_pose_targets_[link].rot.Set(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void AvatarControlPlugin::LinkLinearVelocityCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link)
{
  ROS_DEBUG("LinkLinearVelocityCB called! link name = %s, link vel = %.2f %.2f %.2f ",
            link->GetName().c_str(), msg->x, msg->y, msg->z);
  gazebo::math::Vector3 velocity(msg->x, msg->y, msg->z);
  link->SetLinearVel(velocity);
}

//////////////////////////////////////// Update functions called during OnUpdate() ////////////////////////////////

void AvatarControlPlugin::UpdateLinkVelocitiesFromPoseTargets(const physics::LinkPtr &link)
{
  this->UpdateLinkLinearVelocityFromPositionTarget(link);
  this->UpdateLinkAngularVelocityFromRotationTarget(link);
}

void AvatarControlPlugin::UpdateLinkLinearVelocityFromPositionTarget(const physics::LinkPtr &link)
{
  math::Vector3 linear_velocity = math::Vector3::Zero;
  math::Vector3 position_diff = this->link_pose_targets_.at(link).pos - link->GetWorldPose().pos;

  if (position_diff.GetLength() > this->link_pos_target_diff_threshold_)
  {
    linear_velocity = this->link_linear_velocity_speed_factor_ * position_diff;
  }
  link->SetLinearVel(linear_velocity);
}

void AvatarControlPlugin::UpdateLinkAngularVelocityFromRotationTarget(const physics::LinkPtr &link)
{
  math::Vector3 angular_velocity = math::Vector3::Zero;
  math::Quaternion rotation_diff_quat = this->link_pose_targets_.at(link).rot * link->GetWorldPose().rot.GetInverse();

  math::Vector3 rotation_diff_axis;
  double rotation_diff_angle;
  rotation_diff_quat.GetAsAxis(rotation_diff_axis, rotation_diff_angle);
  if (rotation_diff_angle > this->link_rot_target_diff_threshold_)
  {
    angular_velocity = this->link_angular_velocity_speed_factor_ * rotation_diff_axis * rotation_diff_angle;
  }
  link->SetAngularVel(angular_velocity);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AvatarControlPlugin)

} // namespace gazebo

