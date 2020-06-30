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

    this->model_target_rotation_ = ignition::math::Quaterniond::Zero;
    this->root_link_ = nullptr;
    this->rotation_velocity_speed_factor_ = 10.0f;

    this->link_linear_velocity_speed_factor_ = 10.0f;
    this->link_angular_velocity_speed_factor_ = 10.0f;
    this->link_pos_target_diff_threshold_ = 0.1f;
    this->link_rot_target_diff_threshold_ = 0.1f;

    this->keep_model_on_ground_ = false;
    this->step_height_ = 0.1f;

    this->model_target_pose_.reset(new ignition::math::Pose3d());

    ROS_INFO("sdf name %s, sdf description %s", sdf->GetName().c_str(), sdf->GetDescription().c_str());

    // parse through all controllers and create model and link controllers
    this->ParseControllers(sdf);

    // Controller time control.
    this->last_update_time_ = this->model_->GetWorld()->SimTime();

    // Listen to the update event. This event is broadcast every simulation iteration.
    eventconnection_update_world_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&AvatarControlPlugin::OnUpdate, this, _1));
  }

  // Called by the world update start event
  void AvatarControlPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    gazebo::common::Time curTime = this->model_->GetWorld()->SimTime();

    // update link linear velocities via link pose targets
    /*for (std::map<physics::LinkPtr, gazebo::math::Vector3>::iterator it = link_position_targets_.begin();
         it != link_position_targets_.end(); ++it) {
      this->UpdateLinkLinearVelocityFromPositionTarget(it->first);
    }
    for (std::map<physics::LinkPtr, gazebo::math::Quaternion>::iterator it = link_rotation_targets_.begin();
         it != link_rotation_targets_.end(); ++it) {
      this->UpdateLinkAngularVelocityFromRotationTarget(it->first);
    }*/

    this->last_update_time_ = curTime;

    if (this->joint_controller_)
    {
      this->joint_controller_->Update();

      // Add constant forces to joints (if there are any) ///////////////////////
      auto jointMap = this->joint_controller_->GetJoints();

      for (auto const &jointForce : constantJointForces)
      {
        auto jiter = jointMap.find(jointForce.first);

        ROS_INFO("In possesion of joint force");

        if (jiter != jointMap.end())
        {
          jiter->second->SetForce(0, jointForce.second);
        }
      }
    }

    this->UpdateModelVelocitiesFromModelPoseTarget();

    this->UpdateModelAngularVelocityFromModelRotationTarget();

    this->UpdateModelPositionOnGround();
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
      else if (controller_type == "link_velocities")
      {
        this->CreateControllersLinkVelocities(sdf_elem_controller);
      }
      else if (controller_type == "link_pose")
      {
        this->CreateControllerLinkPoseTarget(sdf_elem_controller);
      }
      else if (controller_type == "all_joints_set_position")
      {
        this->CreateControllerAllJointsSetPosition(sdf_elem_controller);
      }
      else if (controller_type == "all_joints_pid_position_target")
      {
        this->CreateControllerAllJointsPIDPositionTarget(sdf_elem_controller);
      }
      else if (controller_type == "model_pose_target")
      {
        this->CreateControllersModelPoseTarget(sdf_elem_controller);
      }
      else if (controller_type == "constant_joint_forces")
      {
        this->CreateControllerConstantJointForces(sdf_elem_controller);
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
    std::string topic_name = "/" + model_->GetName() + "/cmd_rot";

    model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Quaternion");

    // Add ROS topic for velocity control
    subscribers_model_rotation_.push_back(
        node_handle_.subscribe<geometry_msgs::Quaternion>(
            topic_name, 1, boost::bind(&AvatarControlPlugin::ModelRotationCB, this, _1)));

    ROS_INFO("Added new rotation controller for model %s on topic %s", model_->GetName().c_str(), topic_name.c_str());
  }

  void AvatarControlPlugin::CreateControllerLinkPoseTarget(const sdf::ElementPtr &sdf_elem_controller)
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

    // topic: pose target
    std::string topic_name = model_->GetName() + "/" + link_name + "/target_pose";

    model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Pose");

    subscribers_link_pose_.push_back(
        node_handle_.subscribe<geometry_msgs::Pose>(
            topic_name, 1, boost::bind(&AvatarControlPlugin::LinkPoseTargetCB, this, _1, link)));

    ROS_INFO("Added new pose controller for link %s on topic %s", link->GetName().c_str(), topic_name.c_str());
  }

  void AvatarControlPlugin::CreateControllersLinkVelocities(const sdf::ElementPtr &sdf_elem_controller)
  {
    std::vector<physics::LinkPtr> links = model_->GetLinks();
    for (auto link : links)
    {
      std::string link_name = link->GetName();
      std::string string_old = "::";
      std::string string_new = "/";
      std::string::size_type pos = 0u;
      while ((pos = link_name.find(string_old, pos)) != std::string::npos)
      {
        link_name.replace(pos, string_old.length(), string_new);
        pos += string_new.length();
      }

      std::string linear_topic_name = "/" + model_->GetName() + "/" + link_name + "/linear_vel";
      model_->SaveControllerActuatorRosTopics(linear_topic_name, "geometry_msgs/Vector3");
      // Add ROS topic for velocity control
      subscribers_link_linear_velocity_.push_back(
          node_handle_.subscribe<geometry_msgs::Vector3>(
              linear_topic_name, 1, boost::bind(&AvatarControlPlugin::LinkLinearVelocityCB, this, _1, link)));
      ROS_DEBUG("Added new linear velocity controller for link %s on topic %s", link->GetName().c_str(),
                linear_topic_name.c_str());

      std::string angular_topic_name = "/" + model_->GetName() + "/" + link_name + "/angular_vel";
      model_->SaveControllerActuatorRosTopics(angular_topic_name, "geometry_msgs/Vector3");
      // Add ROS topic for velocity control
      subscribers_link_angular_velocity_.push_back(
          node_handle_.subscribe<geometry_msgs::Vector3>(
              angular_topic_name, 1, boost::bind(&AvatarControlPlugin::LinkAngularVelocityCB, this, _1, link)));
      ROS_DEBUG("Added new angular velocity controller for link %s on topic %s", link->GetName().c_str(),
                angular_topic_name.c_str());
    }
  }

  void AvatarControlPlugin::CreateControllerAllJointsSetPosition(const sdf::ElementPtr &sdf_elem_controller)
  {
    std::vector<physics::JointPtr> joints = model_->GetJoints();
    for (auto joint : joints)
    {
      // create topic
      std::string joint_name = joint->GetName();
      std::string string_old = "::";
      std::string string_new = "/";
      std::string::size_type pos = 0u;
      while ((pos = joint_name.find(string_old, pos)) != std::string::npos)
      {
        joint_name.replace(pos, string_old.length(), string_new);
        pos += string_new.length();
      }
      std::string topic_name = "/" + model_->GetName() + "/" + joint_name + "/set_position";
      model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Vector3");
      subscribers_joint_set_position_.push_back(
          node_handle_.subscribe<geometry_msgs::Vector3>(
              topic_name, 1, boost::bind(&AvatarControlPlugin::JointSetPositionCB, this, _1, joint)));

      //ROS_INFO("Added new position controller for joint %s on topic %s", joint->GetName().c_str(), topic_name.c_str());
    }
  }

  void AvatarControlPlugin::CreateControllerAllJointsPIDPositionTarget(const sdf::ElementPtr &sdf_elem_controller)
  {
    this->joint_controller_.reset(new physics::JointController(
        this->model_));

    std::vector<physics::JointPtr> joints = model_->GetJoints();
    for (auto joint : joints)
    {
      this->joint_controller_->AddJoint(joint);
      std::string scoped_name = joint->GetScopedName();
      this->joint_controller_->SetPositionPID(scoped_name, common::PID(100, 50, 10));

      // create topic
      std::string joint_name = joint->GetName();
      std::string string_old = "::";
      std::string string_new = "/";
      std::string::size_type pos = 0u;
      while ((pos = joint_name.find(string_old, pos)) != std::string::npos)
      {
        joint_name.replace(pos, string_old.length(), string_new);
        pos += string_new.length();
      }
      // set position target topic
      /*std::string topic_name = "/" + model_->GetName() + "/" + joint_name + "/set_pid_position_target";
      model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Vector3");
      subscribers_joint_pid_position_target_.push_back(
        node_handle_.subscribe<geometry_msgs::Vector3>(
          topic_name, 1, boost::bind(&AvatarControlPlugin::JointPIDPositionTargetCB, this, _1, joint)
        )
      );*/
      // set PID parameters topic
      std::string topic_name_params = "/" + model_->GetName() + "/" + joint_name + "/set_pid_params";
      model_->SaveControllerActuatorRosTopics(topic_name_params, "geometry_msgs/Vector3");
      subscribers_joint_pid_params_.push_back(
          node_handle_.subscribe<geometry_msgs::Vector3>(
              topic_name_params, 1, boost::bind(&AvatarControlPlugin::JointPIDParamCB, this, _1, joint)));

      //ROS_INFO("Added new position target PID-controller for joint %s on topic %s", joint->GetName().c_str(), topic_name.c_str());
    }

    // PID target position topic via joint states message
    std::string topic_name_joint_states = "/" + model_->GetName() + "/set_joint_pid_pos_targets";
    model_->SaveControllerActuatorRosTopics(topic_name_joint_states, "gazebo_msgs/JointStates");
    subscribers_joint_pid_position_target_.push_back(
        node_handle_.subscribe<gazebo_msgs::JointStates>(
            topic_name_joint_states, 1, boost::bind(&AvatarControlPlugin::JointStatesPIDPositionTargetCB, this, _1)));
  }

  void AvatarControlPlugin::CreateControllersModelPoseTarget(const sdf::ElementPtr &sdf_elem_controller)
  {
    sdf::ElementPtr root_link_elem = sdf_elem_controller->GetElement("root_link");
    std::string sdf_root_link_name = "";
    if (root_link_elem != NULL)
    {
      sdf_root_link_name = root_link_elem->Get<std::string>();
    }
    else
    {
      ROS_WARN("ModelPoseTarget controller has no element <root_link>");
      return;
    }
    this->root_link_ = this->model_->GetLink(sdf_root_link_name);

    std::string topic_name_pose_target = "/" + model_->GetName() + "/set_pose_target";
    model_->SaveControllerActuatorRosTopics(topic_name_pose_target, "geometry_msgs/Pose");
    subscribers_model_pose_target_.push_back(
        node_handle_.subscribe<geometry_msgs::Pose>(
            topic_name_pose_target, 1, boost::bind(&AvatarControlPlugin::ModelPoseTargetCB, this, _1)));
    ROS_INFO("Added new model pose target controller for %s", model_->GetName().c_str());
  }

  void AvatarControlPlugin::CreateControllerConstantJointForces(const sdf::ElementPtr &sdf_elem_controller)
  {
    std::vector<physics::JointPtr> joints = model_->GetJoints();
    for (auto joint : joints)
    {
      // create topic
      std::string joint_name = joint->GetName();
      std::string string_old = "::";
      std::string string_new = "/";
      std::string::size_type pos = 0u;
      while ((pos = joint_name.find(string_old, pos)) != std::string::npos)
      {
        joint_name.replace(pos, string_old.length(), string_new);
        pos += string_new.length();
      }

      std::string topic_name = "/" + model_->GetName() + "/" + joint_name + "/set_constant_force";
      model_->SaveControllerActuatorRosTopics(topic_name, "geometry_msgs/Vector3");
      subscribers_constant_joint_force_.push_back(
          node_handle_.subscribe<geometry_msgs::Vector3>(
              topic_name, 1, boost::bind(&AvatarControlPlugin::ConstantJointForceCB, this, _1, joint)));
    }
  }

  //////////////////////////////////////// ROS topic callback functions //////////////////////////////////////////

  void AvatarControlPlugin::ModelRotationCB(const geometry_msgs::Quaternion::ConstPtr &msg)
  {
    ROS_DEBUG("ModelRotationCB called! quaternion(xyzw) = %.2f %.2f %.2f %.2f", msg->x, msg->y, msg->z, msg->w);

    this->model_target_rotation_.Set(msg->w, msg->x, msg->y, msg->z);
  }

  void AvatarControlPlugin::LinkPoseTargetCB(const geometry_msgs::Pose::ConstPtr &msg, const physics::LinkPtr &link)
  {
    ROS_DEBUG("LinkPoseTargetCB called! link name = %s, link pos = %.2f %.2f %.2f, link rot = %.2f %.2f %.2f %.2f",
              link->GetName().c_str(),
              msg->position.x, msg->position.y, msg->position.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    this->SetLinkPositionTarget(msg->position.x, msg->position.y, msg->position.z, link);
    this->SetLinkRotationTarget(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z, link);
  }

  void
  AvatarControlPlugin::LinkPositionTargetCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link)
  {
    ROS_DEBUG("LinkPositionTargetCB called! link name = %s, link pos = %.2f %.2f %.2f",
              link->GetName().c_str(),
              msg->x, msg->y, msg->z);

    this->SetLinkPositionTarget(msg->x, msg->y, msg->z, link);
  }

  void AvatarControlPlugin::LinkRotationTargetCB(const geometry_msgs::Quaternion::ConstPtr &msg,
                                                 const physics::LinkPtr &link)
  {
    ROS_DEBUG("LinkRotationTargetCB called! link name = %s, link rot = %.2f %.2f %.2f %.2f",
              link->GetName().c_str(),
              msg->w, msg->x, msg->y, msg->z);

    this->SetLinkRotationTarget(msg->w, msg->x, msg->y, msg->z, link);
  }

  void
  AvatarControlPlugin::LinkLinearVelocityCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link)
  {
    ROS_DEBUG("LinkLinearVelocityCB called! link name = %s, vel = %.2f %.2f %.2f ",
              link->GetName().c_str(), msg->x, msg->y, msg->z);
    ignition::math::Vector3d velocity(msg->x, msg->y, msg->z);
    link->SetLinearVel(velocity);
  }

  void AvatarControlPlugin::LinkAngularVelocityCB(const geometry_msgs::Vector3::ConstPtr &msg,
                                                  const physics::LinkPtr &link)
  {
    ROS_DEBUG("LinkAngularVelocityCB called! link name = %s, vel = %.2f %.2f %.2f ",
              link->GetName().c_str(), msg->x, msg->y, msg->z);
    ignition::math::Vector3d velocity(msg->x, msg->y, msg->z);
    link->SetAngularVel(velocity);
  }

  void AvatarControlPlugin::JointSetPositionCB(const geometry_msgs::Vector3::ConstPtr &msg,
                                               const physics::JointPtr &joint)
  {
    ROS_DEBUG("JointSetPositionCB called! joint name = %s, pos = %.2f %.2f %.2f ",
              joint->GetName().c_str(), msg->x, msg->y, msg->z);

    joint->SetPosition(0, msg->x);
    if (joint->DOF() >= 2)
    {
      joint->SetPosition(1, msg->y);
    }
    if (joint->DOF() >= 3)
    {
      joint->SetPosition(2, msg->z);
    }
  }

  void AvatarControlPlugin::JointPIDPositionTargetCB(const geometry_msgs::Vector3::ConstPtr &msg,
                                                     const physics::JointPtr &joint)
  {
    if (this->joint_controller_)
    {
      //ROS_INFO("JointPIDPositionTargetCB() - name = %s, scoped name = %s, pos target = %.2f %.2f %.2f ",
      //         joint->GetName().c_str(), joint->GetScopedName().c_str(), msg->x, msg->y, msg->z);
      this->joint_controller_->SetPositionTarget(joint->GetScopedName(), msg->x);
    }
  }

  void AvatarControlPlugin::JointStatesPIDPositionTargetCB(const gazebo_msgs::JointStates::ConstPtr &msg)
  {
    if (this->joint_controller_)
    {
      for (int i = 0; i < msg->name.size(); i = i + 1)
      {
        //ROS_INFO("JointStatesPIDPositionTargetCB() - %s - %f", msg->name.at(i).c_str(), msg->position.at(i));
        std::string joint_name = msg->name.at(i);
        physics::JointPtr joint = this->model_->GetJoint(joint_name);
        if (joint != nullptr)
        {
          //ROS_INFO("%s - %f", joint->GetScopedName().c_str(), msg->position.at(i));
          this->joint_controller_->SetPositionTarget(joint->GetScopedName(), msg->position.at(i));
        }
        else
        {
          ROS_INFO("JointStatesPIDPositionTargetCB - joint pointer == null");
        }
      }
    }
  }

  void AvatarControlPlugin::JointPIDParamCB(const geometry_msgs::Vector3::ConstPtr &msg,
                                            const physics::JointPtr &joint)
  {
    if (this->joint_controller_)
    {
      //ROS_INFO("JointPIDParamCB called! joint name = %s, params = %.2f %.2f %.2f ", joint->GetName().c_str(), msg->x, msg->y, msg->z);
      this->joint_controller_->SetPositionPID(joint->GetScopedName(), common::PID(msg->x, msg->y, msg->z));
    }
  }

  void AvatarControlPlugin::ModelPoseTargetCB(const geometry_msgs::Pose::ConstPtr &msg)
  {
    //ROS_INFO("ModelPoseTargetCB() - %.2f %.2f %.2f", msg->position.x, msg->position.y, msg->position.z);
    this->model_target_pose_->Set(
        ignition::math::Vector3d(msg->position.x, msg->position.y, msg->position.z),
        ignition::math::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
  }

  void AvatarControlPlugin::ConstantJointForceCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::JointPtr &joint)
  {
    if (msg->x == 0.0)
    {
      constantJointForces.erase(joint->GetScopedName());
      // ROS_INFO("Erased joint constant force");
    }
    else
    {
      constantJointForces[joint->GetScopedName()] = msg->x;
      // ROS_INFO("Added joint constant force: %s", joint->GetScopedName().c_str());
    }
  }

  //////////////////////////////////////// Update functions called during OnUpdate() ////////////////////////////////

  void AvatarControlPlugin::UpdateLinkLinearVelocityFromPositionTarget(const physics::LinkPtr &link)
  {
    ignition::math::Vector3d linear_velocity = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d position_diff = this->link_position_targets_.at(link) - link->WorldPose().Pos();

    if (position_diff.Length() > this->link_pos_target_diff_threshold_)
    {
      linear_velocity = this->link_linear_velocity_speed_factor_ * position_diff;
    }
    link->SetLinearVel(linear_velocity);
  }

  void AvatarControlPlugin::UpdateLinkAngularVelocityFromRotationTarget(const physics::LinkPtr &link)
  {
    ignition::math::Vector3d angular_velocity = ignition::math::Vector3d::Zero;
    ignition::math::Quaterniond rotation_diff_quat = this->link_rotation_targets_.at(link) * link->WorldPose().Rot().Inverse();

    ignition::math::Vector3d rotation_diff_axis;
    double rotation_diff_angle;
    rotation_diff_quat.Axis(rotation_diff_axis, rotation_diff_angle);
    if (rotation_diff_angle > this->link_rot_target_diff_threshold_)
    {
      angular_velocity = this->link_angular_velocity_speed_factor_ * rotation_diff_axis * rotation_diff_angle;
    }
    link->SetAngularVel(angular_velocity);
  }

  void AvatarControlPlugin::UpdateJointPositionFromPositionTarget(const physics::JointPtr &joint)
  {
  }

  void AvatarControlPlugin::UpdateModelVelocitiesFromModelPoseTarget()
  {
    //have to get the parent/root link, setting model velocities resets joint forces
    if (this->root_link_ != nullptr)
    {
      //ROS_INFO("UpdateModelVelocitiesFromModelPoseTarget() - %s", this->root_link_->GetName().c_str());
      ignition::math::Pose3d current_pose = this->model_->WorldPose();

      // linear velocity
      ignition::math::Vector3d linear_velocity = ignition::math::Vector3d::Zero;
      ignition::math::Vector3d pos_difference = this->model_target_pose_->Pos() - current_pose.Pos();
      if (pos_difference.Length() > 0.1f)
      {
        linear_velocity = pos_difference * 10.0f;
      }
      this->root_link_.get()->SetLinearVel(linear_velocity);

      // angular velocity
      ignition::math::Vector3d angular_velocity = ignition::math::Vector3d::Zero;
      ignition::math::Quaterniond rotation_difference = this->model_target_pose_->Rot() * current_pose.Rot().Inverse();
      if (rotation_difference.Euler().Length() > 0.1f)
      {
        angular_velocity = rotation_difference.Euler() * 10.0f;
      }
      this->root_link_->SetAngularVel(angular_velocity);
    }

    /*// linear velocity
    math::Vector3 linear_velocity = math::Vector3::Zero;
    math::Vector3 pos_difference = this->model_target_pose_->Pos() - current_pose.Pos();
    if (pos_difference.Length() > 0.1f) 
    {
      linear_velocity = pos_difference * 10.0f;
    }
    this->model_->SetLinearVel(linear_velocity);

    // angular velocity
    math::Vector3 angular_velocity = math::Vector3::Zero;
    math::Quaternion rotation_difference = this->model_target_pose_->Rot() * current_pose.Rot().Inverse();
    if (rotation_difference.Euler().Length() > 0.1f) 
    {
      angular_velocity = rotation_difference.Euler() * 10.0f;
    }
    this->model_->SetAngularVel(angular_velocity);*/
  }

  /**
   *  set angular velocity from target model rotation (this->model_target_rotation_)
   */
  void AvatarControlPlugin::UpdateModelAngularVelocityFromModelRotationTarget()
  {
    if (this->model_target_rotation_ == ignition::math::Quaterniond::Zero)
    {
      return;
    }

    ignition::math::Pose3d model_world_pose = this->model_->WorldPose();
    ignition::math::Quaterniond rotation_diff_quat = this->model_target_rotation_ * model_world_pose.Rot().Inverse();
    ROS_INFO("UpdateModelAngularVelocityFromModelRotationTarget - %.2f %.2f %.2f %.2f", rotation_diff_quat.X(), rotation_diff_quat.Y(), rotation_diff_quat.Z(), rotation_diff_quat.W());

    ignition::math::Vector3d rotation_diff_axis;
    double rotation_diff_angle;
    rotation_diff_quat.ToAxis(rotation_diff_axis, rotation_diff_angle);

    ignition::math::Vector3d model_angular_velocity = ignition::math::Vector3d::Zero;
    if (rotation_diff_angle > 0.1f)
    {
      model_angular_velocity = this->rotation_velocity_speed_factor_ * rotation_diff_axis * rotation_diff_angle;
    }
    this->model_->SetAngularVel(model_angular_velocity);
  }

  /**
   * keep model on ground if keep_model_on_ground_ is true
   */
  void AvatarControlPlugin::UpdateModelPositionOnGround()
  {
    ignition::math::Pose3d model_world_pose = this->model_->WorldPose();
    if (this->keep_model_on_ground_)
    {
      ignition::math::Vector3d up = ignition::math::Vector3d::UnitZ;
      ignition::math::Vector3d ground_ray_start = model_world_pose.Pos() + (up * this->step_height_);
      ignition::math::Vector3d ground_ray_end = model_world_pose.Pos() - (up * 100.0f);

      gazebo::physics::PhysicsEnginePtr engine = model_->GetWorld()->Physics();
      gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));
      ray->SetPoints(ground_ray_start, ground_ray_end);

      double distance = 0.0;
      std::string entity_name = "";
      ray->GetIntersection(distance, entity_name);
      if (!entity_name.empty())
      {
        ignition::math::Vector3d vec_to_ground = (ground_ray_start - (distance * up)) - model_world_pose.Pos();
        if (vec_to_ground.Length() > 0.05f)
        {
          this->model_->SetLinearVel(vec_to_ground * 30);
        }
      }
    }
  }

  //////////////////////////////////////// Helper functions ////////////////////////////////

  void AvatarControlPlugin::SetLinkPositionTarget(double x, double y, double z, const physics::LinkPtr &link)
  {
    auto target = link_position_targets_.find(link);
    if (target == link_position_targets_.end())
    {
      link_position_targets_.insert(std::make_pair(link, ignition::math::Vector3d::Zero));
    }

    link_position_targets_[link].Set(x, y, z);
  }

  void
  AvatarControlPlugin::SetLinkRotationTarget(double w, double x, double y, double z, const physics::LinkPtr &link)
  {
    auto target = link_rotation_targets_.find(link);
    if (target == link_rotation_targets_.end())
    {
      link_rotation_targets_.insert(std::make_pair(link, ignition::math::Quaterniond()));
    }

    link_rotation_targets_[link].Set(w, x, y, z);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AvatarControlPlugin)

} // namespace gazebo
