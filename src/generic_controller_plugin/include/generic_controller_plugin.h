#ifndef GENERIC_CONTROLLER_PLUGIN_H
#define GENERIC_CONTROLLER_PLUGIN_H

#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>

#include <mutex>

// Version check for joint update message support: Restrict to specific Gazebo version 6.0.6 the EPFL repository version has 'forked' from
#if GAZEBO_MAJOR_VERSION == 6 && GAZEBO_MINOR_VERSION == 0 && GAZEBO_PATCH_VERSION == 6
#define GAZEBO_HBP_SUPPORT_JOINT_STATE_MESSAGES
#endif

namespace gazebo
{

typedef std::map<std::string, physics::JointPtr> JointMap;

class GenericControlPlugin : public ModelPlugin
{

public:

  GenericControlPlugin();
  ~GenericControlPlugin();

  // Load the plugin and initilize all controllers
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Simulation update callback function
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:

  // check if controller for current joint is specified in SDF and return pointer to sdf element
  bool existsControllerSDF(sdf::ElementPtr &sdf_ctrl_def, const sdf::ElementPtr &sdf,
                           const physics::JointPtr &joint);

#ifdef GAZEBO_HBP_SUPPORT_JOINT_STATE_MESSAGES
  // check if visual properties (for client-side animation of models) exist, and return a pointer to the SDF element
  bool existsVisualSDF(sdf::ElementPtr &sdf_visual_def, const sdf::ElementPtr& sdf,
                       const physics::JointPtr& joint);
#endif

  // get PID controller values from SDF
  common::PID getControllerPID(const sdf::ElementPtr &sdf_ctrl_def);

  // get controller type from SDF
  std::string getControllerType(const sdf::ElementPtr &sdf_ctrl_def);

  // Method for creating a position controller for a given joint
  void createPositionController(const physics::JointPtr &joint, const common::PID &pid_param);

  // Method for creating a velocity controller for a given joint
  void createVelocityController(const physics::JointPtr &joint, const common::PID &pid_param);

  // Generic position command callback function (ROS topic)
  void positionCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint);

  // Generic velocity command callback function (ROS topic)
  void velocityCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint);

  // ROS node handle
  ros::NodeHandle m_nh;

  // Pointer to the model
  physics::ModelPtr m_model;

  // Pointer to joint controllers
  physics::JointControllerPtr m_joint_controller;

  // Map of joint pointers
  JointMap m_joints;

  // Pointer to the update event connection
  event::ConnectionPtr m_updateConnection;

  // ROS subscriber for joint control values
  std::vector<ros::Subscriber> m_pos_sub_vec;
  std::vector<ros::Subscriber> m_vel_sub_vec;

#ifdef GAZEBO_HBP_SUPPORT_JOINT_STATE_MESSAGES
  // Maps for joint names and rotation axes (visual properties for client-side animation)
  std::map<std::string, std::string> m_joint_name_mappings;
  std::map<std::string, geometry_msgs::Vector3> m_joint_axis_mappings;
#endif

  /// \brief keep track of controller update sim-time.
  private: gazebo::common::Time lastControllerUpdateTime;

  /// \brief Controller update mutex.
  private: std::mutex mutex;

  // ROS joint state publisher
  private: ros::Publisher m_joint_state_pub;
  private: sensor_msgs::JointState m_js;

};

} // namespace gazebo

#endif
