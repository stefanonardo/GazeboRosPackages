#ifndef USER_CONTROLLER_PLUGIN_H
#define USER_CONTROLLER_PLUGIN_H

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

#include <ros/ros.h>

#include <mutex>

namespace gazebo
{

typedef std::map<std::string, physics::JointPtr> JointMap;

class UserControlPlugin : public ModelPlugin
{

public:

  UserControlPlugin();
  ~UserControlPlugin();

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
  // Create controller for link velocity
  void CreateControllerLinkVelocity(const sdf::ElementPtr &sdf_elem_controller);

  // Generic model rotation command callback function (ROS topic)
  void ModelRotationCB(const geometry_msgs::Quaternion::ConstPtr &msg);
  // Generic link velocity command callback function (ROS topic)
  void LinkVelocityCB(const geometry_msgs::Vector3::ConstPtr &msg, const physics::LinkPtr &link);

  // ROS node handle
  ros::NodeHandle node_handle_;

  // Pointer to the model
  physics::ModelPtr model_;

  // Pointer to the update event connection
  event::ConnectionPtr eventconnection_update_world_;

  // ROS subscribers
  std::vector<ros::Subscriber> subscribers_model_rotation_;
  std::vector<ros::Subscriber> subscribers_link_velocity_;

  /// \brief keep track of controller update sim-time.
  gazebo::common::Time last_update_time_;

  /// \brief Controller update mutex.
  std::mutex mutex_;
  
  // saves parent model rotation updates from topics (applied during OnUpdate() )
  gazebo::math::Quaternion model_target_rotation_;
  float rotation_velocity_speed_factor_;
  
  // whether to keep the parent model positioned on ground
  bool keep_model_on_ground_;
  // determines ability to move up when recalculating parent model position on ground
  float step_height_;

};

} // namespace gazebo

#endif
