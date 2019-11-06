#ifndef __GAZEBO_ROS_REPLAY_PLUGIN_HH__
#define __GAZEBO_ROS_REPLAY_PLUGIN_HH__

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>

#include <std_srvs/Trigger.h>

namespace gazebo
{

/// \brief A base virtual implementation for the recording and playback plugins.
class GazeboRosReplayPlugin : public SystemPlugin
{

public:

  /// \brief Destructor
  virtual ~GazeboRosReplayPlugin()
  {
    // disconnect all listener events
    if (this->_gazeboLoadedConnection)
    {
       this->_gazeboLoadedConnection.reset();
    }

    // disconnect the node handles and shutdown any advertised services/topics
    if (this->_nh)
    {
      this->_nh->shutdown();
      this->_nh.reset();
    }
    if (this->_gz_nh)
    {
      this->_gz_nh->Fini();
      this->_gz_nh.reset();
    }
  }

  /// \brief Load must be implemented, but we do everything in Init() instead
  ///        because GazeboRosApiPlugin::Load() guarantees ROS initialization.
  virtual void Load(int /*_argc*/, char** /*_argv*/) {};

  /// \brief Initialize ROS services and schedule Gazebo service init.
  virtual void Init()
  {
    // initialize ROS node handle on /gazebo/<type>
    this->_nh.reset(new ros::NodeHandle("~/" + this->replayType()));

    // schedule Gazebo interaction for after Gazebo initialization
    auto loaded_cb = boost::bind(&GazeboRosReplayPlugin::onGazeboLoaded, this, _1);
    this->_gazeboLoadedConnection = event::Events::ConnectWorldCreated(loaded_cb);
  }

  /// \brief Initialize Gazebo interfaces after the world is created.
  virtual void onGazeboLoaded(std::string world_name)
  {
    // initialize the Gazebo node handle after Gazebo is launched
    this->_gz_nh.reset(new transport::Node());
    this->_gz_nh->Init(world_name);

    // store a reference to the gazebo world
    this->_world = gazebo::physics::get_world(world_name);
  }

protected:

  /// \brief The type of recorder plugin loaded, implementation specific.
  virtual std::string replayType() const = 0;

  /// \brief ROS NodeHandle
  ros::NodeHandlePtr _nh;

  /// \brief Gazebo NodeHandle
  gazebo::transport::NodePtr _gz_nh;

  /// \brief Interface to the Gazebo world.
  gazebo::physics::WorldPtr _world;

  /// \brief Mutex to protect concurrent access while allocating/releasing.
  boost::recursive_mutex _mutex;

private:

  /// \brief Notification that Gazebo has been loaded.
  event::ConnectionPtr _gazeboLoadedConnection;

}; // class GazeboRosReplayPlugin

} // namespace gazebo

#endif
