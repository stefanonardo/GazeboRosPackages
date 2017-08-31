#ifndef __GAZEBO_ROS_PLAYBACK_PLUGIN_HH__
#define __GAZEBO_ROS_PLAYBACK_PLUGIN_HH__

#include "gazebo_ros_replay_plugin.h"

#include "rosbag/player.h"

#include <cle_ros_msgs/SimulationPlayback.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

namespace gazebo
{

/// \brief A plugin to enable coordinated Gazebo/ROS playback.
class GazeboRosPlaybackPlugin : public GazeboRosReplayPlugin
{

public:

  /// \brief Constructor
  GazeboRosPlaybackPlugin() : _isPlaying(false),
                              _logIndex(1) {};

  /// \brief Initialize ROS services and schedule Gazebo service init.
  virtual void Init();

  /// \brief Configure playback and load initial state.
  bool onConfigure(cle_ros_msgs::SimulationPlayback::Request &req,
                   cle_ros_msgs::SimulationPlayback::Response &res);

  /// \brief Start playback.
  bool onStart(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res);

  /// \brief Pause playback and maintain current position.
  bool onPause(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res);

  /// \brief Stop playback and discard current position.
  bool onStop(std_srvs::Trigger::Request &req,
              std_srvs::Trigger::Response &res);

  /// \brief Reset playback, stop current playback and given playback.
  bool onReset(cle_ros_msgs::SimulationPlayback::Request &req,
               cle_ros_msgs::SimulationPlayback::Response &res);

protected:

  /// \brief The type of replay plugin loaded.
  virtual std::string replayType() const { return "playback"; }

private:

  /// \brief Method to run threaded for playback.
  void playback();

  /// \brief Helper to set current playback file.
  bool setPlaybackIndex(int index);

  /// \brief Helper method to strip SDF plugins, not needed.
  void removeSDFPlugins(sdf::ElementPtr elem);

  /// \brief Rosbag thread for running record loop.
  boost::shared_ptr<boost::thread> _playbackThreadPtr;

  /// \brief State flag to indicate if we are playing.
  std::atomic<bool> _isPlaying;

  /// \brief State of current log file being played.
  std::atomic<int> _logIndex;

  /// \brief A deep copy of the current Gazebo log file open, if dereferenced Gazebo crashes.
  std::string _gzFileStr;

  /// \brief A deep copy of the current Gazebo raw SDF loaded, if dereferenced Gazebo crashes.
  std::string _sdfStr;

  /// \brief A reference to the world SDF loaded, if dereferenced Gazebo crashes.
  sdf::SDFPtr _sdfPtr;

  /// \brief The rosbag player instance.
  boost::shared_ptr<rosbag::Player> _rosbagPtr;

  /// \brief Rosbag thread for running player loop.
  boost::shared_ptr<boost::thread> _rosbagThreadPtr;

  /// \brief The path to the recordings, empty if not configured.
  boost::filesystem::path _recordingDir;

  /// \brief Service to handle a configuration request.
  ros::ServiceServer _configureService;

  /// \brief Service to handle a start request.
  ros::ServiceServer _startService;

  /// \brief Service to handle a pause request.
  ros::ServiceServer _pauseService;

  /// \brief Service to handle a stop request.
  ros::ServiceServer _stopService;

  /// \brief Service to handle a reset request.
  ros::ServiceServer _resetService;

}; // class GazeboRosPlaybackPlugin

} // namespace gazebo

#endif
