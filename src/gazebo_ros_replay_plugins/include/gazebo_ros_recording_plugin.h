#ifndef __GAZEBO_ROS_RECORDING_PLUGIN_HH__
#define __GAZEBO_ROS_RECORDING_PLUGIN_HH__

#include <rosbag/recorder.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include "gazebo_ros_replay_plugin.h"

namespace gazebo
{

/// \brief A plugin to enable coordinated Gazebo/ROS recording.
class GazeboRosRecordingPlugin : public GazeboRosReplayPlugin
{

public:

  /// \brief Constructor
  GazeboRosRecordingPlugin() : _isRecording(false),
                               _recordCount(0) {};

  /// \brief Destructor
  virtual ~GazeboRosRecordingPlugin()
  {
    // disconnect time reset listener events
    if (this->_timeResetConnection)
    {
      this->_timeResetConnection.reset();
    }
  }

  /// \brief Initialize ROS services and schedule Gazebo service init.
  virtual void Init();

  /// \brief Initialize Gazebo interfaces after the world is created.
  virtual void onGazeboLoaded(std::string world_name);

  /// \brief Start a recording session.
  bool onStart(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res);

  /// \brief Stop the current recording (if any).
  bool onStop(std_srvs::Trigger::Request &req,
              std_srvs::Trigger::Response &res);

  /// \brief Cancel the current recording (if any).
  bool onCancel(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res);

  /// \brief Handle simulation time (full) reset event by discarding any recorded files.
  virtual void onReset();

  /// \brief Cleanup event, delete any recorded files on local system.
  bool onCleanup(std_srvs::Trigger::Request &req,
                 std_srvs::Trigger::Response &res);

  /// \brief Status, indicate if recording is currently in progress.
  bool isRecording(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res);

  /// \brief Handler for user model material changes.
  void onModelChange(const boost::shared_ptr<gazebo::msgs::Model const> &msg);

  /// \brief Handler for user light changes.
  void onLightChange(const boost::shared_ptr<gazebo::msgs::Light const> &msg);

protected:

  /// \brief The type of recorder plugin loaded.
  virtual std::string replayType() const { return "recording"; }

private:

  /// \brief Monitor waiting threads and toggle recording when all changes are processed.
  void processChanges();

  /// \brief Wait for user driven changes off the main Gazebo thread.
  void waitForChange(const boost::shared_ptr<gazebo::msgs::Model const> &modelMsg,
                     const boost::shared_ptr<gazebo::msgs::Light const> &lightMsg);

  /// \brief State flag to indicate if we are recording.
  std::atomic<bool> _isRecording;

  /// \brief Total number of recorded segments, used for unique file paths.
  std::atomic<int> _recordCount;

  /// \brief The path to the recordings, empty if not set.
  boost::filesystem::path _tmpDir;

  /// \brief Service to handle a start request.
  ros::ServiceServer _startService;

  /// \brief Service to handle a stop request.
  ros::ServiceServer _stopService;

  /// \brief Service to handle a cancel request.
  ros::ServiceServer _cancelService;

  /// \brief Service to handle a cleanup request.
  ros::ServiceServer _cleanupService;

  /// \brief Service to handle a state request.
  ros::ServiceServer _stateService;

  /// \brief Simulation time reset handler.
  event::ConnectionPtr _timeResetConnection;

  /// \brief Gazebo publisher to logging interface.
  gazebo::transport::PublisherPtr _logControlPublisher;

  /// \brief Gazebo subscriber for user model material changes during recording.
  gazebo::transport::SubscriberPtr _materialSubscriber;

  /// \brief Gazebo subscriber for user light changes during recording.
  gazebo::transport::SubscriberPtr _lightSubscriber;

  /// \brief Rosbag recorder instance, source modified for the NRP.
  boost::shared_ptr<rosbag::Recorder> _rosbagPtr;

  /// \brief Rosbag thread for running record loop.
  boost::shared_ptr<boost::thread> _rosbagThreadPtr;

  /// \brief Thread for handling/monitoring model and light changes.
  boost::shared_ptr<boost::thread> _changeThreadPtr;

  /// \brief List of running threads monitoring model/light changes.
  std::queue<std::pair<gazebo::common::Time, boost::shared_ptr<boost::thread>>> _changeThreads;

  /// \brief Mutex to protect concurrent access while monitoring changes.
  boost::recursive_mutex _changeMutex;

}; // class GazeboRosRecordingPlugin

} // namespace gazebo

#endif
