#include <gazebo/util/LogRecord.hh>

#include "gazebo_ros_recording_plugin.h"

namespace gazebo
{

void GazeboRosRecordingPlugin::Init()
{
  // parent base setup
  GazeboRosReplayPlugin::Init();

  // handle recording commands
  this->_startService = this->_nh->advertiseService("start",
                                                    &GazeboRosRecordingPlugin::onStart,
                                                    this);
  this->_stopService = this->_nh->advertiseService("stop",
                                                   &GazeboRosRecordingPlugin::onStop,
                                                   this);
  this->_cancelService = this->_nh->advertiseService("cancel",
                                                     &GazeboRosRecordingPlugin::onCancel,
                                                     this);
  this->_cleanupService = this->_nh->advertiseService("cleanup",
                                                      &GazeboRosRecordingPlugin::onCleanup,
                                                      this);

  // request for recording state
  this->_stateService = this->_nh->advertiseService("get_recording",
                                                     &GazeboRosRecordingPlugin::isRecording,
                                                     this);

  // provide a message in the logs
  ROS_INFO("Gazebo ROS Recording Plugin initialized.");
}

void GazeboRosRecordingPlugin::onGazeboLoaded(std::string world_name)
{
  // parent base setup
  GazeboRosReplayPlugin::onGazeboLoaded(world_name);

  // listen for simulation time reset events
  auto reset_cb = std::bind(&GazeboRosRecordingPlugin::onReset, this);
  this->_timeResetConnection = event::Events::ConnectTimeReset(reset_cb);

  // Gazebo publisher for controlling logging
  this->_logControlPublisher = this->_gz_nh->Advertise<gazebo::msgs::LogControl>("~/log/control");
}

bool GazeboRosRecordingPlugin::onStart(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if already recording report an error
  if (this->_isRecording)
  {
    res.message = "Recording is already in progress!";
    res.success = false;
    return true;
  }

  // pause the world to start both recorders at the same time
  bool is_paused = this->_world->IsPaused();
  this->_world->SetPaused(true);

  // catch any filesystem issues (significant failure) and notify caller
  try
  {
    // create a temp directory if this is our first start/after reset
    if (this->_tmpDir.empty())
    {
      this->_tmpDir = boost::filesystem::temp_directory_path();
      this->_tmpDir /= boost::filesystem::unique_path();
    }

    // increment recording count
    ++this->_recordCount;
    std::string count_str = std::to_string(this->_recordCount);

    // generate target directories for gzserver/ros recordings
    std::string gzserver_dir = (this->_tmpDir / "gzserver" / count_str).native();
    std::string rosbag_dir = (this->_tmpDir / "ros").native();

    // start gzserver recording (inspired by gazebo/tools/gz_log.cc)
    gazebo::msgs::LogControl msg;
    msg.set_base_path(gzserver_dir);
    msg.set_start(true);
    this->_logControlPublisher->Publish<gazebo::msgs::LogControl>(msg, true);

    // configure ROS recording (all topics compressed to dest silently), this is a patched
    // version of RecorderOptions and Recorder in this repository
    rosbag::RecorderOptions opts;
    opts.record_all = true;
    opts.quiet = true;
    opts.append_date = false;
    opts.compression = rosbag::compression::BZ2;
    opts.path = rosbag_dir;
    opts.prefix = count_str;
    opts.do_exclude = true;
    opts.exclude_regex = "/gazebo/(.*)|/clock|/ros_cle_simulation/(.*)|/rosout(.*)|";

    // start the ROS recorder loop in a thread as it is blocking
    this->_rosbagPtr.reset(new rosbag::Recorder(opts));
    auto rosbag_run = boost::bind(&rosbag::Recorder::run, this->_rosbagPtr);
    this->_rosbagThreadPtr.reset(new boost::thread(rosbag_run));

    // indicate success in starting recording
    this->_isRecording = true;
    res.success = true;
  }
  catch(const boost::filesystem::filesystem_error& e)
  {
    res.message = e.code().message();
    res.success = false;
  }

  // restore the world to its previous paused/running state
  this->_world->SetPaused(is_paused);

  return true;
}

bool GazeboRosRecordingPlugin::onStop(std_srvs::Trigger::Request &req,
                                      std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if not recording report an error
  if (!this->_isRecording)
  {
    res.message = "Recording is not in progress!";
    res.success = false;
    return true;
  }

  // pause the world to start both recorders at the same time
  bool is_paused = this->_world->IsPaused();
  this->_world->SetPaused(true);

  // stop rosbag recorder thread "gracefully" and release resources
  this->_rosbagPtr->stop();
  this->_rosbagThreadPtr->join();
  this->_rosbagThreadPtr.reset();
  this->_rosbagPtr.reset();

  // stop gzserver recording
  gazebo::msgs::LogControl msg;
  msg.set_stop(true);
  this->_logControlPublisher->Publish<gazebo::msgs::LogControl>(msg, true);

  // since we haven't patched Gazebo recording, make the generated path more logical
  // move from the nested <num>/<date>/gzserver/state.log to just <num>.log
  try
  {
    // the recorded logs will start at the <num> dir
    std::string count_str = std::to_string(this->_recordCount);
    boost::filesystem::path gz_base = this->_tmpDir / "gzserver" / count_str;
    boost::filesystem::path gz_target = this->_tmpDir / "gzserver" / (count_str + ".log");

    // do some magic to get the timestamp subdir, rest of the path is fixed
    boost::filesystem::path gz_source = boost::filesystem::directory_iterator(gz_base)->path();
    gz_source = gz_source / "gzserver" / "state.log";

    // wait for the file to be completely written (before shutdown/moving it)
    while (!gazebo::util::LogRecord::Instance()->IsReadyToStart())
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));

    // copy the source to our target and deleted the nested dirs
    boost::filesystem::rename(gz_source, gz_target);
    boost::filesystem::remove_all(gz_base);

    // mark recording as successfully stopped
    this->_isRecording = false;
    res.success = true;
  }
  catch(const boost::filesystem::filesystem_error& e)
  {
    res.message = e.code().message();
    res.success = false;
  }

  // restore the world to its previous paused/running state
  this->_world->SetPaused(is_paused);

  return true;
}

bool GazeboRosRecordingPlugin::onCancel(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // stop the recording first and ensure it succeeded, else propagate the error
  this->onStop(req, res);
  if (!res.success)
    return true;

  // try to delete the recorded files, propagate any filesystem errors (critical)
  try
  {
    // file paths for the recorded files
    std::string count_str = std::to_string(this->_recordCount);
    boost::filesystem::path gz_file = this->_tmpDir / "gzserver" / (count_str + ".log");
    boost::filesystem::path ros_file = this->_tmpDir / "ros" / (count_str + ".bag");

    // delete the files (safe to call if files don't exist)
    boost::filesystem::remove(gz_file);
    boost::filesystem::remove(ros_file);

    // decrement the recording index for the next set of files
    --this->_recordCount;
  }
  catch(const boost::filesystem::filesystem_error& e)
  {
    res.message = e.code().message();
    res.success = false;
  }

  return true;
}

void GazeboRosRecordingPlugin::onReset()
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // same behavior as cleanup, destroy any recorded files, ignore return value
  std_srvs::Trigger trigger;
  this->onCleanup(trigger.request, trigger.response);
}

bool GazeboRosRecordingPlugin::onCleanup(std_srvs::Trigger::Request &req,
                                         std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // try to stop the current recording (if in progress)
  if (this->_isRecording) {

    // propagate any critical filesystem errors (won't be able to cleanup)
    this->onStop(req, res);
    if (!res.success)
      return true;
  }

  // recursively delete the temp dir and recorded contents, report failure
  try
  {
    boost::filesystem::remove_all(this->_tmpDir);
    this->_tmpDir.clear();

    res.success = true;
  }
  catch(const boost::filesystem::filesystem_error& e)
  {
    res.message = e.code().message();
    res.success = false;
  }

  // reset the recording index for the new recording directory
  this->_recordCount = 0;

  return true;
}

bool GazeboRosRecordingPlugin::isRecording(std_srvs::Trigger::Request &req,
                                           std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // return the path (or empty if none) and recording state
  res.message = this->_tmpDir.native();
  res.success = this->_isRecording;

  return true;
}

// register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosRecordingPlugin)

} // namespace gazebo
