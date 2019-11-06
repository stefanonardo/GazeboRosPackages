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

  // thread to handle change subscriptions below
  auto changes_run = boost::bind(&GazeboRosRecordingPlugin::processChanges, this);
  this->_changeThreadPtr.reset(new boost::thread(changes_run));

  // Gazebo subscribers for user light and material changes
  this->_materialSubscriber = this->_gz_nh->Subscribe("~/recording/model/modify",
                                                      &GazeboRosRecordingPlugin::onModelChange,
                                                      this);
  this->_lightSubscriber = this->_gz_nh->Subscribe("~/recording/light/modify",
                                                   &GazeboRosRecordingPlugin::onLightChange,
                                                   this);
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

  // before starting, make sure the light SDF is up to date, this is not done
  // automatically even as visuals are updated
  for(auto light : this->_world->Lights())
  {
    gazebo::msgs::Light msg;
    light->FillMsg(msg);
    light->UpdateParameters(gazebo::msgs::LightToSDF(msg));
  }
  this->_world->UpdateStateSDF();

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

    // ignore ROS/CLE lifecycle and error publishers, whitelist logs for pop up messages
    opts.exclude_regex = "/ros_cle_simulation/(?!logs).*|/rosout(.*)";

    // allowable gazebo types, all others published by Gazebo or its plugins are ignored
    opts.gazebo_type_whitelist.push_back("sensor_msgs/JointState");

    // the rate at which gazebo topics will be recorded (in seconds)
    opts.gazebo_rate_limit = ros::Duration(1.0 / float(GZ_NRP_LOG_RECORD_RATE));

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
  gazebo::util::LogRecord::Instance()->Stop();

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

void GazeboRosRecordingPlugin::onModelChange(const boost::shared_ptr<gazebo::msgs::Model const> &msg)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if not recording, ignore the material change
  if(!this->_isRecording)
    return;

  // spawn a thread to wait for the result, we can't block here since Gazebo will be blocked
  boost::unique_lock<boost::recursive_mutex> clock(this->_changeMutex);
  auto wait_run = boost::bind(&GazeboRosRecordingPlugin::waitForChange, this, msg, nullptr);
  boost::shared_ptr<boost::thread> threadPtr;
  threadPtr.reset(new boost::thread(wait_run));
  this->_changeThreads.push(std::make_pair(this->_world->SimTime(), threadPtr));
}

void GazeboRosRecordingPlugin::onLightChange(const boost::shared_ptr<gazebo::msgs::Light const> &msg)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if not recording, ignore the light change
  if(!this->_isRecording)
    return;

  // spawn a thread to wait for the result, we can't block here since Gazebo will be blocked
  boost::unique_lock<boost::recursive_mutex> clock(this->_changeMutex);
  auto wait_run = boost::bind(&GazeboRosRecordingPlugin::waitForChange, this, nullptr, msg);
  boost::shared_ptr<boost::thread> threadPtr;
  threadPtr.reset(new boost::thread(wait_run));
  this->_changeThreads.push(std::make_pair(this->_world->SimTime(), threadPtr));
}

void GazeboRosRecordingPlugin::processChanges()
{
  // run in the background until Gazebo is terminated
  while(this->_nh->ok())
  {
    // idle wait while no changes to process
    bool empty = true;
    while(empty)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
      {
        boost::unique_lock<boost::recursive_mutex> lock(this->_changeMutex);
        empty = _changeThreads.empty();
      }
    }

    // compute a time window to wait for messages (250 miliseconds)
    gazebo::common::Time end(0, 250 * 1e6);
    {
       boost::unique_lock<boost::recursive_mutex> lock(this->_changeMutex);
       end += _changeThreads.front().first;
    }

    // process the queue and wait until we hit the window
    bool ok = true;
    empty = false;
    while(ok)
    {
      std::pair<gazebo::common::Time, boost::shared_ptr<boost::thread>> current;
      {
        // check that the queue has items to process
        boost::unique_lock<boost::recursive_mutex> lock(this->_changeMutex);
        empty = _changeThreads.empty();
        if(!empty)
        {
          // check that we want to process the next item
          current = _changeThreads.front();
          if(current.first > end)
          {
            ok = false;
            break;
          }

          // remove the first item
          _changeThreads.pop();
        }
      }

      // if empty, wait for more events until the timeout window expires
      if(empty)
      {
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        ok = this->_world->SimTime() < end;
      }

      // otherwise, wait for the thread to complete (this won't hang as it is limited in the thread)
      else
        current.second->join();
    }

    // toggle recording to save the state changes
    {
      boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

      // if the user stopped recording in our wait cycle, ignore the changes as they'll be logged later
      if(!this->_isRecording)
        continue;

      // pause the world to let it process messages
      bool is_paused = this->_world->IsPaused();
      this->_world->SetPaused(true);

      // wait a short period for the world to update, playback will be slightly delayed but there's no
      // way to check this safely - 250ms is probably 1000x longer than needed but this is ok until a
      // better solution is required
      this->_world->Step(1);
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
      this->_world->UpdateStateSDF();

      // stop recording and restart with the new world state, reuse the logic for between files while is
      // much safer than trying to force it into replay (lots of race conditions in Gazebo playback)
      std_srvs::Trigger trigger;
      this->onStop(trigger.request, trigger.response);
      this->onStart(trigger.request, trigger.response);

      // restore the world state
      this->_world->SetPaused(is_paused);
    }
  }
}

void GazeboRosRecordingPlugin::waitForChange(const boost::shared_ptr<gazebo::msgs::Model const> &modelMsg,
                                             const boost::shared_ptr<gazebo::msgs::Light const> &lightMsg)
{
  // wait for a maximum time to ensure we don't just hang forever
  int timeout = 5 * 1000; // 5 seconds
  int elapsed = 0;
  bool done = false;

  while((!done) && (elapsed < timeout)) {

    // wait for the model to reflect the change request
    if(modelMsg)
    {
      auto model = this->_world->ModelByName(modelMsg->name());
      gazebo::msgs::Model updated;
      updated.Clear();
      model->FillMsg(updated);

      // Modified from gazebo_ros_api, this is needed for the NRP messages for some reason
      // check the individual link visuals as these are all that can be changed
      done = false;
      if(updated.link_size() == modelMsg->link_size()) {

        // look at all link individually
        done = true;
        for (unsigned int i = 0; i < updated.link_size(); i++) {
          auto msg_link = modelMsg->link().Get(i);
          auto updated_link = updated.link().Get(i);

          // make sure we have the same number of visuals as well
          if(msg_link.visual_size() != updated_link.visual_size()) {
            done = false;
            continue;
          }

          // compare each link visual, abort if there are differences
          for(unsigned int j = 0; j < updated_link.visual().size(); j++) {
            auto msg_visual = msg_link.visual().Get(i);
            auto updated_visual = updated_link.visual().Get(i);

            // check that the visual materials exist
            if(msg_visual.has_material() != updated_visual.has_material()) {
              done = false;
              continue;
            }

            // check that the materials are the same
            if(msg_visual.material().DebugString() != updated_visual.material().DebugString()) {
              done = false;
              continue;
            }
          }
        }
      }
    }

    // wait for the light to relfect the change request
    else
    {
      // create a mustable light to remove the pose for comparison
      gazebo::msgs::Light target = *lightMsg;
      if(target.has_pose())
        target.mutable_pose()->Clear();

      // get the current light state with no pose
      auto light = this->_world->LightByName(lightMsg->name());
      gazebo::msgs::Light updated;
      updated.Clear();
      light->FillMsg(updated);
      if(updated.has_pose())
        updated.mutable_pose()->Clear();

      // perform the actual check of light parameters
      done = (target.DebugString() == updated.DebugString());
      if(done)
      {
        // unfortunately the SDF isn't updated by default, force it here
        light->FillMsg(updated);
        light->UpdateParameters(gazebo::msgs::LightToSDF(updated));
      }
    }

    // don't spin at 100% cpu in between, even with a tiny sleep
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    elapsed += 10;
  }

  // we didn't see the actual change, log it and exit (Gazbeo may be broken somewhere)
  if(elapsed == timeout)
  {
    if(modelMsg)
      ROS_ERROR("GazeboRosRecordingPlugin: model change was not seen for model %s", modelMsg->name().c_str());
    else
      ROS_ERROR("GazeboRosRecordingPlugin: light change was not seen for light %s", lightMsg->name().c_str());
  }
}

// register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosRecordingPlugin)

} // namespace gazebo
