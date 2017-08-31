#include "gazebo_ros_playback_plugin.h"

#include <gazebo/util/LogPlay.hh>
#include <gazebo/physics/World.hh>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/unordered_map.hpp>

namespace gazebo
{

void GazeboRosPlaybackPlugin::Init()
{
  // parent base setup
  GazeboRosReplayPlugin::Init();

  // handle playing commands
  this->_configureService = this->_nh->advertiseService("configure",
                                                        &GazeboRosPlaybackPlugin::onConfigure,
                                                        this);
  this->_startService = this->_nh->advertiseService("start",
                                                    &GazeboRosPlaybackPlugin::onStart,
                                                    this);
  this->_pauseService = this->_nh->advertiseService("pause",
                                                    &GazeboRosPlaybackPlugin::onPause,
                                                    this);
  this->_stopService = this->_nh->advertiseService("stop",
                                                   &GazeboRosPlaybackPlugin::onStop,
                                                   this);
  this->_resetService = this->_nh->advertiseService("reset",
                                                    &GazeboRosPlaybackPlugin::onReset,
                                                    this);

  // provide a message in the logs
  ROS_INFO("Gazebo ROS Playback Plugin initialized.");
}

bool GazeboRosPlaybackPlugin::onConfigure(cle_ros_msgs::SimulationPlayback::Request &req,
                                          cle_ros_msgs::SimulationPlayback::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // player thread is running, cannot reliably reconfigure running gazebo
  if (!this->_recordingDir.empty())
  {
    res.message = "Playback is in progress, cannot reconfigure.";
    res.success = false;
    return true;
  }

  // try to validate the path and recording folders
  try
  {
    boost::filesystem::path dir(req.path);

    // check that the path exists and is a directory
    if(!boost::filesystem::is_directory(dir))
    {
      res.message = "Playback recording directory does not exist!";
      res.success = false;
    }

    // check that the path contains the required subfolders
    else
    {
      this->_recordingDir = dir;

      // playback configuration check files, if failed, abort
      if(!this->setPlaybackIndex(1))
      {
        res.message = "Playback configuration failed, aborting!";
        res.success = false;
      }

      // successfully configured, launch playback thread
      else
      {
        // spawn a thread for playback
        auto player = boost::bind(&GazeboRosPlaybackPlugin::playback, this);
        this->_playbackThreadPtr.reset(new boost::thread(player));

        // mark the configuration as successful
        res.success = true;
      }
    }
  }
  catch(const boost::filesystem::filesystem_error& e)
  {
    res.message = e.code().message();
    res.success = false;
  }

  return true;
}

bool GazeboRosPlaybackPlugin::onStart(std_srvs::Trigger::Request &req,
                                      std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if not configured report an error
  if (this->_recordingDir.empty())
  {
    res.message = "Playback is not configured!";
    res.success = false;
    return true;
  }

  // set the play flag for the running thread
  this->_isPlaying = true;

  res.success = true;
  return true;
}

bool GazeboRosPlaybackPlugin::onPause(std_srvs::Trigger::Request &req,
                                      std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if not configured report an error
  if (this->_recordingDir.empty())
  {
    res.message = "Playback is not configured!";
    res.success = false;
    return true;
  }

  // set the play flag for the running thread
  this->_isPlaying = false;

  res.success = true;
  return true;
}

bool GazeboRosPlaybackPlugin::onStop(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if not configured report an error
  if (this->_recordingDir.empty())
  {
    res.message = "Playback is not configured!";
    res.success = false;
    return true;
  }

  // clear running flags to terminate thread
  this->_isPlaying = false;
  this->_recordingDir.clear();

  // wait for the playback thread to terminate
  this->_playbackThreadPtr->join();
  this->_playbackThreadPtr.reset();

  res.success = true;
  return true;
}

bool GazeboRosPlaybackPlugin::onReset(cle_ros_msgs::SimulationPlayback::Request &req,
                                      cle_ros_msgs::SimulationPlayback::Response &res)
{
  // guard instance / state variables
  boost::unique_lock<boost::recursive_mutex> lock(this->_mutex);

  // if not configured, we can't reset
  if (this->_recordingDir.empty())
  {
    res.message = "Playback is not configured!";
    res.success = false;
    return true;
  }

  // if currently playing, set the state to paused as the frontend expects this
  if (this->_isPlaying)
  {
    std_srvs::Trigger trigger;
    this->onPause(trigger.request, trigger.response);
    if(!trigger.response.success)
    {
      res.message = trigger.response.message;
      res.success = false;
      return true;
    }
  }

  // reset the playback index and reset the log playback
  this->setPlaybackIndex(1);

  // reset the gazebo world to update the frontend display
  this->_world->Reset();

  // successfully reset
  res.success = true;
  return true;
}

void GazeboRosPlaybackPlugin::playback()
{
  // run until explicitly stopped / shutdown
  while(!this->_recordingDir.empty())
  {

    // match our time increment to the Gazebo log recording rate
    gazebo::common::Time dt = gazebo::common::Time(1, 0) / GZ_NRP_LOG_RECORD_RATE;

    // realtime playback rate in milliseconds based on the recording rate
    int dt_ms = floor(dt.Double() * 1000);

    // while playing, process ROS and gazbeo synchronized playback
    while(this->_isPlaying)
    {
      // next target time is the current sim time plus the recording rate increment
      gazebo::common::Time target = this->_world->GetSimTime() + dt;

      // run the ros and gzserver updates in parallel
      auto rosbag_run = boost::bind(&rosbag::Player::runFor, this->_rosbagPtr, target.sec, target.nsec);
      auto gz_run = boost::bind(&gazebo::physics::World::Step, this->_world, 1);
      boost::thread rosbag_thread(rosbag_run);
      boost::thread gz_thread(gz_run);

      // sleep to ensure realtime, this can be mosified in the future to support faster/slower playback
      boost::this_thread::sleep(boost::posix_time::milliseconds(dt_ms));

      // wait for the log players to return
      rosbag_thread.join();
      gz_thread.join();

      // we've reached the end of the current playback files
      if(this->_isPlaying && this->_world->GetSimTime() >= gazebo::util::LogPlay::Instance()->LogEndTime())
      {
        // load the next files or rollover back to the start
        int next = this->_logIndex + 1;
        if(!this->setPlaybackIndex(next)) {
          this->setPlaybackIndex(1);
        }
      }
    }

    // while paused, but still configured/running wait for the user to press play
    while(!this->_isPlaying && !this->_recordingDir.empty())
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  }
}

bool GazeboRosPlaybackPlugin::setPlaybackIndex(int index)
{
  // assumes previous directory validation, internal use only
  boost::filesystem::path gz_dir = this->_recordingDir / "gzserver";
  boost::filesystem::path ros_dir = this->_recordingDir / "ros";

  // ensure the files exist at the index, if not return failure
  boost::filesystem::path gz_file = gz_dir / (std::to_string(index) + ".log");
  boost::filesystem::path ros_file = ros_dir / (std::to_string(index) + ".bag");
  if(!boost::filesystem::exists(gz_file) || !boost::filesystem::exists(ros_file))
    return false;

  // stop an existing rosbag player and cleanup
  if(this->_rosbagPtr)
  {
    this->_rosbagPtr->shutdown();
    this->_rosbagThreadPtr->join();
    this->_rosbagThreadPtr.reset();
    this->_rosbagPtr.reset();
  }

  // start the paused rosbag player for the given rosbag, publish as fast as possible
  rosbag::PlayerOptions opts;
  opts.start_paused = true;
  opts.quiet = true;
  opts.bags.push_back(ros_file.native());

  // spawn the rosbag player in a separate thread
  this->_rosbagPtr.reset(new rosbag::Player(opts));
  auto rosbag_run = boost::bind(&rosbag::Player::publish, this->_rosbagPtr);
  this->_rosbagThreadPtr.reset(new boost::thread(rosbag_run));

  // open the log for Gazebo
  this->_gzFileStr = gz_file.native().c_str();
  gazebo::util::LogPlay::Instance()->Open(this->_gzFileStr);

  // properly update/delete models in the world with changes that
  // may have happened between files, inserts are handled by playback
  // get the SDF world description from the log file
  gazebo::util::LogPlay::Instance()->Step(this->_sdfStr);

  // load the sdf
  this->_sdfPtr.reset(new sdf::SDF);
  sdf::init(this->_sdfPtr);
  sdf::readString(this->_sdfStr, this->_sdfPtr);

  // strip plugins from the sdf
  sdf::ElementPtr sdf_world = this->_sdfPtr->Root()->GetElement("world");
  this->removeSDFPlugins(sdf_world);

  // load all of the models and ligts into a name->SDF map for later
  boost::unordered_map <std::string, sdf::ElementPtr> objects;
  if(sdf_world->HasElement("model"))
  {
    sdf::ElementPtr elem = sdf_world->GetElement("model");
    while(elem)
    {
      objects[elem->GetAttribute("name")->GetAsString()] = elem;
      elem = elem->GetNextElement("model");
    }
  }
  if(sdf_world->HasElement("light"))
  {
    sdf::ElementPtr elem = sdf_world->GetElement("light");
    while(elem)
    {
      objects[elem->GetAttribute("name")->GetAsString()] = elem;
      elem = elem->GetNextElement("light");
    }
  }

  // mark models for deletion / update them between scene changes
  for(auto model : this->_world->GetModels())
  {
    // the model is no longer in the scene, mark for deletion
    if(objects.find(model->GetName()) == objects.end())
      this->_world->RemoveModel(model->GetName());

    // check if the model SDF has changed
    else if(model->UnscaledSDF()->ToString("") !=
            objects[model->GetName()]->ToString(""))
    {
      // do a strange update sandwich otherwise fillmsg will except or fail to publish
      // using model/modify causes a crash/race condition as well
      model->UpdateParameters(objects[model->GetName()]);
      gazebo::msgs::Model msg;
      model->FillMsg(msg);
      model->UpdateParameters(objects[model->GetName()]);
    }
  }

  // mark lights for deletion / update them between scene changes
  for(auto light : this->_world->Lights())
  {
    // the light is no longer in the scene, mark for deletion
    if(objects.find(light->GetName()) == objects.end())
      this->_world->RemoveModel(light->GetName());
    else
    {
      // update the light in the world (e.g. position) and scene
      auto msg = gazebo::msgs::LightFromSDF(objects[light->GetName()]);
      this->_gz_nh->Advertise<gazebo::msgs::Light>("~/light/modify")->Publish(msg, true);
    }
  }

  // rewind to let the logger run normally within gazebo
  gazebo::util::LogPlay::Instance()->Rewind();

  // before we advance, reset sim time for rollover loading
  if(index == 1)
    this->_world->ResetTime();

  // mark the current file index
  this->_logIndex = index;

  // successfully set and started
  return true;
}

void GazeboRosPlaybackPlugin::removeSDFPlugins(sdf::ElementPtr elem)
{
  // Allow sensor plugins to playback the simulation state as this saves a lot
  // of space in the rosbag (e.g. images will be deterministically produced by
  // Gazebo, so we don't need to save them in the rosbag).
  if(elem->GetName() == "sensor")
    return;

  // SDF API is a bit strange, trying to use GetElement can result in insertion
  // so take the performance hit and check with HasElement first
  while(elem->HasElement("plugin"))
    elem->RemoveChild(elem->GetElement("plugin"));

  // recurse on all children to make sure we cover nested cases
  for(auto child = elem->GetFirstElement(); child; child = child->GetNextElement())
    this->removeSDFPlugins(child);
}

// register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosPlaybackPlugin)

} // namespace gazebo
