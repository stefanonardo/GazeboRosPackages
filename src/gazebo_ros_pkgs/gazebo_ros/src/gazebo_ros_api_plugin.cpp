/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig, John Hsu, Dave Coleman
 * Date: Jun 10 2013
 */

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo_ros/gazebo_ros_api_plugin.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Base.hh>

namespace gazebo
{

GazeboRosApiPlugin::GazeboRosApiPlugin() :
  physics_reconfigure_initialized_(false),
  world_created_(false),
  stop_(false),
  plugin_loaded_(false),
  pub_link_states_connection_count_(0),
  pub_model_states_connection_count_(0),
  pub_contact_data_connection_count_(0),
  pub_clock_frequency_(0)
{
}

GazeboRosApiPlugin::~GazeboRosApiPlugin()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","GazeboRosApiPlugin Deconstructor start");

  // Unload the sigint event
  sigint_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","After sigint_event unload");

  // Don't attempt to unload this plugin if it was never loaded in the Load() function
  if(!plugin_loaded_)
  {
    ROS_DEBUG_STREAM_NAMED("api_plugin","Deconstructor skipped because never loaded");
    return;
  }

  // Disconnect slots
  load_gazebo_ros_api_plugin_event_.reset();
  wrench_update_event_.reset();
  force_update_event_.reset();
  time_update_event_.reset();

  // NRP: Disconnect contact data callback event
  contacts_update_event_.reset();

  ROS_DEBUG_STREAM_NAMED("api_plugin","Slots disconnected");

  if (pub_link_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    pub_link_states_event_.reset();
  if (pub_model_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    pub_model_states_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Disconnected World Updates");

  // Stop the multi threaded ROS spinner
  async_ros_spin_->stop();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Async ROS Spin Stopped");

  // Shutdown the ROS node
  nh_->shutdown();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Node Handle Shutdown");

  // Shutdown ROS queue
  gazebo_callback_queue_thread_->join();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Callback Queue Joined");

  // Physics Dynamic Reconfigure
  physics_reconfigure_thread_->join();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Physics reconfigure joined");

  // Delete Force and Wrench Jobs
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();)
  {
    delete (*iter);
    iter = force_joint_jobs_.erase(iter);
  }
  force_joint_jobs_.clear();
  ROS_DEBUG_STREAM_NAMED("api_plugin","ForceJointJobs deleted");
  for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
  {
    delete (*iter);
    iter = wrench_body_jobs_.erase(iter);
  }
  wrench_body_jobs_.clear();
  lock_.unlock();
  ROS_DEBUG_STREAM_NAMED("api_plugin","WrenchBodyJobs deleted");

  ROS_DEBUG_STREAM_NAMED("api_plugin","Unloaded");
}

void GazeboRosApiPlugin::shutdownSignal()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","shutdownSignal() recieved");
  stop_ = true;
}

void GazeboRosApiPlugin::Load(int argc, char** argv)
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","Load");

  // connect to sigint event
  sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboRosApiPlugin::shutdownSignal,this));

  // setup ros related
  if (!ros::isInitialized())
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
  else
    ROS_ERROR("Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

  // check if the ros master is available - required
  while(!ros::master::check())
  {
    ROS_WARN_STREAM_NAMED("api_plugin","No ROS master - start roscore to continue...");
    // wait 0.5 second
    usleep(500*1000); // can't use ROS Time here b/c node handle is not yet initialized

    if(stop_)
    {
      ROS_WARN_STREAM_NAMED("api_plugin","Canceled loading Gazebo ROS API plugin by sigint event");
      return;
    }
  }

  nh_.reset(new ros::NodeHandle("~")); // advertise topics and services in this node's namespace

  // Built-in multi-threaded ROS spinning
  async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
  async_ros_spin_->start();

  /// \brief setup custom callback queue
  gazebo_callback_queue_thread_.reset(new boost::thread( &GazeboRosApiPlugin::gazeboQueueThread, this) );

  /// \brief start a thread for the physics dynamic reconfigure node
  physics_reconfigure_thread_.reset(new boost::thread(boost::bind(&GazeboRosApiPlugin::physicsReconfigureThread, this)));

  // below needs the world to be created first
  load_gazebo_ros_api_plugin_event_ = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboRosApiPlugin::loadGazeboRosApiPlugin,this,_1));

  plugin_loaded_ = true;
  ROS_INFO("Finished loading Gazebo ROS API Plugin.");
}

void GazeboRosApiPlugin::loadGazeboRosApiPlugin(std::string world_name)
{
  // make sure things are only called once
  lock_.lock();
  if (world_created_)
  {
    lock_.unlock();
    return;
  }

  // set flag to true and load this plugin
  world_created_ = true;
  lock_.unlock();

  world_ = gazebo::physics::get_world(world_name);
  if (!world_)
  {
    //ROS_ERROR("world name: [%s]",world->GetName().c_str());
    // connect helper function to signal for scheduling torque/forces, etc
    ROS_FATAL("cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
    return;
  }

  gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebonode_->Init(world_name);
  //stat_sub_ = gazebonode_->Subscribe("~/world_stats", &GazeboRosApiPlugin::publishSimTime, this); // TODO: does not work in server plugin?
  factory_pub_ = gazebonode_->Advertise<gazebo::msgs::Factory>("~/factory");
  nrp_factory_light_pub_ = gazebonode_->Advertise<gazebo::msgs::Light>("~/factory/light");
  // request_pub_ = gazebonode_->Advertise<gazebo::msgs::Request>("~/request");
  response_sub_ = gazebonode_->Subscribe("~/response",&GazeboRosApiPlugin::onResponse, this);

  // reset topic connection counts
  pub_link_states_connection_count_ = 0;
  pub_model_states_connection_count_ = 0;

  /// \brief advertise all services
  advertiseServices();

  // hooks for applying forces, publishing simtime on /clock
  wrench_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::wrenchBodySchedulerSlot,this));
  force_update_event_  = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::forceJointSchedulerSlot,this));
  time_update_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishSimTime,this));
}

void GazeboRosApiPlugin::onResponse(ConstResponsePtr &response)
{

}

void GazeboRosApiPlugin::gazeboQueueThread()
{
  static const double timeout = 0.001;
  while (nh_->ok())
  {
    gazebo_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosApiPlugin::advertiseServices()
{
  // publish clock for simulated ros time
  pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock",10);

  // Advertise spawn services on the custom queue
  std::string spawn_sdf_entity_service_name("spawn_sdf_entity");
  ros::AdvertiseServiceOptions spawn_sdf_entity_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnEntity>(
                                                                   spawn_sdf_entity_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::spawnSDFEntity,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  spawn_sdf_entity_service_ = nh_->advertiseService(spawn_sdf_entity_aso);

  // Advertise spawn services on the custom queue
  std::string spawn_urdf_entity_service_name("spawn_urdf_entity");
  ros::AdvertiseServiceOptions spawn_urdf_entity_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnEntity>(
                                                                   spawn_urdf_entity_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::spawnURDFEntity,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  spawn_urdf_entity_service_ = nh_->advertiseService(spawn_urdf_entity_aso);

  // NRRPLT-7855
  // Advertise delete services on the custom queue
  // std::string delete_model_service_name("delete_model");
  // ros::AdvertiseServiceOptions delete_aso =
  //   ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteModel>(
  //                                                                  delete_model_service_name,
  //                                                                  boost::bind(&GazeboRosApiPlugin::deleteModel,this,_1,_2),
  //                                                                  ros::VoidPtr(), &gazebo_queue_);
  // delete_model_service_ = nh_->advertiseService(delete_aso);

  // Advertise more services on the custom queue
  std::string get_model_properties_service_name("get_model_properties");
  ros::AdvertiseServiceOptions get_model_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelProperties>(
                                                                          get_model_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getModelProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_model_properties_service_ = nh_->advertiseService(get_model_properties_aso);

  // Advertise more services on the custom queue
  std::string get_model_state_service_name("get_model_state");
  ros::AdvertiseServiceOptions get_model_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelState>(
                                                                     get_model_state_service_name,
                                                                     boost::bind(&GazeboRosApiPlugin::getModelState,this,_1,_2),
                                                                     ros::VoidPtr(), &gazebo_queue_);
  get_model_state_service_ = nh_->advertiseService(get_model_state_aso);

  // Advertise more services on the custom queue
  std::string get_world_properties_service_name("get_world_properties");
  ros::AdvertiseServiceOptions get_world_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetWorldProperties>(
                                                                          get_world_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getWorldProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_world_properties_service_ = nh_->advertiseService(get_world_properties_aso);

  // Advertise more services on the custom queue
  std::string get_joint_properties_service_name("get_joint_properties");
  ros::AdvertiseServiceOptions get_joint_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetJointProperties>(
                                                                          get_joint_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getJointProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_joint_properties_service_ = nh_->advertiseService(get_joint_properties_aso);

  // Advertise more services on the custom queue
  std::string get_link_properties_service_name("get_link_properties");
  ros::AdvertiseServiceOptions get_link_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkProperties>(
                                                                         get_link_properties_service_name,
                                                                         boost::bind(&GazeboRosApiPlugin::getLinkProperties,this,_1,_2),
                                                                         ros::VoidPtr(), &gazebo_queue_);
  get_link_properties_service_ = nh_->advertiseService(get_link_properties_aso);

  // Advertise more services on the custom queue
  std::string get_link_state_service_name("get_link_state");
  ros::AdvertiseServiceOptions get_link_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkState>(
                                                                    get_link_state_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::getLinkState,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  get_link_state_service_ = nh_->advertiseService(get_link_state_aso);

  // Advertise more services on the custom queue
  std::string get_physics_properties_service_name("get_physics_properties");
  ros::AdvertiseServiceOptions get_physics_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetPhysicsProperties>(
                                                                            get_physics_properties_service_name,
                                                                            boost::bind(&GazeboRosApiPlugin::getPhysicsProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  get_physics_properties_service_ = nh_->advertiseService(get_physics_properties_aso);

  // publish complete link states in world frame
  ros::AdvertiseOptions pub_link_states_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::LinkStates>(
                                                           "link_states",10,
                                                           boost::bind(&GazeboRosApiPlugin::onLinkStatesConnect,this),
                                                           boost::bind(&GazeboRosApiPlugin::onLinkStatesDisconnect,this),
                                                           ros::VoidPtr(), &gazebo_queue_);
  pub_link_states_ = nh_->advertise(pub_link_states_ao);

  // publish complete model states in world frame
  ros::AdvertiseOptions pub_model_states_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::ModelStates>(
                                                            "model_states",10,
                                                            boost::bind(&GazeboRosApiPlugin::onModelStatesConnect,this),
                                                            boost::bind(&GazeboRosApiPlugin::onModelStatesDisconnect,this),
                                                            ros::VoidPtr(), &gazebo_queue_);
  pub_model_states_ = nh_->advertise(pub_model_states_ao);

  // NRP: publish contact point data after each simulation step
  ros::AdvertiseOptions pub_contact_data_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::ContactsState>(
                                                            "contact_point_data",10,
                                                            boost::bind(&GazeboRosApiPlugin::onContactPointDataConnect,this),
                                                            boost::bind(&GazeboRosApiPlugin::onContactPointDataDisconnect,this),
                                                            ros::VoidPtr(), &gazebo_queue_);
  pub_contact_data_ = nh_->advertise(pub_contact_data_ao);

  // Advertise more services on the custom queue
  std::string set_link_properties_service_name("set_link_properties");
  ros::AdvertiseServiceOptions set_link_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkProperties>(
                                                                         set_link_properties_service_name,
                                                                         boost::bind(&GazeboRosApiPlugin::setLinkProperties,this,_1,_2),
                                                                         ros::VoidPtr(), &gazebo_queue_);
  set_link_properties_service_ = nh_->advertiseService(set_link_properties_aso);

  // Advertise more services on the custom queue
  std::string set_physics_properties_service_name("set_physics_properties");
  ros::AdvertiseServiceOptions set_physics_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetPhysicsProperties>(
                                                                            set_physics_properties_service_name,
                                                                            boost::bind(&GazeboRosApiPlugin::setPhysicsProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  set_physics_properties_service_ = nh_->advertiseService(set_physics_properties_aso);

  // Advertise more services on the custom queue
  std::string set_model_state_service_name("set_model_state");
  ros::AdvertiseServiceOptions set_model_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelState>(
                                                                     set_model_state_service_name,
                                                                     boost::bind(&GazeboRosApiPlugin::setModelState,this,_1,_2),
                                                                     ros::VoidPtr(), &gazebo_queue_);
  set_model_state_service_ = nh_->advertiseService(set_model_state_aso);

  // Advertise more services on the custom queue
  std::string set_model_configuration_service_name("set_model_configuration");
  ros::AdvertiseServiceOptions set_model_configuration_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelConfiguration>(
                                                                             set_model_configuration_service_name,
                                                                             boost::bind(&GazeboRosApiPlugin::setModelConfiguration,this,_1,_2),
                                                                             ros::VoidPtr(), &gazebo_queue_);
  set_model_configuration_service_ = nh_->advertiseService(set_model_configuration_aso);

  // Advertise more services on the custom queue
  std::string set_joint_properties_service_name("set_joint_properties");
  ros::AdvertiseServiceOptions set_joint_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetJointProperties>(
                                                                          set_joint_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::setJointProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  set_joint_properties_service_ = nh_->advertiseService(set_joint_properties_aso);

  // Advertise more services on the custom queue
  std::string set_link_state_service_name("set_link_state");
  ros::AdvertiseServiceOptions set_link_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkState>(
                                                                    set_link_state_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::setLinkState,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  set_link_state_service_ = nh_->advertiseService(set_link_state_aso);

  // Advertise topic on custom queue
  // topic callback version for set_link_state
  ros::SubscribeOptions link_state_so =
    ros::SubscribeOptions::create<gazebo_msgs::LinkState>(
                                                          "set_link_state",10,
                                                          boost::bind( &GazeboRosApiPlugin::updateLinkState,this,_1),
                                                          ros::VoidPtr(), &gazebo_queue_);
  set_link_state_topic_ = nh_->subscribe(link_state_so);

  // topic callback version for set_model_state
  ros::SubscribeOptions model_state_so =
    ros::SubscribeOptions::create<gazebo_msgs::ModelState>(
                                                           "set_model_state",10,
                                                           boost::bind( &GazeboRosApiPlugin::updateModelState,this,_1),
                                                           ros::VoidPtr(), &gazebo_queue_);
  set_model_state_topic_ = nh_->subscribe(model_state_so);

  // Advertise more services on the custom queue
  std::string pause_physics_service_name("pause_physics");
  ros::AdvertiseServiceOptions pause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          pause_physics_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::pausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  pause_physics_service_ = nh_->advertiseService(pause_physics_aso);

  // Advertise more services on the custom queue
  std::string unpause_physics_service_name("unpause_physics");
  ros::AdvertiseServiceOptions unpause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          unpause_physics_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::unpausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  unpause_physics_service_ = nh_->advertiseService(unpause_physics_aso);

  // Advertise more services on the custom queue
  std::string apply_body_wrench_service_name("apply_body_wrench");
  ros::AdvertiseServiceOptions apply_body_wrench_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyBodyWrench>(
                                                                       apply_body_wrench_service_name,
                                                                       boost::bind(&GazeboRosApiPlugin::applyBodyWrench,this,_1,_2),
                                                                       ros::VoidPtr(), &gazebo_queue_);
  apply_body_wrench_service_ = nh_->advertiseService(apply_body_wrench_aso);

  // Advertise more services on the custom queue
  std::string apply_joint_effort_service_name("apply_joint_effort");
  ros::AdvertiseServiceOptions apply_joint_effort_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyJointEffort>(
                                                                        apply_joint_effort_service_name,
                                                                        boost::bind(&GazeboRosApiPlugin::applyJointEffort,this,_1,_2),
                                                                        ros::VoidPtr(), &gazebo_queue_);
  apply_joint_effort_service_ = nh_->advertiseService(apply_joint_effort_aso);

  // Advertise more services on the custom queue
  std::string clear_joint_forces_service_name("clear_joint_forces");
  ros::AdvertiseServiceOptions clear_joint_forces_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::JointRequest>(
                                                                    clear_joint_forces_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::clearJointForces,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  clear_joint_forces_service_ = nh_->advertiseService(clear_joint_forces_aso);

  // Advertise more services on the custom queue
  std::string clear_body_wrenches_service_name("clear_body_wrenches");
  ros::AdvertiseServiceOptions clear_body_wrenches_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::BodyRequest>(
                                                                   clear_body_wrenches_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::clearBodyWrenches,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  clear_body_wrenches_service_ = nh_->advertiseService(clear_body_wrenches_aso);

  // Advertise more services on the custom queue
  std::string reset_simulation_service_name("reset_simulation");
  ros::AdvertiseServiceOptions reset_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_simulation_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::resetSimulation,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  reset_simulation_service_ = nh_->advertiseService(reset_simulation_aso);

  // Advertise more services on the custom queue
  std::string reset_world_service_name("reset_world");
  ros::AdvertiseServiceOptions reset_world_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_world_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::resetWorld,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  reset_world_service_ = nh_->advertiseService(reset_world_aso);

  nrpAdvertiseServices();

  // set param for use_sim_time if not set by user already
  nh_->setParam("/use_sim_time", true);

  // todo: contemplate setting environment variable ROBOT=sim here???
  nh_->getParam("pub_clock_frequency", pub_clock_frequency_);
  last_pub_clock_time_ = world_->SimTime();
}

void GazeboRosApiPlugin::onLinkStatesConnect()
{
  pub_link_states_connection_count_++;
  if (pub_link_states_connection_count_ == 1) // connect on first subscriber
    pub_link_states_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishLinkStates,this));
}

void GazeboRosApiPlugin::onModelStatesConnect()
{
  pub_model_states_connection_count_++;
  if (pub_model_states_connection_count_ == 1) // connect on first subscriber
    pub_model_states_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishModelStates,this));
}

void GazeboRosApiPlugin::onLinkStatesDisconnect()
{
  pub_link_states_connection_count_--;
  if (pub_link_states_connection_count_ <= 0) // disconnect with no subscribers
  {
    pub_link_states_event_.reset();
    if (pub_link_states_connection_count_ < 0) // should not be possible
      ROS_ERROR("one too many disconnect from pub_link_states_ in gazebo_ros.cpp? something weird");
  }
}

void GazeboRosApiPlugin::onModelStatesDisconnect()
{
  pub_model_states_connection_count_--;
  if (pub_model_states_connection_count_ <= 0) // disconnect with no subscribers
  {
    pub_model_states_event_.reset();
    if (pub_model_states_connection_count_ < 0) // should not be possible
      ROS_ERROR("one too many disconnect from pub_model_states_ in gazebo_ros.cpp? something weird");
  }
}

void GazeboRosApiPlugin::onContactPointDataConnect()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin", "Contact point subscriber connecting");
  pub_contact_data_connection_count_++;
  if (pub_contact_data_connection_count_ == 1) // connect on first subscriber
  {
    contacts_sub_ = gazebonode_->Subscribe("~/physics/contacts",
      &GazeboRosApiPlugin::onContactPointDataReceived, this);
    ROS_DEBUG_STREAM_NAMED("api_plugin","Subscribed to gazebo contact message");
  }
}

void GazeboRosApiPlugin::onContactPointDataDisconnect()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","Contact point subscriber disconnecting");
  pub_contact_data_connection_count_--;
  if (pub_contact_data_connection_count_ <= 0) // disconnect with no subscribers
  {
    contacts_sub_.reset();
    if (pub_contact_data_connection_count_ < 0)
      ROS_ERROR("one too many disconnect from pub_contact_data_ in gazebo_ros.cpp? something weird");
    ROS_DEBUG_STREAM_NAMED("api_plugin","Subscriber to gazebo contact message deleted");
  }
}


bool GazeboRosApiPlugin::spawnURDFEntity(gazebo_msgs::SpawnEntity::Request &req,
                                         gazebo_msgs::SpawnEntity::Response &res)
{
  // get name space for the corresponding entity plugins
  std::string entity_namespace = req.entity_namespace;

  // incoming robot entity string
  std::string entity_xml = req.entity_xml;

  if (!isURDF(entity_xml))
  {
    ROS_ERROR("SpawnEntity: Failure - entity format is not URDF.");
    res.success = false;
    res.status_message = "SpawnEntity: Failure - entity format is not URDF.";
    return false;
  }

  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from entity_xml
  /// @todo: does tinyxml have functionality for this?
  /// @todo: should gazebo take care of the declaration?
  {
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    size_t pos1 = entity_xml.find(open_bracket,0);
    size_t pos2 = entity_xml.find(close_bracket,0);
    if (pos1 != std::string::npos && pos2 != std::string::npos)
      entity_xml.replace(pos1,pos2-pos1+2,std::string(""));
  }

  // Now, replace package://xxx with the full path to the package
  {
    std::string package_prefix("package://");
    size_t pos1 = entity_xml.find(package_prefix,0);
    while (pos1 != std::string::npos)
    {
      size_t pos2 = entity_xml.find("/", pos1+10);
      //ROS_DEBUG(" pos %d %d",(int)pos1, (int)pos2);
      if (pos2 == std::string::npos || pos1 >= pos2)
      {
        ROS_ERROR("malformed package name?");
        break;
      }

      std::string package_name = entity_xml.substr(pos1+10,pos2-pos1-10);
      //ROS_DEBUG("package name [%s]", package_name.c_str());
      std::string package_path = ros::package::getPath(package_name);
      if (package_path.empty())
      {
        ROS_FATAL("Package[%s] does not have a path",package_name.c_str());
        res.success = false;
        res.status_message = std::string("urdf reference package name does not exist: ")+package_name;
        return false;
      }
      ROS_DEBUG_ONCE("Package name [%s] has path [%s]", package_name.c_str(), package_path.c_str());

      entity_xml.replace(pos1,(pos2-pos1),package_path);
      pos1 = entity_xml.find(package_prefix, pos1);
    }
  }
  // ROS_DEBUG("Model XML\n\n%s\n\n ",entity_xml.c_str());

  req.entity_xml = entity_xml;

  // Model is now considered convert to SDF
  return spawnSDFEntity(req,res);
}

bool GazeboRosApiPlugin::spawnSDFEntity(gazebo_msgs::SpawnEntity::Request &req,
                                        gazebo_msgs::SpawnEntity::Response &res)
{
  // incoming entity name
  std::string entity_name = req.entity_name;

  // get name space for the corresponding entity plugins
  std::string entity_namespace = req.entity_namespace;

  // get initial pose of entity
  ignition::math::Vector3d initial_xyz(req.initial_pose.position.x,req.initial_pose.position.y,req.initial_pose.position.z);
  // get initial roll pitch yaw (fixed frame transform)
  ignition::math::Quaterniond initial_q(req.initial_pose.orientation.w,req.initial_pose.orientation.x,req.initial_pose.orientation.y,req.initial_pose.orientation.z);

  // reference frame for initial pose definition, modify initial pose if defined
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.reference_frame));
  if (frame)
  {
    // convert to relative pose
    ignition::math::Pose3d frame_pose = frame->WorldPose();
    initial_xyz = frame_pose.Rot().RotateVector(initial_xyz);
    initial_xyz += frame_pose.Pos();
    initial_q *= frame_pose.Rot();
  }

  /// @todo: map is really wrong, need to use tf here somehow
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_DEBUG("SpawnEntity: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    res.success = false;
    res.status_message = "SpawnEntity: reference reference_frame not found, did you forget to scope the link by model name?";
    return true;
  }

  // incoming robot model string
  std::string entity_xml = req.entity_xml;

  // store resulting Gazebo Model XML to be sent to spawn queue
  // get incoming string containing either an URDF or a Gazebo Model XML
  // grab from parameter server if necessary convert to SDF if necessary
  stripXmlDeclaration(entity_xml);

  // put string in TiXmlDocument for manipulation
  TiXmlDocument gazebo_entity_xml;
  gazebo_entity_xml.Parse(entity_xml.c_str());

  // optional model manipulations: update initial pose && replace model name
  if (isSDF(entity_xml))
  {
    updateSDFAttributes(gazebo_entity_xml, entity_name, initial_xyz, initial_q);

    // Walk recursively through the entire SDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!entity_namespace.empty())
    {
      // Get root element for SDF
      TiXmlNode* model_tixml = gazebo_entity_xml.FirstChild("sdf");
      model_tixml = (!model_tixml) ?
          gazebo_entity_xml.FirstChild("gazebo") : model_tixml;
      if (model_tixml)
      {
        walkChildAddRobotNamespace(model_tixml, entity_namespace);
      }
      else
      {
        ROS_WARN("Unable to add robot namespace to xml");
      }
    }
  }
  else if (isURDF(entity_xml))
  {
    updateURDFModelPose(gazebo_entity_xml, initial_xyz, initial_q);
    updateURDFName(gazebo_entity_xml, entity_name);

    // Walk recursively through the entire URDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!entity_namespace.empty())
    {
      // Get root element for URDF
      TiXmlNode* model_tixml = gazebo_entity_xml.FirstChild("robot");
      if (model_tixml)
      {
        walkChildAddRobotNamespace(model_tixml, entity_namespace);
      }
      else
      {
        ROS_WARN("Unable to add robot namespace to xml");
      }
    }
  }
  else
  {
    ROS_ERROR("GazeboRosApiPlugin SpawnEntity Failure: input xml format not recognized");
    res.success = false;
    res.status_message = std::string("GazeboRosApiPlugin SpawnEntity Failure: input entity_xml not SDF or URDF, or cannot be converted to Gazebo compatible format.");
    return true;
  }

  // do spawning check if spawn worked, return response
  return spawnAndConform(gazebo_entity_xml, entity_name, res);
}

bool GazeboRosApiPlugin::deleteModel(gazebo_msgs::DeleteModel::Request &req,
                                     gazebo_msgs::DeleteModel::Response &res)
{
  // clear forces, etc for the body in question
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
  if (!model)
  {
    ROS_ERROR("DeleteModel: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "DeleteModel: model does not exist";
    return true;
  }

  // delete wrench jobs on bodies
  for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
  {
    gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
    if (body)
    {
      // look for it in jobs, delete body wrench jobs
      clearBodyWrenches(body->GetScopedName());
    }
  }

  // delete force jobs on joints
  gazebo::physics::Joint_V joints = model->GetJoints();
  for (unsigned int i=0;i< joints.size(); i++)
  {
    // look for it in jobs, delete joint force jobs
    clearJointForces(joints[i]->GetName());
  }

  // send delete model request
  this->nrpPublishRequest("entity_delete", req.model_name);

  ros::Duration model_spawn_timeout(60.0);
  ros::Time timeout = ros::Time::now() + model_spawn_timeout;
  // wait and verify that model is deleted
  while (true)
  {
    if (ros::Time::now() > timeout)
    {
      res.success = false;
      res.status_message = std::string("DeleteModel: Model pushed to delete queue, but delete service timed out waiting for model to disappear from simulation");
      return true;
    }
    {
      //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
      if (!world_->ModelByName(req.model_name)) break;
    }
    ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
    usleep(1000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("DeleteModel: successfully deleted model");
  return true;
}

bool GazeboRosApiPlugin::getModelState(gazebo_msgs::GetModelState::Request &req,
                                       gazebo_msgs::GetModelState::Response &res)
{
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.relative_entity_name));
  if (!model)
  {
    ROS_ERROR("GetModelState: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "GetModelState: model does not exist";
    return true;
  }
  else
  {
     /**
     * @brief creates a header for the result
     * @author Markus Bader markus.bader@tuwien.ac.at
     * @date 21th Nov 2014
     **/
    {
      std::map<std::string, unsigned int>::iterator it = access_count_get_model_state_.find(req.model_name);
      if(it == access_count_get_model_state_.end())
      {
        access_count_get_model_state_.insert( std::pair<std::string, unsigned int>(req.model_name, 1) );
        res.header.seq = 1;
      }
      else
      {
        it->second++;
        res.header.seq = it->second;
      }
      res.header.stamp = ros::Time::now();
      res.header.frame_id = req.relative_entity_name; /// @brief this is a redundant information
    }
    // get model pose
    ignition::math::Pose3d       model_pose = model->WorldPose();
    ignition::math::Vector3d    model_pos = model_pose.Pos();
    ignition::math::Quaterniond model_rot = model_pose.Rot();

    //get model scale
    ignition::math::Vector3d    model_scale = model->Scale();

    // get model twist
    ignition::math::Vector3d model_linear_vel  = model->WorldLinearVel();
    ignition::math::Vector3d model_angular_vel = model->WorldAngularVel();


    if (frame)
    {
      // convert to relative pose
      ignition::math::Pose3d frame_pose = frame->WorldPose();
      model_pos = model_pos - frame_pose.Pos();
      model_pos = frame_pose.Rot().RotateVectorReverse(model_pos);
      model_rot *= frame_pose.Rot().Inverse();

      // convert to relative rates
      ignition::math::Vector3d frame_vpos = frame->WorldLinearVel(); // get velocity in gazebo frame
      ignition::math::Vector3d frame_veul = frame->WorldAngularVel(); // get velocity in gazebo frame
      model_linear_vel = frame_pose.Rot().RotateVector(model_linear_vel - frame_vpos);
      model_angular_vel = frame_pose.Rot().RotateVector(model_angular_vel - frame_veul);
    }
    /// @todo: FIXME map is really wrong, need to use tf here somehow
    else if (req.relative_entity_name == "" || req.relative_entity_name == "world" || req.relative_entity_name == "map" || req.relative_entity_name == "/map")
    {
      ROS_DEBUG("GetModelState: relative_entity_name is empty/world/map, using inertial frame");
    }
    else
    {
      res.success = false;
      res.status_message = "GetModelState: reference relative_entity_name not found, did you forget to scope the body by model name?";
      return true;
    }

    // fill in response
    res.pose.position.x = model_pos.X();
    res.pose.position.y = model_pos.Y();
    res.pose.position.z = model_pos.Z();
    res.pose.orientation.w = model_rot.W();
    res.pose.orientation.x = model_rot.X();
    res.pose.orientation.y = model_rot.Y();
    res.pose.orientation.z = model_rot.Z();

    res.scale.x = model_scale.X();
    res.scale.y = model_scale.Y();
    res.scale.z = model_scale.Z();

    res.twist.linear.x = model_linear_vel.X();
    res.twist.linear.y = model_linear_vel.Y();
    res.twist.linear.z = model_linear_vel.Z();
    res.twist.angular.x = model_angular_vel.X();
    res.twist.angular.y = model_angular_vel.Y();
    res.twist.angular.z = model_angular_vel.Z();

    res.success = true;
    res.status_message = "GetModelState: got properties";
    return true;
  }
  return true;
}

bool GazeboRosApiPlugin::getModelProperties(gazebo_msgs::GetModelProperties::Request &req,
                                            gazebo_msgs::GetModelProperties::Response &res)
{
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
  if (!model)
  {
    ROS_ERROR("GetModelProperties: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "GetModelProperties: model does not exist";
    return true;
  }
  else
  {
    // get model parent name
    gazebo::physics::ModelPtr parent_model = boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetParent());
    if (parent_model) res.parent_model_name = parent_model->GetName();

    // get list of child bodies, geoms
    res.body_names.clear();
    res.geom_names.clear();
    for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
      if (body)
      {
        res.body_names.push_back(body->GetName());
        // get list of geoms
        for (unsigned int j = 0; j < body->GetChildCount() ; j++)
        {
          gazebo::physics::CollisionPtr geom = boost::dynamic_pointer_cast<gazebo::physics::Collision>(body->GetChild(j));
          if (geom)
            res.geom_names.push_back(geom->GetName());
        }
      }
    }

    // get list of joints
    res.joint_names.clear();

    gazebo::physics::Joint_V joints = model->GetJoints();
    for (unsigned int i=0;i< joints.size(); i++)
      res.joint_names.push_back( joints[i]->GetScopedName() );

    // get children model names
    res.child_model_names.clear();
    for (unsigned int j = 0; j < model->GetChildCount(); j++)
    {
      gazebo::physics::ModelPtr child_model = boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetChild(j));
      if (child_model)
        res.child_model_names.push_back(child_model->GetName() );
    }

    // get list of sensors
    res.sensor_names.clear();
    std::vector<std::string> sensors = model->GetSensorNames();

    for (unsigned int i=0; i< sensors.size(); i++)
    {
        res.sensor_names.push_back(sensors[i]);
    }

    // get list of sensor types
    res.sensor_types.clear();
    std::vector<std::string> sensor_types_names = model->GetSensorTypeNames();

    for (unsigned int i=0; i< sensor_types_names.size(); i++)
    {
        res.sensor_types.push_back(sensor_types_names[i]);
    }    

    // get list of camera RosTopics
    res.sensor_names_ROS.clear();
    res.rostopic_sensor_urls.clear();
    res.sensor_ros_message_type.clear();

    std::map<std::string,std::tuple<std::string, std::string>> rostopicsensors = model->GetSensorRosTopics();

    for (std::map<std::string,std::tuple<std::string, std::string>>::iterator it = rostopicsensors.begin(); it != rostopicsensors.end(); it++ )
    {
        res.sensor_names_ROS.push_back(it->first);
        res.rostopic_sensor_urls.push_back(model->GetName() + "/" + std::get<0>(it->second));
        res.sensor_ros_message_type.push_back(std::get<1>(it->second));
    }

    // get a list of Actuator Rostopics
    res.rostopic_actuator_urls.clear();
    res.actuator_ros_message_type.clear();

    auto rostopic_actuator = model->GetActuatorRosTopics();

    for (auto it = rostopic_actuator.begin(); it != rostopic_actuator.end(); it++)
    {
        res.rostopic_actuator_urls.push_back(model->GetName() + "/" + it->first);
        res.actuator_ros_message_type.push_back(it->second);
    }

    // is model static
    res.is_static = model->IsStatic();

    res.success = true;
    res.status_message = "GetModelProperties: got properties";
    return true;
  }
  return true;
}

bool GazeboRosApiPlugin::getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,
                                            gazebo_msgs::GetWorldProperties::Response &res)
{
  res.sim_time = world_->SimTime().Double();
  res.model_names.clear();
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
    res.model_names.push_back(world_->ModelByIndex(i)->GetName());
  //gzerr << "disablign rendering has not been implemented, rendering is always enabled\n";
  res.rendering_enabled = true; //world->GetRenderEngineEnabled();
  res.success = true;
  res.status_message = "GetWorldProperties: got properties";
  return true;
}

bool GazeboRosApiPlugin::getJointProperties(gazebo_msgs::GetJointProperties::Request &req,
                                            gazebo_msgs::GetJointProperties::Response &res)
{
  gazebo::physics::JointPtr joint;
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
    if (joint) break;
  }

  if (!joint)
  {
    res.success = false;
    res.status_message = "GetJointProperties: joint not found";
    return true;
  }
  else
  {
    if (joint->HasType(gazebo::physics::Base::HINGE_JOINT))
    {
      res.type = res.REVOLUTE;
    }
    else if (joint->HasType(gazebo::physics::Base::BALL_JOINT))
    {
      res.type = res.BALL;
    }
    else if (joint->HasType(gazebo::physics::Base::SLIDER_JOINT))
    {
      res.type = res.PRISMATIC;
    }
    else if (joint->HasType(gazebo::physics::Base::UNIVERSAL_JOINT))
    {
      res.type = res.UNIVERSAL;
    }
    else if (joint->HasType(gazebo::physics::Base::FIXED_JOINT))
    {
      res.type = res.FIXED;
    }
    else
    {
      res.success = false;
      res.status_message = "GetJointProperties: joint type not supported";
      return true;
    }

    res.damping.clear(); // to be added to gazebo
    //res.damping.push_back(joint->GetDamping(0));

    res.position.clear(); // use GetAngle(i)
    res.position.push_back(joint->Position(0));

    res.rate.clear(); // use GetVelocity(i)
    res.rate.push_back(joint->GetVelocity(0));

    res.success = true;
    res.status_message = "GetJointProperties: got properties";
    return true;
  }
}

bool GazeboRosApiPlugin::getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,
                                           gazebo_msgs::GetLinkProperties::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
  if (!body)
  {
    res.success = false;
    res.status_message = "GetLinkProperties: link not found, did you forget to scope the link by model name?";
    return true;
  }
  else
  {
    /// @todo: validate
    res.gravity_mode = body->GetGravityMode();

    res.mass = body->GetInertial()->Mass();

    gazebo::physics::InertialPtr inertia = body->GetInertial();
    res.ixx = inertia->IXX();
    res.iyy = inertia->IYY();
    res.izz = inertia->IZZ();
    res.ixy = inertia->IXY();
    res.ixz = inertia->IXZ();
    res.iyz = inertia->IYZ();

    ignition::math::Vector3d com = body->GetInertial()->CoG();
    res.com.position.x = com.X();
    res.com.position.y = com.Y();
    res.com.position.z = com.Z();
    res.com.orientation.x = 0; // @todo: gazebo do not support rotated inertia yet
    res.com.orientation.y = 0;
    res.com.orientation.z = 0;
    res.com.orientation.w = 1;

    res.success = true;
    res.status_message = "GetLinkProperties: got properties";
    return true;
  }
}

bool GazeboRosApiPlugin::getLinkState(gazebo_msgs::GetLinkState::Request &req,
                                      gazebo_msgs::GetLinkState::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.reference_frame));

  if (!body)
  {
    res.success = false;
    res.status_message = "GetLinkState: link not found, did you forget to scope the link by model name?";
    return true;
  }

  // get body pose
  ignition::math::Pose3d body_pose = body->WorldPose();
  // Get inertial rates
  ignition::math::Vector3d body_vpos = body->WorldLinearVel(); // get velocity in gazebo frame
  ignition::math::Vector3d body_veul = body->WorldAngularVel(); // get velocity in gazebo frame

  if (frame)
  {
    // convert to relative pose
    ignition::math::Pose3d frame_pose = frame->WorldPose();
    body_pose.Pos() = body_pose.Pos() - frame_pose.Pos();
    body_pose.Pos() = frame_pose.Rot().RotateVectorReverse(body_pose.Pos());
    body_pose.Rot() *= frame_pose.Rot().Inverse();

    // convert to relative rates
    ignition::math::Vector3d frame_vpos = frame->WorldLinearVel(); // get velocity in gazebo frame
    ignition::math::Vector3d frame_veul = frame->WorldAngularVel(); // get velocity in gazebo frame
    body_vpos = frame_pose.Rot().RotateVector(body_vpos - frame_vpos);
    body_veul = frame_pose.Rot().RotateVector(body_veul - frame_veul);
  }
  /// @todo: FIXME map is really wrong, need to use tf here somehow
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_DEBUG("GetLinkState: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    res.success = false;
    res.status_message = "GetLinkState: reference reference_frame not found, did you forget to scope the link by model name?";
    return true;
  }

  res.link_state.link_name = req.link_name;
  res.link_state.pose.position.x = body_pose.Pos().X();
  res.link_state.pose.position.y = body_pose.Pos().Y();
  res.link_state.pose.position.z = body_pose.Pos().Z();
  res.link_state.pose.orientation.x = body_pose.Rot().X();
  res.link_state.pose.orientation.y = body_pose.Rot().Y();
  res.link_state.pose.orientation.z = body_pose.Rot().Z();
  res.link_state.pose.orientation.w = body_pose.Rot().W();
  res.link_state.twist.linear.x = body_vpos.X();
  res.link_state.twist.linear.y = body_vpos.Y();
  res.link_state.twist.linear.z = body_vpos.Z();
  res.link_state.twist.angular.x = body_veul.X();
  res.link_state.twist.angular.y = body_veul.Y();
  res.link_state.twist.angular.z = body_veul.X();
  res.link_state.reference_frame = req.reference_frame;

  res.success = true;
  res.status_message = "GetLinkState: got state";
  return true;
}

bool GazeboRosApiPlugin::setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,
                                           gazebo_msgs::SetLinkProperties::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
  if (!body)
  {
    res.success = false;
    res.status_message = "SetLinkProperties: link not found, did you forget to scope the link by model name?";
    return true;
  }
  else
  {
    gazebo::physics::InertialPtr mass = body->GetInertial();
    // @todo: FIXME: add inertia matrix rotation to Gazebo
    // mass.SetInertiaRotation(gazebo::math::Quaternionion(req.com.orientation.w,res.com.orientation.x,req.com.orientation.y req.com.orientation.z));
    mass->SetCoG(ignition::math::Vector3d(req.com.position.x,req.com.position.y,req.com.position.z));
    mass->SetInertiaMatrix(req.ixx,req.iyy,req.izz,req.ixy,req.ixz,req.iyz);
    mass->SetMass(req.mass);
    body->SetGravityMode(req.gravity_mode);
    // @todo: mass change unverified
    res.success = true;
    res.status_message = "SetLinkProperties: properties set";
    return true;
  }
}

bool GazeboRosApiPlugin::setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,
                                              gazebo_msgs::SetPhysicsProperties::Response &res)
{
  // pause simulation if requested
  bool is_paused = world_->IsPaused();
  world_->SetPaused(true);

  // supported updates
  gazebo::physics::PhysicsEnginePtr pe = (world_->Physics());
  pe->SetMaxStepSize(req.time_step);
  pe->SetRealTimeUpdateRate(req.max_update_rate);
  pe->SetGravity(ignition::math::Vector3d(req.gravity.x,req.gravity.y,req.gravity.z));

  if (world_->Physics()->GetType() == "ode")
  {
    // stuff only works in ODE right now
    pe->SetAutoDisableFlag(req.ode_config.auto_disable_bodies);
#if GAZEBO_MAJOR_VERSION >= 3
    pe->SetParam("precon_iters", req.ode_config.sor_pgs_precon_iters);
    pe->SetParam("iters", req.ode_config.sor_pgs_iters);
    pe->SetParam("sor", req.ode_config.sor_pgs_w);
    pe->SetParam("cfm", req.ode_config.cfm);
    pe->SetParam("erp", req.ode_config.erp);
    pe->SetParam("contact_surface_layer",
        req.ode_config.contact_surface_layer);
    pe->SetParam("contact_max_correcting_vel",
        req.ode_config.contact_max_correcting_vel);
    pe->SetParam("max_contacts", req.ode_config.max_contacts);
#else
    pe->SetSORPGSPreconIters(req.ode_config.sor_pgs_precon_iters);
    pe->SetSORPGSIters(req.ode_config.sor_pgs_iters);
    pe->SetSORPGSW(req.ode_config.sor_pgs_w);
    pe->SetWorldCFM(req.ode_config.cfm);
    pe->SetWorldERP(req.ode_config.erp);
    pe->SetContactSurfaceLayer(req.ode_config.contact_surface_layer);
    pe->SetContactMaxCorrectingVel(req.ode_config.contact_max_correcting_vel);
    pe->SetMaxContacts(req.ode_config.max_contacts);
#endif

    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "physics engine updated";
  }
  // Let's just say it's just not ODE anymore now
  else if (world_->Physics()->GetType() == "opensim")
  {
    try
    {
      pe->SetParam("integrator_accuracy", req.opensim_config.integrator_accuracy);
      pe->SetParam("min_step_size", req.opensim_config.min_step_size);
      pe->SetParam("stiffness", req.opensim_config.stiffness);
      pe->SetParam("dissipation", req.opensim_config.dissipation);
      pe->SetParam("transitionVelocity", req.opensim_config.transitionVelocity);
      pe->SetParam("staticFriction", req.opensim_config.staticFriction);
      pe->SetParam("dynamicFriction", req.opensim_config.dynamicFriction);
      pe->SetParam("viscousFriction", req.opensim_config.viscousFriction);
    }
    catch (boost::bad_any_cast& ex)
    {
      gzerr << "Exception setting OpenSim physics engine parameters: " << ex.what() << std::endl;
    }

    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "physics engine updated";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR("ROS set_physics_properties service call does not yet support physics engine [%s].", world_->Physics()->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + world_->Physics()->GetType() + "]: set_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,
                                              gazebo_msgs::GetPhysicsProperties::Response &res)
{
  // supported updates
  res.time_step = world_->Physics()->GetMaxStepSize();
  res.pause = world_->IsPaused();
  res.max_update_rate = world_->Physics()->GetRealTimeUpdateRate();
  ignition::math::Vector3d gravity = world_->Gravity();
  res.gravity.x = gravity.X();
  res.gravity.y = gravity.Y();
  res.gravity.z = gravity.Z();

  // stuff only works in ODE right now
  if (world_->Physics()->GetType() == "ode")
  {
    res.ode_config.auto_disable_bodies =
      world_->Physics()->GetAutoDisableFlag();
#if GAZEBO_MAJOR_VERSION >= 3
    res.ode_config.sor_pgs_precon_iters = boost::any_cast<int>(
      world_->Physics()->GetParam("precon_iters"));
    res.ode_config.sor_pgs_iters = boost::any_cast<int>(
        world_->Physics()->GetParam("iters"));
    res.ode_config.sor_pgs_w = boost::any_cast<double>(
        world_->Physics()->GetParam("sor"));
    res.ode_config.contact_surface_layer = boost::any_cast<double>(
      world_->Physics()->GetParam("contact_surface_layer"));
    res.ode_config.contact_max_correcting_vel = boost::any_cast<double>(
      world_->Physics()->GetParam("contact_max_correcting_vel"));
    res.ode_config.cfm = boost::any_cast<double>(
        world_->Physics()->GetParam("cfm"));
    res.ode_config.erp = boost::any_cast<double>(
        world_->Physics()->GetParam("erp"));
    res.ode_config.max_contacts = boost::any_cast<int>(
      world_->Physics()->GetParam("max_contacts"));
#else
    res.ode_config.sor_pgs_precon_iters = world_->Physics()->GetSORPGSPreconIters();
    res.ode_config.sor_pgs_iters = world_->Physics()->GetSORPGSIters();
    res.ode_config.sor_pgs_w = world_->Physics()->GetSORPGSW();
    res.ode_config.contact_surface_layer = world_->Physics()->GetContactSurfaceLayer();
    res.ode_config.contact_max_correcting_vel = world_->Physics()->GetContactMaxCorrectingVel();
    res.ode_config.cfm = world_->Physics()->GetWorldCFM();
    res.ode_config.erp = world_->Physics()->GetWorldERP();
    res.ode_config.max_contacts = world_->Physics()->GetMaxContacts();
#endif

    res.success = true;
    res.status_message = "GetPhysicsProperties: got properties";
  }
  // Let's just say it's just not ODE anymore now
  else if (world_->Physics()->GetType() == "opensim")
  {
    try
    {
      res.opensim_config.integrator = boost::any_cast<std::string>(world_->Physics()->GetParam("integrator_type"));
      res.opensim_config.integrator_accuracy = boost::any_cast<double>(world_->Physics()->GetParam("integrator_accuracy"));
      res.opensim_config.min_step_size = boost::any_cast<double>(world_->Physics()->GetParam("min_step_size"));
      res.opensim_config.stiffness = boost::any_cast<double>(world_->Physics()->GetParam("stiffness"));
      res.opensim_config.dissipation = boost::any_cast<double>(world_->Physics()->GetParam("dissipation"));
      res.opensim_config.transitionVelocity = boost::any_cast<double>(world_->Physics()->GetParam("transitionVelocity"));
      res.opensim_config.staticFriction = boost::any_cast<double>(world_->Physics()->GetParam("staticFriction"));
      res.opensim_config.dynamicFriction = boost::any_cast<double>(world_->Physics()->GetParam("dynamicFriction"));
      res.opensim_config.viscousFriction = boost::any_cast<double>(world_->Physics()->GetParam("viscousFriction"));
    }
    catch (boost::bad_any_cast& ex)
    {
      gzerr << "Exception getting parameters for OpenSim physics engine: " << ex.what() << std::endl;
    }

    res.success = true;
    res.status_message = "GetPhysicsProperties: got properties (OpenSim)";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR("ROS get_physics_properties service call does not yet support physics engine [%s].", world_->Physics()->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + world_->Physics()->GetType() + "]: get_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::setJointProperties(gazebo_msgs::SetJointProperties::Request &req,
                                            gazebo_msgs::SetJointProperties::Response &res)
{
  /// @todo: current settings only allows for setting of 1DOF joints (e.g. HingeJoint and SliderJoint) correctly.
  gazebo::physics::JointPtr joint;
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
    if (joint) break;
  }

  if (!joint)
  {
    res.success = false;
    res.status_message = "SetJointProperties: joint not found";
    return true;
  }
  else
  {
    for(unsigned int i=0;i< req.ode_joint_config.damping.size();i++)
      joint->SetDamping(i,req.ode_joint_config.damping[i]);
#if GAZEBO_MAJOR_VERSION >= 4
    for(unsigned int i=0;i< req.ode_joint_config.hiStop.size();i++)
      joint->SetParam("hi_stop",i,req.ode_joint_config.hiStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.loStop.size();i++)
      joint->SetParam("lo_stop",i,req.ode_joint_config.loStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.erp.size();i++)
      joint->SetParam("erp",i,req.ode_joint_config.erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.cfm.size();i++)
      joint->SetParam("cfm",i,req.ode_joint_config.cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_erp.size();i++)
      joint->SetParam("stop_erp",i,req.ode_joint_config.stop_erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_cfm.size();i++)
      joint->SetParam("stop_cfm",i,req.ode_joint_config.stop_cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fudge_factor.size();i++)
      joint->SetParam("fudge_factor",i,req.ode_joint_config.fudge_factor[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fmax.size();i++)
      joint->SetParam("fmax",i,req.ode_joint_config.fmax[i]);
    for(unsigned int i=0;i< req.ode_joint_config.vel.size();i++)
      joint->SetParam("vel",i,req.ode_joint_config.vel[i]);
#else
    for(unsigned int i=0;i< req.ode_joint_config.hiStop.size();i++)
      joint->SetAttribute("hi_stop",i,req.ode_joint_config.hiStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.loStop.size();i++)
      joint->SetAttribute("lo_stop",i,req.ode_joint_config.loStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.erp.size();i++)
      joint->SetAttribute("erp",i,req.ode_joint_config.erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.cfm.size();i++)
      joint->SetAttribute("cfm",i,req.ode_joint_config.cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_erp.size();i++)
      joint->SetAttribute("stop_erp",i,req.ode_joint_config.stop_erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_cfm.size();i++)
      joint->SetAttribute("stop_cfm",i,req.ode_joint_config.stop_cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fudge_factor.size();i++)
      joint->SetAttribute("fudge_factor",i,req.ode_joint_config.fudge_factor[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fmax.size();i++)
      joint->SetAttribute("fmax",i,req.ode_joint_config.fmax[i]);
    for(unsigned int i=0;i< req.ode_joint_config.vel.size();i++)
      joint->SetAttribute("vel",i,req.ode_joint_config.vel[i]);
#endif

    res.success = true;
    res.status_message = "SetJointProperties: properties set";
    return true;
  }
}

bool GazeboRosApiPlugin::setModelState(gazebo_msgs::SetModelState::Request &req,
                                       gazebo_msgs::SetModelState::Response &res)
{
  ignition::math::Vector3d target_pos(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);

  if(req.model_state.scale.x == 0 || req.model_state.scale.y == 0 || req.model_state.scale.z == 0) {

    ROS_DEBUG("SetModelState: scale cannot be 0");
    res.success = false;
    res.status_message = "SetModelState: scale cannot be 0";
    return true;
  }

  ignition::math::Vector3d target_scale(req.model_state.scale.x,req.model_state.scale.y,req.model_state.scale.z);
  ignition::math::Quaterniond target_rot(req.model_state.pose.orientation.w,req.model_state.pose.orientation.x,req.model_state.pose.orientation.y,req.model_state.pose.orientation.z);
  target_rot.Normalize(); // eliminates invalid rotation (0, 0, 0, 0)
  ignition::math::Pose3d target_pose(target_pos,target_rot);
  ignition::math::Vector3d target_pos_dot(req.model_state.twist.linear.x,req.model_state.twist.linear.y,req.model_state.twist.linear.z);
  ignition::math::Vector3d target_rot_dot(req.model_state.twist.angular.x,req.model_state.twist.angular.y,req.model_state.twist.angular.z);

  boost::recursive_mutex::scoped_lock lock(scene_lock_);

  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_state.model_name);
  if (!model)
  {
    ROS_ERROR("Updating ModelState: model [%s] does not exist",req.model_state.model_name.c_str());
    res.success = false;
    res.status_message = "SetModelState: model does not exist";
    return true;
  }
  else
  {
    gazebo::physics::LinkPtr relative_entity = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.model_state.reference_frame));
    if (relative_entity)
    {
      ignition::math::Pose3d  frame_pose = relative_entity->WorldPose(); // - myBody->GetCoMPose();
      ignition::math::Vector3d frame_pos = frame_pose.Pos();
      ignition::math::Quaterniond frame_rot = frame_pose.Rot();

      //std::cout << " debug : " << relative_entity->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
      //target_pose = frame_pose + target_pose; // seems buggy, use my own
      target_pose.Pos() = model->WorldPose().Pos() + frame_rot.RotateVector(target_pos);
      target_pose.Rot() = frame_rot * target_pose.Rot();

      // Velocities should be commanded in the requested reference
      // frame, so we need to translate them to the world frame
      target_pos_dot = frame_rot.RotateVector(target_pos_dot);
      target_rot_dot = frame_rot.RotateVector(target_rot_dot);
    }
    /// @todo: FIXME map is really wrong, need to use tf here somehow
    else if (req.model_state.reference_frame == "" || req.model_state.reference_frame == "world" || req.model_state.reference_frame == "map" || req.model_state.reference_frame == "/map" )
    {
      ROS_DEBUG("Updating ModelState: reference frame is empty/world/map, usig inertial frame");
    }
    else
    {
      ROS_ERROR("Updating ModelState: for model[%s], specified reference frame entity [%s] does not exist",
                req.model_state.model_name.c_str(),req.model_state.reference_frame.c_str());
      res.success = false;
      res.status_message = "SetModelState: specified reference frame entity does not exist";
      return true;
    }

    //ROS_ERROR("target state: %f %f %f",target_pose.pos.x,target_pose.pos.y,target_pose.pos.z);
    bool is_paused = world_->IsPaused();
    world_->SetPaused(true);
    model->SetWorldPose(target_pose, true, true);//tell gzserver to publish the new pose on ~/pose/info

    model->SetScale(target_scale, true); //and the scale on ~/model/info

    world_->SetPaused(is_paused);
    //ignition::math::Pose3d p3d = model->WorldPose();
    //ROS_ERROR("model updated state: %f %f %f",p3d.pos.x,p3d.pos.y,p3d.pos.z);

    // set model velocity
    model->SetLinearVel(target_pos_dot);
    model->SetAngularVel(target_rot_dot);

    res.success = true;
    res.status_message = "SetModelState: set model state done";
    return true;
  }
}

void GazeboRosApiPlugin::updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state)
{
  gazebo_msgs::SetModelState::Response res;
  gazebo_msgs::SetModelState::Request req;
  req.model_state = *model_state;
  /*bool success =*/ setModelState(req,res);
}

bool GazeboRosApiPlugin::applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,
                                          gazebo_msgs::ApplyJointEffort::Response &res)
{
  gazebo::physics::JointPtr joint;
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
    if (joint)
    {
      GazeboRosApiPlugin::ForceJointJob* fjj = new GazeboRosApiPlugin::ForceJointJob;
      fjj->joint = joint;
      fjj->force = req.effort;
      fjj->start_time = req.start_time;
      if (fjj->start_time < ros::Time(world_->SimTime().Double()))
        fjj->start_time = ros::Time(world_->SimTime().Double());
      fjj->duration = req.duration;
      lock_.lock();
      force_joint_jobs_.push_back(fjj);
      lock_.unlock();

      res.success = true;
      res.status_message = "ApplyJointEffort: effort set";
      return true;
    }
  }

  res.success = false;
  res.status_message = "ApplyJointEffort: joint not found";
  return true;
}

bool GazeboRosApiPlugin::resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->Reset();
  return true;
}

bool GazeboRosApiPlugin::resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->ResetEntities(gazebo::physics::Base::MODEL);
  return true;
}

bool GazeboRosApiPlugin::pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->SetPaused(true);
  return true;
}

bool GazeboRosApiPlugin::unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->SetPaused(false);
  return true;
}

bool GazeboRosApiPlugin::clearJointForces(gazebo_msgs::JointRequest::Request &req,
                                          gazebo_msgs::JointRequest::Response &res)
{
  return clearJointForces(req.joint_name);
}
bool GazeboRosApiPlugin::clearJointForces(std::string joint_name)
{
  bool search = true;
  lock_.lock();
  while(search)
  {
    search = false;
    for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();++iter)
    {
      if ((*iter)->joint->GetName() == joint_name)
      {
        // found one, search through again
        search = true;
        delete (*iter);
        force_joint_jobs_.erase(iter);
        break;
      }
    }
  }
  lock_.unlock();
  return true;
}

bool GazeboRosApiPlugin::clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,
                                           gazebo_msgs::BodyRequest::Response &res)
{
  return clearBodyWrenches(req.body_name);
}
bool GazeboRosApiPlugin::clearBodyWrenches(std::string body_name)
{
  bool search = true;
  lock_.lock();
  while(search)
  {
    search = false;
    for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();++iter)
    {
      //ROS_ERROR("search %s %s",(*iter)->body->GetScopedName().c_str(), body_name.c_str());
      if ((*iter)->body->GetScopedName() == body_name)
      {
        // found one, search through again
        search = true;
        delete (*iter);
        wrench_body_jobs_.erase(iter);
        break;
      }
    }
  }
  lock_.unlock();
  return true;
}

bool GazeboRosApiPlugin::setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,
                                               gazebo_msgs::SetModelConfiguration::Response &res)
{
  std::string gazebo_model_name = req.model_name;

  // search for model with name
  gazebo::physics::ModelPtr gazebo_model = world_->ModelByName(req.model_name);
  if (!gazebo_model)
  {
    ROS_ERROR("SetModelConfiguration: model [%s] does not exist",gazebo_model_name.c_str());
    res.success = false;
    res.status_message = "SetModelConfiguration: model does not exist";
    return true;
  }

  if (req.joint_names.size() == req.joint_positions.size())
  {
    std::map<std::string, double> joint_position_map;
    for (unsigned int i = 0; i < req.joint_names.size(); i++)
    {
      joint_position_map[req.joint_names[i]] = req.joint_positions[i];
    }

    // make the service call to pause gazebo
    bool is_paused = world_->IsPaused();
    if (!is_paused) world_->SetPaused(true);

    gazebo_model->SetJointPositions(joint_position_map);

    // resume paused state before this call
    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "SetModelConfiguration: success";
    return true;
  }
  else
  {
    res.success = false;
    res.status_message = "SetModelConfiguration: joint name and position list have different lengths";
    return true;
  }
}

bool GazeboRosApiPlugin::setLinkState(gazebo_msgs::SetLinkState::Request &req,
                                      gazebo_msgs::SetLinkState::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_state.link_name));
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_state.reference_frame));
  if (!body)
  {
    ROS_ERROR("Updating LinkState: link [%s] does not exist",req.link_state.link_name.c_str());
    res.success = false;
    res.status_message = "SetLinkState: link does not exist";
    return true;
  }

  /// @todo: FIXME map is really wrong, unless using tf here somehow
  // get reference frame (body/model(link)) pose and
  // transform target pose to absolute world frame
  ignition::math::Vector3d target_pos(req.link_state.pose.position.x,req.link_state.pose.position.y,req.link_state.pose.position.z);
  ignition::math::Quaterniond target_rot(req.link_state.pose.orientation.w,req.link_state.pose.orientation.x,req.link_state.pose.orientation.y,req.link_state.pose.orientation.z);
  ignition::math::Pose3d target_pose(target_pos,target_rot);
  ignition::math::Vector3d target_linear_vel(req.link_state.twist.linear.x,req.link_state.twist.linear.y,req.link_state.twist.linear.z);
  ignition::math::Vector3d target_angular_vel(req.link_state.twist.angular.x,req.link_state.twist.angular.y,req.link_state.twist.angular.z);

  if (frame)
  {
    ignition::math::Pose3d  frame_pose = frame->WorldPose(); // - myBody->GetCoMPose();
    ignition::math::Vector3d frame_pos = frame_pose.Pos();
    ignition::math::Quaterniond frame_rot = frame_pose.Rot();

    //std::cout << " debug : " << frame->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
    //target_pose = frame_pose + target_pose; // seems buggy, use my own
    target_pose.Pos() = frame_pos + frame_rot.RotateVector(target_pos);
    target_pose.Rot() = frame_rot * target_pose.Rot();

    ignition::math::Vector3d frame_linear_vel = frame->WorldLinearVel();
    ignition::math::Vector3d frame_angular_vel = frame->WorldAngularVel();
    target_linear_vel -= frame_linear_vel;
    target_angular_vel -= frame_angular_vel;
  }
  else if (req.link_state.reference_frame == "" || req.link_state.reference_frame == "world" || req.link_state.reference_frame == "map" || req.link_state.reference_frame == "/map")
  {
    ROS_INFO("Updating LinkState: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    ROS_ERROR("Updating LinkState: reference_frame is not a valid link name");
    res.success = false;
    res.status_message = "SetLinkState: failed";
    return true;
  }

  //std::cout << " debug : " << target_pose << std::endl;
  //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());

  bool is_paused = world_->IsPaused();
  if (!is_paused) world_->SetPaused(true);
  body->SetWorldPose(target_pose);
  world_->SetPaused(is_paused);

  // set body velocity to desired twist
  body->SetLinearVel(target_linear_vel);
  body->SetAngularVel(target_angular_vel);

  res.success = true;
  res.status_message = "SetLinkState: success";
  return true;
}

void GazeboRosApiPlugin::updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state)
{
  gazebo_msgs::SetLinkState::Request req;
  gazebo_msgs::SetLinkState::Response res;
  req.link_state = *link_state;
  /*bool success = */ setLinkState(req,res);
}

void GazeboRosApiPlugin::transformWrench( ignition::math::Vector3d &target_force, ignition::math::Vector3d &target_torque,
                                          ignition::math::Vector3d reference_force, ignition::math::Vector3d reference_torque,
                                          ignition::math::Pose3d target_to_reference )
{
  // rotate force into target frame
  target_force = target_to_reference.Rot().RotateVector(reference_force);
  // rotate torque into target frame
  target_torque = target_to_reference.Rot().RotateVector(reference_torque);

  // target force is the refence force rotated by the target->reference transform
  target_torque = target_torque + target_to_reference.Pos().Cross(target_force);
}

bool GazeboRosApiPlugin::applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,
                                         gazebo_msgs::ApplyBodyWrench::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.body_name));
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.reference_frame));
  if (!body)
  {
    ROS_ERROR("ApplyBodyWrench: body [%s] does not exist",req.body_name.c_str());
    res.success = false;
    res.status_message = "ApplyBodyWrench: body does not exist";
    return true;
  }

  // target wrench
  ignition::math::Vector3d reference_force(req.wrench.force.x,req.wrench.force.y,req.wrench.force.z);
  ignition::math::Vector3d reference_torque(req.wrench.torque.x,req.wrench.torque.y,req.wrench.torque.z);
  ignition::math::Vector3d reference_point(req.reference_point.x,req.reference_point.y,req.reference_point.z);

  ignition::math::Vector3d target_force;
  ignition::math::Vector3d target_torque;

  /// shift wrench to body frame if a non-zero reference point is given
  ///   @todo: to be more general, should we make the reference point a reference pose?
  reference_torque = reference_torque + reference_point.Cross(reference_force);

  /// @todo: FIXME map is really wrong, need to use tf here somehow
  if (frame)
  {
    // get reference frame (body/model(body)) pose and
    // transform target pose to absolute world frame
    // @todo: need to modify wrench (target force and torque by frame)
    //        transform wrench from reference_point in reference_frame
    //        into the reference frame of the body
    //        first, translate by reference point to the body frame
    ignition::math::Pose3d target_to_reference = frame->WorldPose() - body->WorldPose();
    ROS_DEBUG("reference frame for applied wrench: [%f %f %f, %f %f %f]-[%f %f %f, %f %f %f]=[%f %f %f, %f %f %f]",
              body->WorldPose().Pos().X(),
              body->WorldPose().Pos().Y(),
              body->WorldPose().Pos().Z(),
              body->WorldPose().Rot().Euler().X(),
              body->WorldPose().Rot().Euler().Y(),
              body->WorldPose().Rot().Euler().Z(),
              frame->WorldPose().Pos().X(),
              frame->WorldPose().Pos().Y(),
              frame->WorldPose().Pos().Z(),
              frame->WorldPose().Rot().Euler().X(),
              frame->WorldPose().Rot().Euler().Y(),
              frame->WorldPose().Rot().Euler().Z(),
              target_to_reference.Pos().X(),
              target_to_reference.Pos().Y(),
              target_to_reference.Pos().Z(),
              target_to_reference.Rot().Euler().X(),
              target_to_reference.Rot().Euler().Y(),
              target_to_reference.Rot().Euler().Z()
              );
    transformWrench(target_force, target_torque, reference_force, reference_torque, target_to_reference);
    ROS_ERROR("wrench defined as [%s]:[%f %f %f, %f %f %f] --> applied as [%s]:[%f %f %f, %f %f %f]",
              frame->GetName().c_str(),
              reference_force.X(),
              reference_force.Y(),
              reference_force.Z(),
              reference_torque.X(),
              reference_torque.Y(),
              reference_torque.Z(),
              body->GetName().c_str(),
              target_force.X(),
              target_force.Y(),
              target_force.Z(),
              target_torque.X(),
              target_torque.Y(),
              target_torque.Z()
              );

  }
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_INFO("ApplyBodyWrench: reference_frame is empty/world/map, using inertial frame, transferring from body relative to inertial frame");
    // FIXME: transfer to inertial frame
    ignition::math::Pose3d target_to_reference = body->WorldPose();
    target_force = reference_force;
    target_torque = reference_torque;

  }
  else
  {
    ROS_ERROR("ApplyBodyWrench: reference_frame is not a valid link name");
    res.success = false;
    res.status_message = "ApplyBodyWrench: reference_frame not found";
    return true;
  }

  // apply wrench
  // schedule a job to do below at appropriate times:
  // body->SetForce(force)
  // body->SetTorque(torque)
  GazeboRosApiPlugin::WrenchBodyJob* wej = new GazeboRosApiPlugin::WrenchBodyJob;
  wej->body = body;
  wej->force = target_force;
  wej->torque = target_torque;
  wej->start_time = req.start_time;
  if (wej->start_time < ros::Time(world_->SimTime().Double()))
    wej->start_time = ros::Time(world_->SimTime().Double());
  wej->duration = req.duration;
  lock_.lock();
  wrench_body_jobs_.push_back(wej);
  lock_.unlock();

  res.success = true;
  res.status_message = "";
  return true;
}

bool GazeboRosApiPlugin::isURDF(std::string model_xml)
{
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("robot"))
    return true;
  else
    return false;
}

bool GazeboRosApiPlugin::isSDF(std::string model_xml)
{
  // FIXME: very crude check
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("gazebo") ||
      doc_in.FirstChild("sdf")) // sdf
    return true;
  else
    return false;
}

void GazeboRosApiPlugin::wrenchBodySchedulerSlot()
{
  // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
  // boost::recursive_mutex::scoped_lock lock(*world->GetMDMutex());
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
  {
    // check times and apply wrench if necessary
    if (ros::Time(world_->SimTime().Double()) >= (*iter)->start_time)
      if (ros::Time(world_->SimTime().Double()) <= (*iter)->start_time+(*iter)->duration ||
          (*iter)->duration.toSec() < 0.0)
      {
        if ((*iter)->body) // if body exists
        {
          (*iter)->body->SetForce((*iter)->force);
          (*iter)->body->SetTorque((*iter)->torque);
        }
        else
          (*iter)->duration.fromSec(0.0); // mark for delete
      }

    if (ros::Time(world_->SimTime().Double()) > (*iter)->start_time+(*iter)->duration &&
        (*iter)->duration.toSec() >= 0.0)
    {
      // remove from queue once expires
      delete (*iter);
      iter = wrench_body_jobs_.erase(iter);
    }
    else
      ++iter;
  }
  lock_.unlock();
}

void GazeboRosApiPlugin::forceJointSchedulerSlot()
{
  // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
  // boost::recursive_mutex::scoped_lock lock(*world->GetMDMutex());
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();)
  {
    // check times and apply force if necessary
    if (ros::Time(world_->SimTime().Double()) >= (*iter)->start_time)
      if (ros::Time(world_->SimTime().Double()) <= (*iter)->start_time+(*iter)->duration ||
          (*iter)->duration.toSec() < 0.0)
      {
        if ((*iter)->joint) // if joint exists
          (*iter)->joint->SetForce(0,(*iter)->force);
        else
          (*iter)->duration.fromSec(0.0); // mark for delete
      }

    if (ros::Time(world_->SimTime().Double()) > (*iter)->start_time+(*iter)->duration &&
        (*iter)->duration.toSec() >= 0.0)
    {
      // remove from queue once expires
      iter = force_joint_jobs_.erase(iter);
    }
    else
      ++iter;
  }
  lock_.unlock();
}

void GazeboRosApiPlugin::publishSimTime(const boost::shared_ptr<gazebo::msgs::WorldStatistics const> &msg)
{
  ROS_ERROR("CLOCK2");
  gazebo::common::Time sim_time = world_->SimTime();
  if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).Double() < 1.0/pub_clock_frequency_)
    return;

  gazebo::common::Time currentTime = gazebo::msgs::Convert( msg->sim_time() );
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(currentTime.Double());
  //  publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}
void GazeboRosApiPlugin::publishSimTime()
{
  gazebo::common::Time sim_time = world_->SimTime();
  if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).Double() < 1.0/pub_clock_frequency_)
    return;

  gazebo::common::Time currentTime = world_->SimTime();
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(currentTime.Double());
  //  publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}

void GazeboRosApiPlugin::publishLinkStates()
{
  gazebo_msgs::LinkStates link_states;

  // fill link_states
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);

    for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      if (body)
      {
        link_states.name.push_back(body->GetScopedName());
        geometry_msgs::Pose pose;
        ignition::math::Pose3d  body_pose = body->WorldPose(); // - myBody->GetCoMPose();
        ignition::math::Vector3d pos = body_pose.Pos();
        ignition::math::Quaterniond rot = body_pose.Rot();
        pose.position.x = pos.X();
        pose.position.y = pos.Y();
        pose.position.z = pos.Z();
        pose.orientation.w = rot.W();
        pose.orientation.x = rot.X();
        pose.orientation.y = rot.Y();
        pose.orientation.z = rot.Z();
        link_states.pose.push_back(pose);
        ignition::math::Vector3d linear_vel  = body->WorldLinearVel();
        ignition::math::Vector3d angular_vel = body->WorldAngularVel();
        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel.X();
        twist.linear.y = linear_vel.Y();
        twist.linear.z = linear_vel.Z();
        twist.angular.x = angular_vel.X();
        twist.angular.y = angular_vel.Y();
        twist.angular.z = angular_vel.Z();
        link_states.twist.push_back(twist);
        // accelerations
        ignition::math::Vector3d linear_accel = body->WorldLinearAccel();
        ignition::math::Vector3d angular_accel = body->WorldAngularAccel();
        twist.linear.x = linear_accel.X();
        twist.linear.y = linear_accel.Y();
        twist.linear.z = linear_accel.Z();
        twist.angular.x = angular_accel.X();
        twist.angular.y = angular_accel.Y();
        twist.angular.z = angular_accel.Z();
        link_states.accel.push_back(twist);
        // forces
        ignition::math::Vector3d force = body->WorldForce();
        ignition::math::Vector3d torque = body->WorldTorque();
        twist.linear.x = force.X();
        twist.linear.y = force.Y();
        twist.linear.z = force.Z();
        twist.angular.x = torque.X();
        twist.angular.y = torque.Y();
        twist.angular.z = torque.Z();
        link_states.force.push_back(twist);
      }
    }
  }

  pub_link_states_.publish(link_states);
}

void GazeboRosApiPlugin::publishModelStates()
{
  gazebo_msgs::ModelStates model_states;

  // fill model_states
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);
    // push information of the model if is not Static
    if(!model->IsStatic())
    {
        model_states.name.push_back(model->GetName());
        geometry_msgs::Pose pose;
        ignition::math::Pose3d  model_pose = model->WorldPose(); // - myBody->GetCoMPose();
        ignition::math::Vector3d pos = model_pose.Pos();
        ignition::math::Quaterniond rot = model_pose.Rot();
        pose.position.x = pos.X();
        pose.position.y = pos.Y();
        pose.position.z = pos.Z();
        pose.orientation.w = rot.W();
        pose.orientation.x = rot.X();
        pose.orientation.y = rot.Y();
        pose.orientation.z = rot.Z();
        model_states.pose.push_back(pose);

        geometry_msgs::Vector3 scaleMsg;
        ignition::math::Vector3d scale = model->Scale();
        scaleMsg.x = scale.X();
        scaleMsg.y = scale.Y();
        scaleMsg.z = scale.Z();
        model_states.scale.push_back(scaleMsg);

        ignition::math::Vector3d linear_vel  = model->WorldLinearVel();
        ignition::math::Vector3d angular_vel = model->WorldAngularVel();
        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel.X();
        twist.linear.y = linear_vel.Y();
        twist.linear.z = linear_vel.Z();
        twist.angular.x = angular_vel.X();
        twist.angular.y = angular_vel.Y();
        twist.angular.z = angular_vel.Z();
        model_states.twist.push_back(twist);
    }
  }

  pub_model_states_.publish(model_states);
}

void GazeboRosApiPlugin::onContactPointDataReceived(ConstContactsPtr &_msg)
{
  gazebo_msgs::ContactsState contacts_state;  // ROS
  for (int i = 0; i < _msg->contact_size(); i++)
  {
    gazebo::msgs::Contact gz_contact_msg = _msg->contact(i);

    gazebo_msgs::ContactState contact_msg;
    contact_msg.collision1_name = gz_contact_msg.collision1();
    contact_msg.collision2_name = gz_contact_msg.collision2();

    for (size_t i = 0; i < gz_contact_msg.normal_size(); ++i)
    {
      geometry_msgs::Vector3 normal_vec;
      normal_vec.x = gz_contact_msg.normal(i).x();
      normal_vec.y = gz_contact_msg.normal(i).y();
      normal_vec.z = gz_contact_msg.normal(i).z();
      contact_msg.contact_normals.push_back(normal_vec);
    }

    for (size_t i = 0; i < gz_contact_msg.position_size(); ++i)
    {
      geometry_msgs::Vector3 position_vec;
      position_vec.x = gz_contact_msg.position(i).x();
      position_vec.y = gz_contact_msg.position(i).y();
      position_vec.z = gz_contact_msg.position(i).z();
      contact_msg.contact_positions.push_back(position_vec);
    }

    for (size_t i = 0; i < gz_contact_msg.depth_size(); ++i)
    {
      contact_msg.depths.push_back(gz_contact_msg.depth(i));
    }

    ignition::math::Vector3d total_force = ignition::math::Vector3d::Zero;
    ignition::math::Vector3d total_torque = ignition::math::Vector3d::Zero;
    for (size_t i = 0; i < gz_contact_msg.wrench_size(); ++i)
    {
      geometry_msgs::Wrench wrench;
      geometry_msgs::Vector3 force_vec;
      force_vec.x = gz_contact_msg.wrench(i).body_1_wrench().force().x();
      force_vec.y = gz_contact_msg.wrench(i).body_1_wrench().force().y();
      force_vec.z = gz_contact_msg.wrench(i).body_1_wrench().force().z();
      wrench.force = force_vec;

      geometry_msgs::Vector3 torque_vec;
      torque_vec.x = gz_contact_msg.wrench(i).body_1_wrench().torque().x();
      torque_vec.y = gz_contact_msg.wrench(i).body_1_wrench().torque().y();
      torque_vec.z = gz_contact_msg.wrench(i).body_1_wrench().torque().z();
      wrench.torque = torque_vec;

      contact_msg.wrenches.push_back(wrench);

      total_force += ignition::math::Vector3d(
        gz_contact_msg.wrench(i).body_1_wrench().force().x(),
        gz_contact_msg.wrench(i).body_1_wrench().force().y(),
        gz_contact_msg.wrench(i).body_1_wrench().force().z()
      );
      total_torque += ignition::math::Vector3d(
        gz_contact_msg.wrench(i).body_1_wrench().torque().x(),
        gz_contact_msg.wrench(i).body_1_wrench().torque().y(),
        gz_contact_msg.wrench(i).body_1_wrench().torque().z()
      );
    }
    contact_msg.total_wrench.force.x = total_force.X();
    contact_msg.total_wrench.force.y = total_force.Y();
    contact_msg.total_wrench.force.z = total_force.Z();
    contact_msg.total_wrench.torque.x = total_torque.X();
    contact_msg.total_wrench.torque.y = total_torque.Y();
    contact_msg.total_wrench.torque.z = total_torque.Z();

    contacts_state.states.push_back(contact_msg);
  }

  pub_contact_data_.publish(contacts_state);
}

void GazeboRosApiPlugin::physicsReconfigureCallback(gazebo_ros::PhysicsConfig &config, uint32_t level)
{
  if (!physics_reconfigure_initialized_)
  {
    gazebo_msgs::GetPhysicsProperties srv;
    physics_reconfigure_get_client_.call(srv);

    config.time_step                   = srv.response.time_step;
    config.max_update_rate             = srv.response.max_update_rate;
    config.gravity_x                   = srv.response.gravity.x;
    config.gravity_y                   = srv.response.gravity.y;
    config.gravity_z                   = srv.response.gravity.z;
    config.auto_disable_bodies         = srv.response.ode_config.auto_disable_bodies;
    config.sor_pgs_precon_iters        = srv.response.ode_config.sor_pgs_precon_iters;
    config.sor_pgs_iters               = srv.response.ode_config.sor_pgs_iters;
    config.sor_pgs_rms_error_tol       = srv.response.ode_config.sor_pgs_rms_error_tol;
    config.sor_pgs_w                   = srv.response.ode_config.sor_pgs_w;
    config.contact_surface_layer       = srv.response.ode_config.contact_surface_layer;
    config.contact_max_correcting_vel  = srv.response.ode_config.contact_max_correcting_vel;
    config.cfm                         = srv.response.ode_config.cfm;
    config.erp                         = srv.response.ode_config.erp;
    config.max_contacts                = srv.response.ode_config.max_contacts;
    physics_reconfigure_initialized_ = true;
  }
  else
  {
    bool changed = false;
    gazebo_msgs::GetPhysicsProperties srv;
    physics_reconfigure_get_client_.call(srv);

    // check for changes
    if (config.time_step                      != srv.response.time_step)                                 changed = true;
    if (config.max_update_rate                != srv.response.max_update_rate)                           changed = true;
    if (config.gravity_x                      != srv.response.gravity.x)                                 changed = true;
    if (config.gravity_y                      != srv.response.gravity.y)                                 changed = true;
    if (config.gravity_z                      != srv.response.gravity.z)                                 changed = true;
    if (config.auto_disable_bodies            != srv.response.ode_config.auto_disable_bodies)            changed = true;
    if ((uint32_t)config.sor_pgs_precon_iters != srv.response.ode_config.sor_pgs_precon_iters)           changed = true;
    if ((uint32_t)config.sor_pgs_iters        != srv.response.ode_config.sor_pgs_iters)                  changed = true;
    if (config.sor_pgs_rms_error_tol          != srv.response.ode_config.sor_pgs_rms_error_tol)          changed = true;
    if (config.sor_pgs_w                      != srv.response.ode_config.sor_pgs_w)                      changed = true;
    if (config.contact_surface_layer          != srv.response.ode_config.contact_surface_layer)          changed = true;
    if (config.contact_max_correcting_vel     != srv.response.ode_config.contact_max_correcting_vel)     changed = true;
    if (config.cfm                            != srv.response.ode_config.cfm)                            changed = true;
    if (config.erp                            != srv.response.ode_config.erp)                            changed = true;
    if ((uint32_t)config.max_contacts         != srv.response.ode_config.max_contacts)                   changed = true;

    if (changed)
    {
      // pause simulation if requested
      gazebo_msgs::SetPhysicsProperties srv;
      srv.request.time_step                             = config.time_step                   ;
      srv.request.max_update_rate                       = config.max_update_rate             ;
      srv.request.gravity.x                             = config.gravity_x                   ;
      srv.request.gravity.y                             = config.gravity_y                   ;
      srv.request.gravity.z                             = config.gravity_z                   ;
      srv.request.ode_config.auto_disable_bodies        = config.auto_disable_bodies         ;
      srv.request.ode_config.sor_pgs_precon_iters       = config.sor_pgs_precon_iters        ;
      srv.request.ode_config.sor_pgs_iters              = config.sor_pgs_iters               ;
      srv.request.ode_config.sor_pgs_rms_error_tol      = config.sor_pgs_rms_error_tol       ;
      srv.request.ode_config.sor_pgs_w                  = config.sor_pgs_w                   ;
      srv.request.ode_config.contact_surface_layer      = config.contact_surface_layer       ;
      srv.request.ode_config.contact_max_correcting_vel = config.contact_max_correcting_vel  ;
      srv.request.ode_config.cfm                        = config.cfm                         ;
      srv.request.ode_config.erp                        = config.erp                         ;
      srv.request.ode_config.max_contacts               = config.max_contacts                ;
      physics_reconfigure_set_client_.call(srv);
      ROS_INFO("physics dynamics reconfigure update complete");
    }
    ROS_INFO("physics dynamics reconfigure complete");
  }
}

void GazeboRosApiPlugin::physicsReconfigureThread()
{
  physics_reconfigure_set_client_ = nh_->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
  physics_reconfigure_get_client_ = nh_->serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");

  // Wait until the rest of this plugin is loaded and the services are being offered
  physics_reconfigure_set_client_.waitForExistence();
  physics_reconfigure_get_client_.waitForExistence();

  physics_reconfigure_srv_.reset(new dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig>());

  physics_reconfigure_callback_ = boost::bind(&GazeboRosApiPlugin::physicsReconfigureCallback, this, _1, _2);
  physics_reconfigure_srv_->setCallback(physics_reconfigure_callback_);

  ROS_INFO("Physics dynamic reconfigure ready.");
}

void GazeboRosApiPlugin::stripXmlDeclaration(std::string &model_xml)
{
  // incoming robot model string is a string containing a Gazebo Model XML
  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
  /// @todo: does tinyxml have functionality for this?
  /// @todo: should gazebo take care of the declaration?
  std::string open_bracket("<?");
  std::string close_bracket("?>");
  size_t pos1 = model_xml.find(open_bracket,0);
  size_t pos2 = model_xml.find(close_bracket,0);
  if (pos1 != std::string::npos && pos2 != std::string::npos)
    model_xml.replace(pos1,pos2-pos1+2,std::string(""));
}

void GazeboRosApiPlugin::updateSDFAttributes(TiXmlDocument &gazebo_entity_xml, std::string model_name,
                                             ignition::math::Vector3d initial_xyz, ignition::math::Quaterniond initial_q)
{
  // This function can handle both regular SDF files and <include> SDFs that are used with the
  // Gazebo Model Database

  TiXmlElement* pose_element; // This is used by both reguar and database SDFs

  // Check SDF for requires SDF element
  TiXmlElement* gazebo_tixml = gazebo_entity_xml.FirstChildElement("sdf");
  if (!gazebo_tixml)
  {
    ROS_WARN("Could not find <sdf> element in sdf, so name and initial position cannot be applied");
    return;
  }

  // Check SDF for optional model element. May not have one
  TiXmlElement* model_tixml = gazebo_tixml->FirstChildElement("model");
  if (model_tixml)
  {
    // Update model name
    if (model_tixml->Attribute("name") != NULL)
    {
      // removing old model name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name",model_name);
  }
  else
  {
    // Check SDF for world element
    TiXmlElement* world_tixml = gazebo_tixml->FirstChildElement("world");
    if (!world_tixml)
    {
      ROS_WARN("Could not find <model> or <world> element in sdf, so name and initial position cannot be applied");
      return;
    }
    // If not <model> element, check SDF for required include element
    model_tixml = world_tixml->FirstChildElement("include");
    if (!model_tixml)
    {
      ROS_WARN("Could not find <include> element in sdf, so name and initial position cannot be applied");
      return;
    }

    // Check for name element
    TiXmlElement* name_tixml = model_tixml->FirstChildElement("name");
    if (!name_tixml)
    {
      // Create the name element
      name_tixml = new TiXmlElement("name");
      model_tixml->LinkEndChild(name_tixml);
    }

    // Set the text within the name element
    TiXmlText* text = new TiXmlText(model_name);
    name_tixml->LinkEndChild( text );
  }


  // Check for the pose element
  pose_element = model_tixml->FirstChildElement("pose");
  ignition::math::Pose3d model_pose;

  // Create the pose element if it doesn't exist
  // Remove it if it exists, since we are inserting a new one
  if (pose_element)
  {
    // save pose_element in ignition::math::Pose3d and remove child
    model_pose = this->parsePose(pose_element->GetText());
    model_tixml->RemoveChild(pose_element);
  }

  // Set and link the pose element after adding initial pose
  {
    // add pose_element Pose to initial pose
    ignition::math::Pose3d new_model_pose = model_pose + ignition::math::Pose3d(initial_xyz, initial_q);

    // Create the string of 6 numbers
    std::ostringstream pose_stream;
    ignition::math::Vector3d model_rpy = new_model_pose.Rot().Euler(); // convert to Euler angles for Gazebo XML
    pose_stream << new_model_pose.Pos().X() << " " << new_model_pose.Pos().Y() << " " << new_model_pose.Pos().Z() << " "
                << model_rpy.X() << " " << model_rpy.Y() << " " << model_rpy.Z();

    // Add value to pose element
    TiXmlText* text = new TiXmlText(pose_stream.str());
    TiXmlElement* new_pose_element = new TiXmlElement("pose");
    new_pose_element->LinkEndChild(text);
    model_tixml->LinkEndChild(new_pose_element);
  }
}

ignition::math::Pose3d GazeboRosApiPlugin::parsePose(const std::string &str)
{
  std::vector<std::string> pieces;
  std::vector<double> vals;

  boost::split(pieces, str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        sdferr << "xml key [" << str
          << "][" << i << "] value [" << pieces[i]
          << "] is not a valid double from a 3-tuple\n";
        return ignition::math::Pose3d();
      }
    }
  }

  if (vals.size() == 6)
    return ignition::math::Pose3d(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
  else
  {
    ROS_ERROR("Beware: failed to parse string [%s] as ignition::math::Pose3d, returning zeros.", str.c_str());
    return ignition::math::Pose3d();
  }
}

ignition::math::Vector3d GazeboRosApiPlugin::parseVector3(const std::string &str)
{
  std::vector<std::string> pieces;
  std::vector<double> vals;

  boost::split(pieces, str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        sdferr << "xml key [" << str
          << "][" << i << "] value [" << pieces[i]
          << "] is not a valid double from a 3-tuple\n";
        return ignition::math::Vector3d();
      }
    }
  }

  if (vals.size() == 3)
    return ignition::math::Vector3d(vals[0], vals[1], vals[2]);
  else
  {
    ROS_ERROR("Beware: failed to parse string [%s] as ignition::math::Vector3d, returning zeros.", str.c_str());
    return ignition::math::Vector3d();
  }
}

void GazeboRosApiPlugin::updateURDFModelPose(TiXmlDocument &gazebo_entity_xml, ignition::math::Vector3d initial_xyz, ignition::math::Quaterniond initial_q)
{
  TiXmlElement* model_tixml = (gazebo_entity_xml.FirstChildElement("robot"));
  if (model_tixml)
  {
    // replace initial pose of robot
    // find first instance of xyz and rpy, replace with initial pose
    TiXmlElement* origin_key = model_tixml->FirstChildElement("origin");

    if (!origin_key)
    {
      origin_key = new TiXmlElement("origin");
      model_tixml->LinkEndChild(origin_key);
    }

    ignition::math::Vector3d xyz;
    ignition::math::Vector3d rpy;
    if (origin_key->Attribute("xyz"))
    {
      xyz = this->parseVector3(origin_key->Attribute("xyz"));
      origin_key->RemoveAttribute("xyz");
    }
    if (origin_key->Attribute("rpy"))
    {
      rpy = this->parseVector3(origin_key->Attribute("rpy"));
      origin_key->RemoveAttribute("rpy");
    }

    // add xyz, rpy to initial pose
    ignition::math::Pose3d model_pose = ignition::math::Pose3d(xyz, ignition::math::Quaterniond(rpy))
                                      + ignition::math::Pose3d(initial_xyz, initial_q);

    std::ostringstream xyz_stream;
    xyz_stream << model_pose.Pos().X() << " " << model_pose.Pos().Y() << " " << model_pose.Pos().Z();

    std::ostringstream rpy_stream;
    ignition::math::Vector3d model_rpy = model_pose.Rot().Euler(); // convert to Euler angles for Gazebo XML
    rpy_stream << model_rpy.X() << " " << model_rpy.Y() << " " << model_rpy.Z();

    origin_key->SetAttribute("xyz",xyz_stream.str());
    origin_key->SetAttribute("rpy",rpy_stream.str());
  }
  else
    ROS_WARN("could not find <model> element in sdf, so name and initial position is not applied");
}

void GazeboRosApiPlugin::updateURDFName(TiXmlDocument &gazebo_entity_xml, std::string model_name)
{
  TiXmlElement* model_tixml = gazebo_entity_xml.FirstChildElement("robot");
  // replace model name if one is specified by the user
  if (model_tixml)
  {
    if (model_tixml->Attribute("name") != NULL)
    {
      // removing old model name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name",model_name);
  }
  else
    ROS_WARN("could not find <robot> element in URDF, name not replaced");
}

void GazeboRosApiPlugin::walkChildAddRobotNamespace(TiXmlNode* robot_xml,
                                                    std::string &entity_namespace)
{
  TiXmlNode* child = 0;
  child = robot_xml->IterateChildren(child);
  while (child != NULL)
  {
    if (child->ValueStr().find(std::string("plugin")) == 0)
    {
      if (child->FirstChildElement("robotNamespace") == NULL)
      {
        TiXmlElement* child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        while (child_elem)
        {
          child->ToElement()->RemoveChild(child_elem);
          child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        }
        TiXmlElement* key = new TiXmlElement("robotNamespace");
        TiXmlText* val = new TiXmlText(entity_namespace);
        key->LinkEndChild(val);
        child->ToElement()->LinkEndChild(key);
      }
    }
    walkChildAddRobotNamespace(child, entity_namespace);
    child = robot_xml->IterateChildren(child);
  }
}

bool GazeboRosApiPlugin::spawnAndConform(TiXmlDocument &gazebo_entity_xml, std::string entity_name,
                                         gazebo_msgs::SpawnEntity::Response &res)
{

  std::string entity_type = gazebo_entity_xml.RootElement()->FirstChild()->Value();
  // Convert the entity type to lower case
  std::transform(entity_type.begin(), entity_type.end(), entity_type.begin(), ::tolower);

  bool isLight = (entity_type == "light");

  // push to factory iface
  std::ostringstream stream;
  stream << gazebo_entity_xml;
  std::string gazebo_entity_xml_string = stream.str();
  ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_entity_xml_string.c_str());

  // publish to factory topic
  gazebo::msgs::Factory msg;
  gazebo::msgs::Init(msg, "spawn_model");
  msg.set_sdf( gazebo_entity_xml_string );

  //ROS_ERROR("attempting to spawn model name [%s] [%s]", entity_name.c_str(),gazebo_entity_xml_string.c_str());

  // FIXME: should use entity_info or add lock to World::receiveMutex
  // looking for Model to see if it exists already
  /* gazebo::msgs::Request *entity_info_msg = gazebo::msgs::CreateRequest("entity_info", entity_name);
  request_pub_->Publish(*entity_info_msg,true); */
  this->nrpPublishRequest("entity_info", entity_name);
  // todo: should wait for response response_sub_, check to see that if _msg->response == "nonexistant"

  gazebo::physics::ModelPtr model = world_->ModelByName(entity_name);
  gazebo::physics::LightPtr light = world_->LightByName(entity_name);
  if ((isLight && light != nullptr) || (model != nullptr))
  {
    ROS_ERROR("SpawnEntity: Failure - entity name %s already exist.",entity_name.c_str());
    res.success = false;
    res.status_message = "SpawnEntity: Failure - entity already exists.";
    return true;
  }

  #if GAZEBO_MAJOR_VERSION > 6
    // for Gazebo 7 and up, use a different method to spawn lights
    if (isLight)
    {
      // Publish the light message to spawn the light (Gazebo 7 and up)
      sdf::SDF sdf_light;
      sdf_light.SetFromString(gazebo_entity_xml_string);
      gazebo::msgs::Light msg = gazebo::msgs::LightFromSDF(sdf_light.Root()->GetElement("light"));
      msg.set_name(entity_name);
      nrp_factory_light_pub_->Publish(msg);
    }
    else
  #else
    // for Gazebo 6 and lower, use the standard factory to spawn all object
    if (true)
  #endif
  {
  // Publish the factory message
  factory_pub_->Publish(msg);
  }
  /// FIXME: should change publish to direct invocation World::LoadModel() and/or
  ///        change the poll for Model existence to common::Events based check.

  /// \brief poll and wait, verify that the model is spawned within Hardcoded 10 seconds
  ros::Duration model_spawn_timeout(10.0);
  ros::Time timeout = ros::Time::now() + model_spawn_timeout;

  while (ros::ok())
  {
    if (ros::Time::now() > timeout)
    {
      res.success = false;
      res.status_message = std::string("SpawnEntity: Model pushed to spawn queue, but spawn service")
        + std::string(" timed out waiting for model to appear in simulation under the name ")
        + entity_name;
      return true;
    }

    {
      //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
      if ((isLight && world_->LightByName(entity_name) != nullptr)
          || (world_->ModelByName(entity_name) != nullptr))
        break;
    }

    ROS_DEBUG_STREAM_ONCE_NAMED("api_plugin","Waiting for " << timeout - ros::Time::now()
      << " for entity " << entity_name << " to spawn");

    usleep(2000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("SpawnEntity: Successfully spawned entity");
  return true;
}

// ===================================================
// BEGIN Custom NRP public methods implementation
// ===================================================


void GazeboRosApiPlugin::nrpAdvertiseServices() {
  // Advertise more services on the custom queue
  std::string advance_simulation_service_name("advance_simulation");
  ros::AdvertiseServiceOptions advance_simulation_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::AdvanceSimulation>(
                                                               advance_simulation_service_name,
                                                               boost::bind(&GazeboRosApiPlugin::nrpAdvanceSimulation,this,_1,_2),
                                                               ros::VoidPtr(), &gazebo_queue_);
  nrp_advance_simulation_service_ = nh_->advertiseService(advance_simulation_aso);

  // Advertise more services on the custom queue
  std::string reset_sim_time_service_name("reset_sim_time");
  ros::AdvertiseServiceOptions reset_sim_time_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_sim_time_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpResetSimTime,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_reset_sim_time_service_ = nh_->advertiseService(reset_sim_time_aso);

  // Advertise more services on the custom queue
  std::string reset_sim_service_name("reset_sim");
  ros::AdvertiseServiceOptions reset_sim_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_sim_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpResetSim,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_reset_sim_service_ = nh_->advertiseService(reset_sim_aso);

  // Advertise more services on the custom queue
  std::string end_world_service_name("end_world");
  ros::AdvertiseServiceOptions end_world_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          end_world_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpEndWorld,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_end_world_service_ = nh_->advertiseService(end_world_aso);

  // Advertise more services on the custom queue
  std::string delete_lights_service_name("delete_lights");
  ros::AdvertiseServiceOptions delete_lights_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          delete_lights_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpDeleteLights,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_delete_lights_service_ = nh_->advertiseService(delete_lights_aso);

  // Advertise more services on the custom queue
  std::string delete_light_service_name("delete_light");
  ros::AdvertiseServiceOptions delete_light_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteLight>(
                                                          delete_light_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpDeleteLight,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_delete_light_service_ = nh_->advertiseService(delete_light_aso);

  // Advertise more services on the custom queue
  std::string get_object_properties_service_name("get_visual_properties");
  ros::AdvertiseServiceOptions get_object_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetVisualProperties>(
                                                          get_object_properties_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpGetVisualProperties,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_get_object_properties_service_ = nh_->advertiseService(get_object_properties_aso);

  // Advertise more services on the custom queue
  std::string set_object_properties_service_name("set_visual_properties");
  ros::AdvertiseServiceOptions set_object_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetVisualProperties>(
                                                          set_object_properties_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpSetVisualProperties,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_set_object_properties_service_ = nh_->advertiseService(set_object_properties_aso);

  // Advertise more services on the custom queue
  std::string get_light_properties_service_name("get_light_properties");
  ros::AdvertiseServiceOptions get_light_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLightProperties>(
                                                          get_light_properties_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpGetLightProperties,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_get_light_properties_service_ = nh_->advertiseService(get_light_properties_aso);

  // Advertise more services on the custom queue
  std::string get_lights_name_service_name("get_lights_name");
  ros::AdvertiseServiceOptions get_lights_name_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLightsName>(
                                                          get_lights_name_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpGetLightsName,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_get_lights_name_service_ = nh_->advertiseService(get_lights_name_aso);

  // Advertise more services on the custom queue
  std::string set_light_properties_service_name("set_light_properties");
  ros::AdvertiseServiceOptions set_light_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLightProperties>(
                                                          set_light_properties_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpSetLightProperties,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_set_light_properties_service_ = nh_->advertiseService(set_light_properties_aso);

  // Allows SDF export via ROS
  std::string export_world_sdf_service_name("export_world_sdf");
  ros::AdvertiseServiceOptions export_world_sdf_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::ExportWorldSDF>(
                                                          export_world_sdf_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpExportWorldSDF,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_export_world_sdf_service_ = nh_->advertiseService(export_world_sdf_aso);

  // Wait for gserver rendering environment (if necessary) to be fully loaded for use by sensors/etc.
  std::string wait_for_rendering_service_name("wait_for_rendering");
  ros::AdvertiseServiceOptions wait_for_rendering_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          wait_for_rendering_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpWaitForRendering,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_wait_for_rendering_service_ = nh_->advertiseService(wait_for_rendering_aso);

  // allow request of scene info
  std::string request_scene_info_name("request_scene_info");
  ros::AdvertiseServiceOptions request_scene_info_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          request_scene_info_name,
                                                          boost::bind(&GazeboRosApiPlugin::nrpRequestSceneInfo,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  nrp_request_scene_info_service_ = nh_->advertiseService(request_scene_info_aso);

  // Sets the parameters for the sensor noise model used
  std::string set_sensor_noise_properties_service_name("set_sensor_noise_properties");
  ros::AdvertiseServiceOptions set_sensor_noise_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetSensorNoiseProperties>(
                                                                   set_sensor_noise_properties_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::nrpSetSensorNoiseProperties,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  nrp_set_sensor_noise_properties_service_ = nh_->advertiseService(set_sensor_noise_properties_aso);

  // Gets the parameters for the sensor noise model used
  std::string get_sensor_noise_properties_service_name("get_sensor_noise_properties");
  ros::AdvertiseServiceOptions get_sensor_noise_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetSensorNoiseProperties>(
                                                                   get_sensor_noise_properties_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::nrpGetSensorNoiseProperties,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  nrp_get_sensor_noise_properties_service_ = nh_->advertiseService(get_sensor_noise_properties_aso);

}

bool GazeboRosApiPlugin::nrpAdvanceSimulation(gazebo_msgs::AdvanceSimulation::Request &req,
                                           gazebo_msgs::AdvanceSimulation::Response &res)
{

  if (!world_->IsPaused()) {
    res.success = false;
    res.status_message = "AdvanceSimulation: simulation is already running";
  } else {
    world_->Step(req.steps);
    res.success = true;
    res.status_message = "AdvanceSimulation: simulation successfully advanced";
  }

  return true;
}

bool GazeboRosApiPlugin::nrpResetSimTime(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res)
{
  world_->ResetTime();
  return true;
}

bool GazeboRosApiPlugin::nrpResetSim(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  world_->Reset();
  return true;
}

bool GazeboRosApiPlugin::nrpEndWorld(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  world_->Fini();
  return true;
}

bool GazeboRosApiPlugin::nrpDeleteLights(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res)
{
  for (const gazebo::msgs::Light& light : nrp_gazeboscene_.light()) {
    this->nrpPublishRequest("entity_delete", light.name());
  }
  return true;
}

bool GazeboRosApiPlugin::nrpDeleteLight(gazebo_msgs::DeleteLight::Request &req,
                                        gazebo_msgs::DeleteLight::Response &res)
{
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);

  if (phy_light == nullptr) {
    res.success = false;
    res.status_message = std::string("DeleteLight: Requested light ") + req.light_name + std::string(" not found!");
  }
  else {
    this->nrpPublishRequest("entity_delete", req.light_name);
    res.success = false;

    for (int i = 0; i < 100; i++) {
      phy_light = world_->LightByName(req.light_name);
      if (phy_light == nullptr) {
        res.success = true;
        res.status_message = std::string("DeleteLight: ") + req.light_name
                           + std::string(" successfully deleted");
        return true;
      }
      // Check every 100ms
      usleep(100000);
    }
  }

  res.status_message = std::string("DeleteLight: Timeout reached while removing light \"") +
                       req.light_name + std::string("\"");

  return true;
}

bool GazeboRosApiPlugin::nrpGetVisualProperties(gazebo_msgs::GetVisualProperties::Request &req,
                                                gazebo_msgs::GetVisualProperties::Response &res)
{
  gazebo::msgs::Model modelMsg;
  gazebo::msgs::Visual visualMsg;


  if (!nrpGetVisualFromWorld(req.model_name, req.link_name, req.visual_name, visualMsg)) {
    res.success = false;
    res.status_message = std::string("getVisualProperties: Requested visual ") + req.visual_name + std::string(" not found!");
    return true;
  }

  res.parent_model_name = visualMsg.parent_name();
  res.cast_shadows = visualMsg.cast_shadows();
  res.transparency = visualMsg.transparency();
  res.laser_retro = visualMsg.laser_retro();
  //res.pose = visualMsg.pose();
  //res.geometry_type = visualMsg.geometry().type();
  res.material_name = visualMsg.material().script().name();
  res.visible = visualMsg.visible();
  res.is_static = visualMsg.is_static();
  res.success = true;

  return true;
}

bool GazeboRosApiPlugin::nrpSetVisualProperties(gazebo_msgs::SetVisualProperties::Request &req,
                                                gazebo_msgs::SetVisualProperties::Response &res)
{
  gazebo::msgs::Visual visualMsg;

if (!nrpGetVisualFromWorld(req.model_name, req.link_name, req.visual_name, visualMsg)) {
    res.success = false;
    res.status_message = std::string("setVisualProperties: Requested visual ") + req.visual_name + std::string(" not found!");
    return true;
  }
    
  if (req.property_name != "material:script:name") {
    res.success = false;
    res.status_message = std::string("nrpSetVisualProperties: Could not set visual property ") +
                         req.property_name + std::string(". The only currently supported property is material:script:name.");
    return true;
  }

  // set the requested properties
  visualMsg.mutable_material()->mutable_script()->set_name(req.property_value);

  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
  gazebo::physics::LinkPtr link = model->GetLink(req.link_name);

  gazebo::msgs::Model modelMsg;
  model->FillMsg(modelMsg);

  uint32_t visual_id;
  link->VisualId(req.visual_name, visual_id);

  uint32_t link_idx;
  auto links = modelMsg.mutable_link();
  for (unsigned int i = 0; i < links->size(); i++) {
    if (links->Get(i).name() == (req.model_name + std::string("::") + req.link_name)) {
      link_idx = i;
      break;
    }
  }

  auto visuals = modelMsg.mutable_link(link_idx)->mutable_visual();
  for (unsigned int i = 0; i < visuals->size(); i++) {
    if (visuals->Get(i).id() == visual_id) {
      *(visuals->Mutable(i)) = visualMsg;
      break;
    }
  }

  nrpCleanModelMsg(modelMsg);
  //std::cout << modelMsg.DebugString() << std::endl;

  // Notify the NRP recording plugin of a material change, use a custom topic to avoid triggering
  // the recorder update with every normal ~/model/modify event.
  gazebo::transport::PublisherPtr recorder_pub =
    gazebonode_->Advertise<gazebo::msgs::Model>("~/recording/model/modify");
  recorder_pub->Publish(modelMsg);

  // Create a publisher on the ~/model/modify topic
  gazebo::transport::PublisherPtr model_pub =
    gazebonode_->Advertise<gazebo::msgs::Model>("~/model/modify");
  model_pub->Publish(modelMsg);

  res.status_message = std::string("nrpSetVisualProperties: Visual properties successfully updated.");
  res.success = true;

  return true;
}

bool GazeboRosApiPlugin::nrpGetLightProperties(gazebo_msgs::GetLightProperties::Request &req,
                                               gazebo_msgs::GetLightProperties::Response &res)
{
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);

  if (phy_light == nullptr) {
      res.success = false;
      res.status_message = std::string("getLightProperties: Requested light ") + req.light_name + std::string(" not found!");
  }
  else {
    gazebo::msgs::Light light;
    phy_light->FillMsg(light);

    res.diffuse.r = light.diffuse().r();
    res.diffuse.g = light.diffuse().g();
    res.diffuse.b = light.diffuse().b();
    res.diffuse.a = light.diffuse().a();

    res.attenuation_constant = light.attenuation_constant();
    res.attenuation_linear = light.attenuation_linear();
    res.attenuation_quadratic = light.attenuation_quadratic();

    res.success = true;
  }

  return true;
}

bool GazeboRosApiPlugin::nrpGetLightsName(gazebo_msgs::GetLightsName::Request &req,
                                          gazebo_msgs::GetLightsName::Response &res)
{
  gazebo::msgs::Scene scene = world_->SceneMsg();
  res.light_names.clear();

  for (const gazebo::msgs::Light &light : scene.light())
  {
    res.light_names.push_back(light.name());
  }

  res.success = true;
  res.status_message = std::string("getLightsName: got Lights");

  return true;
}

bool GazeboRosApiPlugin::nrpExportWorldSDF(gazebo_msgs::ExportWorldSDF::Request &req, gazebo_msgs::ExportWorldSDF::Response &res) {
  res.sdf_dump = "";
  if (this->world_ == NULL) return false;

  boost::shared_ptr<msgs::Response> response = gazebo::transport::request(world_->Name(), "world_sdf_save");

  msgs::GzString msg;
  std::string msgData;

  // Make sure the response is correct
  if (response->response() == "error" || response->type() != msg.GetTypeName()) {
    return false;
  }

  // Parse the response message
  msg.ParseFromString(response->serialized_data());
  // Parse the string into sdf, so we can check whether a world exists.
  sdf::SDF sdf_parsed;
  sdf_parsed.SetFromString(msg.data());

  // Check that sdf contains world
#if SDF_MAJOR_VERSION > 3
  if (!sdf_parsed.Root()->HasElement("world")) {
#else
  if (!sdf_parsed.root->HasElement("world")) {
#endif
    return false;
  }

#if SDF_MAJOR_VERSION > 3
  res.sdf_dump = sdf_parsed.Root()->ToString("");
#else
  res.sdf_dump = sdf_parsed.root->ToString("");
#endif

  return true;
}

bool GazeboRosApiPlugin::nrpSetLightProperties(gazebo_msgs::SetLightProperties::Request &req,
                                               gazebo_msgs::SetLightProperties::Response &res)
{
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);

  if (phy_light == nullptr) {
    res.success = false;
    res.status_message = std::string("getLightProperties: Requested light ") + req.light_name + std::string(" not found!");
  }
  else {
    gazebo::msgs::Light light;

    phy_light->FillMsg(light);

    light.mutable_diffuse()->set_r(req.diffuse.r);
    light.mutable_diffuse()->set_g(req.diffuse.g);
    light.mutable_diffuse()->set_b(req.diffuse.b);
    light.mutable_diffuse()->set_a(req.diffuse.a);

    light.set_attenuation_constant(req.attenuation_constant);
    light.set_attenuation_linear(req.attenuation_linear);
    light.set_attenuation_quadratic(req.attenuation_quadratic);

    // Notify the NRP recording plugin of a light change, use a custom topic to avoid triggering
    // the recorder update with every normal ~/light/modify event.
    gazebonode_->Advertise<gazebo::msgs::Light>("~/recording/light/modify")->Publish(light, true);

    gazebonode_->Advertise<gazebo::msgs::Light>("~/light/modify")->Publish(light, true);

    res.success = true;
  }

  return true;
}

bool GazeboRosApiPlugin::nrpWaitForRendering(std_srvs::Empty::Request &req,
                                             std_srvs::Empty::Response &res)
{

  // Check if a rendering scene exists, one will be created immediately when an object that
  // requires one is inserted (e.g. a robot with a camera sensor). If one does not exist, then
  // there is no need to wait for the rendeirng environment.
  gazebo::rendering::ScenePtr scene = rendering::get_scene(world_->Name());
  if (scene == nullptr)
    return true;

  // If paused, advance the simulation physics by 1 iteration, this will not impact the
  // brain->robot sync but is required to ensure sensors and plugins load as quickly as possible,
  // otherwise they will sit idle for 5 seconds each before fully loading.
  if (world_->IsPaused())
    world_->Step(1);

  // Wait until the world (e.g. models/sensors/plugins loaded via this plugin) matches the items
  // in the rendering scene (e.g. all meshes/textures/sensors/plugins are loaded). Without this,
  // the world (our reference) is out of sync with the rendering environment (the sensor/camera
  // reference) and things dependent on the rendered scene will be blocking/out of sync.
  bool waiting = true;
  while (waiting) {
    waiting = false;

    for (gazebo::physics::LightPtr light : world_->Lights())
      waiting = (scene->GetLight(light->GetName()) == nullptr) || waiting;

    for (gazebo::physics::ModelPtr model : world_->Models())
      waiting = (scene->GetVisual(model->GetName()) == nullptr) || waiting;

    if (waiting)
      usleep(250 * 1000); // 0.25s between checks, full environment loading can take a few seconds
  }

  // Done waiting, everything has been loaded in the rendering environment. This will succeed even if
  // textures and things are missing.
  return true;
}

bool GazeboRosApiPlugin::nrpRequestSceneInfo(std_srvs::Empty::Request &req,
                                             std_srvs::Empty::Response &res)
{
  // right now we pipe the ROS service request through to gazebo
  // expect the response on gazebo topic ~/response
  //TODO: (maybe) publish on a dedicated ROS topic 
  this->nrpPublishRequest("scene_info", "");
  return true;
}

bool GazeboRosApiPlugin::nrpSetSensorNoiseProperties(gazebo_msgs::SetSensorNoiseProperties::Request &req,
                                                     gazebo_msgs::SetSensorNoiseProperties::Response &res)
{
    gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
    if (!model)
    {
      res.success = false;
      res.status_message = "SetSensorNoiseProperties: model does not exist";
      return true;
    }
    else
    {
        bool sensor_found = false;
        bool sensor_type_found = false;

        std::map<std::string, std::tuple<std::string, std::string, std::string>> noise_sensors = model->GetSensorNoisesInformation();

        auto it = noise_sensors.find(req.sensor_name);
        if (it != noise_sensors.end())
        {
            sensor_found = true;
            if (req.noise_type == "gaussian")
            {
                it->second = std::make_tuple(req.noise_type, std::to_string(req.mean), std::to_string(req.std_dev));
                sensor_type_found = true;

                if(model->SetSensorNoiseParameters(it))
                {
                    res.success = true;
                    res.status_message = "SetSensorNoiseProperties: Sensor data Updated";
                }
                else
                {
                    res.success = false;
                    res.status_message = "SetSensorNoiseProperties: Sensor data NOT Updated";
                }
            }
            else
            {
                it->second = std::make_tuple(req.noise_type, std::to_string(req.mean), std::to_string(req.std_dev));
                sensor_type_found = true;

                std::cout << "Noise type input: " << req.noise_type << std::endl;

                if(model->SetSensorNoiseParameters(it))
                {
                    res.success = true;
                    res.status_message = "SetSensorNoiseProperties: Sensor data not Gaussian Noise Updated";
                }
                else
                {
                    res.success = false;
                    res.status_message = "SetSensorNoiseProperties: Sensor data NOT Updated";
                }
            }
        }

        if(!sensor_found)
        {
            res.success = false;
            res.status_message = "SetSensorNoiseProperties: Sensor_name NOT found";
        }
        else if (sensor_found && !sensor_type_found)
        {
            res.success = false;
            res.status_message = "SetSensorNoiseProperties: Type of noise of sensor not gaussian";
        }
        return true;
    }

}

bool GazeboRosApiPlugin::nrpGetSensorNoiseProperties(gazebo_msgs::GetSensorNoiseProperties::Request &req,
                                                     gazebo_msgs::GetSensorNoiseProperties::Response &res)
{
    gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
    if (!model)
    {
      res.success = false;
      res.status_message = "GetSensorNoiseProperties: model does not exist";
      return true;
    }
    else
    {
        bool name_found = false;
        bool noise_model_found = false;
        std::map<std::string, std::tuple<std::string, std::string, std::string>> noise_sensors = model->GetSensorNoisesInformation();

        auto it = noise_sensors.find(req.sensor_name);
        if (it != noise_sensors.end())
        {
            name_found = true;
            if (std::get<0>(it->second) != "")
            {
                res.type_noise = std::get<0>(it->second);
                res.mean = atof(std::get<1>(it->second).c_str());
                res.std_dev = atof(std::get<2>(it->second).c_str());
                noise_model_found = true;
            }
        }

        if (name_found == true && noise_model_found == true)
        {
            res.success = true;
            res.status_message = "GetSensorNoiseProperties: Sensor_name found, Noise Properties found";
        }
        else if (name_found == true && noise_model_found == false)
        {
            res.type_noise = "";
            res.mean = 0.0;
            res.std_dev = 0.0;
            res.success = false;
            res.status_message = "GetSensorNoiseProperties: Sensor_name found, No Noise element in model";
        }
        else
        {
            res.type_noise = "";
            res.mean = 0.0;
            res.std_dev = 0.0;
            res.success = false;
            res.status_message = "GetSensorNoiseProperties: Sensor_name NOT found";
        }

        return true;
    }

}

// ===================================================
// END Custom NRP public methods implementation
// ===================================================

// ===================================================
// BEGIN Custom NRP private methods implementation
// ===================================================


// HBP helper function
bool GazeboRosApiPlugin::nrpGetVisualFromWorld(const std::string &model_name,
                                               const std::string &link_name,
                                               const std::string &visual_name,
                                               gazebo::msgs::Visual &visualMsg)
{
  gazebo::physics::ModelPtr model = world_->ModelByName(model_name);
  if (model == nullptr)
    return false;

  gazebo::physics::LinkPtr link = model->GetLink(link_name);
  if (link == nullptr)
    return false;

  uint32_t visual_idx;
  if (!link->VisualId(visual_name, visual_idx))
    return false;

  visualMsg = link->GetVisualMessage(model_name + std::string("::") + link_name + std::string("::") + visual_name);

  return true;
}

void GazeboRosApiPlugin::nrpCleanModelMsg(gazebo::msgs::Model &modelMsg) {
  // Clean up the model message
  auto joints = modelMsg.mutable_joint();
  for (unsigned int i = 0; i < joints->size(); i++) {
    //std::cout << modelMsg.name() << " " << joints->Mutable(i)->mutable_name() << std::endl;
    joints->Mutable(i)->mutable_name()
      ->replace(0, modelMsg.name().size() + 2, std::string(""));
  }

  auto links = modelMsg.mutable_link();
  for (unsigned int i = 0; i < links->size(); i++) {
    auto link = links->Mutable(i);
    auto collisions = modelMsg.mutable_link(i)->mutable_collision();
    for (unsigned int j = 0; j < collisions->size(); j++) {
      collisions->Mutable(j)->mutable_name()
        ->replace(0, link->name().size() + 2, std::string(""));
    }
    //std::cout << modelMsg.name() << " " << link->name() << std::endl;
    link->mutable_name()->replace(0, modelMsg.name().size() + 2, std::string(""));
  }
}

// HBP helper function - use one time request publishers to avoid race conditions with scene updates
// and other concurrent deletes, adding mutex-es in the above code with a class level publisher
// does not resolve the issue, this plugin should be refactored in the future
void GazeboRosApiPlugin::nrpPublishRequest(const std::string &type, const std::string &value) {
  gazebo::transport::PublisherPtr pub = gazebonode_->Advertise<gazebo::msgs::Request>("~/request");
  gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest(type, value);
  pub->Publish(*msg, true);
}

// ===================================================
// END Custom NRP private methods implementation
// ===================================================

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)
}
