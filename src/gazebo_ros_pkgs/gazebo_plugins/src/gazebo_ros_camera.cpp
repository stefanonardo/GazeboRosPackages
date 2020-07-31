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

/*
 @mainpage
   Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <sensor_msgs/fill_image.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <ros/ros.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCamera::GazeboRosCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCamera::~GazeboRosCamera()
{
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
}

void GazeboRosCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  GazeboRosCameraUtils::Load(_parent, _sdf);

  // Load service
  this->_nh = ros::NodeHandle();
  this->_service = this->_nh.advertiseService<gazebo_msgs::GetCameraImage::Request, gazebo_msgs::GetCameraImage::Response>(GetRobotNamespace(_parent, _sdf) + "/" + this->image_topic_name_ + "_service", boost::bind(&GazeboRosCamera::getNewCameraImage, this, _1, _2));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      common::Time cur_time = this->world_->SimTime();
      if (cur_time - this->sensor_update_time_ >= this->update_period_)
      {
        this->PutCameraData(_image);
        this->PublishCameraInfo();
        this->last_update_time_ = cur_time;
      }
    }
  }
}

bool GazeboRosCamera::getNewCameraImage(const gazebo_msgs::GetCameraImageRequest &req, gazebo_msgs::GetCameraImageResponse &res)
{
    // Prevent publisher from copying while  we change
    boost::mutex::scoped_lock lock(this->lock_);

    // Force rendering of new image
    try
    {
        this->camera->Render(true);
    }
    catch(const Ogre::InternalErrorException &e)
    {
        // Camera image cannot be rendered when simulation has not yet started. Inform user of problem
        if(ros::Time::now().toNSec() <= 1000000)
        {
            throw std::logic_error("Couldn't render camera image. Make sure simulation has run at least once before requesting image");
        }
        else
            throw;
    }

    // Update sensor update time as well as image published by publisher
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    fillImage(this->image_msg_, this->type_, this->height_, this->width_,
        this->skip_*this->width_, reinterpret_cast<const void*>(this->camera->ImageData()));


    // copy data into image
    res.image_data.header.frame_id = this->frame_name_;
    res.image_data.header.stamp.sec = this->sensor_update_time_.sec;
    res.image_data.header.stamp.nsec = this->sensor_update_time_.nsec;

    res.image_data = this->image_msg_;

    res.success = true;

    return res.success;
}
}
