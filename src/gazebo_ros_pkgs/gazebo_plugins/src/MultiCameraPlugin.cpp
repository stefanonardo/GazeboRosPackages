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
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo_plugins/MultiCameraPlugin.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(MultiCameraPlugin)

/////////////////////////////////////////////////
MultiCameraPlugin::MultiCameraPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
MultiCameraPlugin::~MultiCameraPlugin()
{
  this->parentSensor.reset();
  this->camera.clear();
}

/////////////////////////////////////////////////
void MultiCameraPlugin::Load(sensors::SensorPtr _sensor,
  sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
#if GAZEBO_MAJOR_VERSION <= 6  
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);
#else
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);
#endif
  if (!this->parentSensor)
  {
    gzerr << "MultiCameraPlugin requires a CameraSensor.\n";
#if GAZEBO_MAJOR_VERSION <= 6  
    if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
#else
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
#endif      
      gzmsg << "It is a depth camera sensor\n";
#if GAZEBO_MAJOR_VERSION <= 6  
    if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
#else
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
#endif
      gzmsg << "It is a camera sensor\n";
  }

  if (!this->parentSensor)
  {
    gzerr << "MultiCameraPlugin not attached to a camera sensor\n";
    return;
  }

    // save camera attributes
#if GAZEBO_MAJOR_VERSION > 6  
  for (unsigned int i = 0; i < this->parentSensor->CameraCount(); ++i)
  {
    this->camera.push_back(this->parentSensor->Camera(i));

    // save camera attributes
    this->width.push_back(this->camera[i]->ImageWidth());
    this->height.push_back(this->camera[i]->ImageHeight());
    this->depth.push_back(this->camera[i]->ImageDepth());
    this->format.push_back(this->camera[i]->ImageFormat());

    std::string cameraName = this->parentSensor->Camera(i)->Name();
#else
  for (unsigned int i = 0; i < this->parentSensor->GetCameraCount(); ++i)
  {
    this->camera.push_back(this->parentSensor->GetCamera(i));

    // save camera attributes
    this->width.push_back(this->camera[i]->GetImageWidth());
    this->height.push_back(this->camera[i]->GetImageHeight());
    this->depth.push_back(this->camera[i]->GetImageDepth());
    this->format.push_back(this->camera[i]->GetImageFormat());

    std::string cameraName = this->parentSensor->GetCamera(i)->GetName();
#endif
    // gzdbg << "camera(" << i << ") name [" << cameraName << "]\n";

    // FIXME: hardcoded 2 camera support only
    if (cameraName.find("left") != std::string::npos)
    {
      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&MultiCameraPlugin::OnNewFrameLeft,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("right") != std::string::npos)
    {
      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&MultiCameraPlugin::OnNewFrameRight,
        this, _1, _2, _3, _4, _5)));
    }
  }

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void MultiCameraPlugin::OnNewFrameLeft(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/camera/me.jpg");
    */
}

/////////////////////////////////////////////////
void MultiCameraPlugin::OnNewFrameRight(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/camera/me.jpg");
    */
}
