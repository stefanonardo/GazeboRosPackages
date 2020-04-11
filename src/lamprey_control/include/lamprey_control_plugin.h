#ifndef _LAMPREY_PLUGIN_HH_
#define _LAMPREY_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <gazebo_msgs/JointPositions.h>
#include <ros/callback_queue.h>

using namespace gazebo;

class LampreyControl : public ModelPlugin
{
    /// \brief Constructor
public:
    LampreyControl();
    ~LampreyControl();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
    virtual void OnPositionTargets(const gazebo_msgs::JointPositions::ConstPtr &msg);
    void QueueThread();

private:
    // Gazebo related
    physics::ModelPtr model;
    physics::WorldPtr world;
    double phase;

    // Ros related
    std::string node_namespace;
    std::shared_ptr<ros::NodeHandle> rosnode;
    ros::Subscriber target_positions_sub;
    ros::CallbackQueue queue;
    std::thread callback_queue_thread;
};

#endif