#include "lamprey_control_plugin.h"
#include <math.h>
#include <iostream>

int pidSetChoice = 0; // bis 3
const int nPids = 8;

const double FREQUENCY = 1.4;

double pids[3][nPids][3] = {
    //    PID 1             PID 2             PID 3              PID 4             PID 5             PID 6              PID 7            PID 8
    // masses 0.55:
    {{60.0, 0.0, 0.0}, {60.0, 0.0, 0.0}, {80.0, 0.0, 0.1}, {100, 0.0, 0.0}, {140.0, 0.0, 0.1}, {200.0, 0.0, 0.1}, {160.0, 0.0, 0.0}, {10.0, 0.0, 0.0}},
    // masses 0.18 start:
    {{2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.1}, {3.5, 0.0, 0.0}, {4.5, 0.0, 0.1}, {6.5, 0.0, 0.1}, {5.5, 0.0, 0.0}, {0.1, 0.0, 0.0}},

    {{60.0, 0.0, 0.0}, {60.0, 0.0, 0.0}, {80.0, 0.0, 0.1}, {100, 0.0, 0.0}, {140.0, 0.0, 0.1}, {200.0, 0.0, 0.1}, {160.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}
    // bis hier
};

LampreyControl::LampreyControl() : phase(0.0)
{
}

LampreyControl::~LampreyControl()
{

    std::cout
        << "Model "
        << this->model->GetName()
        << " deleted"
        << std::endl;
}

void LampreyControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->model = _model;

    // Save pointers
    this->world = this->model->GetWorld();

    // Set up PID parameters based on the choice of pidSet
    for (int i = 0; i < nPids; ++i)
    {
        this->model->GetJointController()->SetPositionPID(this->model->GetJoints()[i]->GetScopedName(), gazebo::common::PID(pids[pidSetChoice][i][0], pids[pidSetChoice][i][1], pids[pidSetChoice][i][2]));
    }

    this->node_namespace = "";
    if (_sdf->HasElement("robotNamespace"))
        this->node_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    gzdbg << "Plugin model name: " << this->model->GetName() << "\n";

    // Initialize the ROS node for the gazebo client if necessary
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "lamprey_controller",
                  ros::init_options::NoSigintHandler);
    }
    rosnode.reset(new ros::NodeHandle(node_namespace));

    std::string lamprey_position_topic_name = "/lamprey_positions";
    // Subscribe to the image topic
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<gazebo_msgs::JointPositions>(lamprey_position_topic_name, 1,
                                                                   boost::bind(&LampreyControl::OnPositionTargets, this, _1),
                                                                   ros::VoidPtr(), &queue);
    target_positions_sub = rosnode->subscribe(so);
    model->SaveControllerActuatorRosTopics(lamprey_position_topic_name, "gazebo_msgs::JointPositions");

    this->callback_queue_thread =
        std::thread(std::bind(&LampreyControl::QueueThread, this));
}

void LampreyControl::OnPositionTargets(const gazebo_msgs::JointPositions::ConstPtr &msg)
{
    for (int i = 0; i < nPids; ++i)
    {
        auto joint = this->model->GetJoints()[i];
        double curr_angle;
#if GAZEBO_MAJOR_VERSION >= 8
        curr_angle = joint->Position(0);
#else
        curr_angle = joint->GetAngle(0).Radian();
#endif
        this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), msg->positions[i] - curr_angle);
    }
}

void LampreyControl::QueueThread()
{
    static const double timeout = 0.01;
    while (rosnode->ok())
    {
        queue.callAvailable(ros::WallDuration(timeout));
    }
}

GZ_REGISTER_MODEL_PLUGIN(LampreyControl);
