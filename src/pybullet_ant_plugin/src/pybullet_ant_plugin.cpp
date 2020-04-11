/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 * https://www.humanbrainproject.eu
 *
 * The Human Brain Project is a European Commission funded project
 * in the frame of the Horizon2020 FET Flagship plan.
 * http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
/*
 * Desc: Gazebo plugin providing force (torque) controller for the PyBullet Ant.
 * This plugin provides ROS topics to control single force joints and one topic to controll all joints at the same time.
 * Furthermore, it provides topics to generate an observation of the current state of the Ant, resembling the
 * "calc_state" function in PyBullet and a topic to run one simulation step in Gazebo.
 *
 * The plugin is derived from the generic_controller written by Lars Pfotzer.
 * Author: Patrick Deubel
 */

#include "pybullet_ant_plugin.h"

namespace gazebo {

    PyBulletAntPlugin::PyBulletAntPlugin() {
        m_nh = ros::NodeHandle();
        m_initial_z = -1;
        m_walk_target_x = 1000;
        m_walk_target_y = 0;
        m_walk_target_dist = 0;

        m_body_rpy = ignition::math::Vector3d(0, 0, 0);
        m_potential = 0;
        m_joint_speeds = std::vector<double>(8, 0.0);
        m_joints_at_limit = 0;

        m_dt = 0;

        // m_actions has already been initialized in header file
        m_electricity_cost = -2.0;
        m_stall_torque_cost = -0.1;
        m_joints_at_limit_cost = -0.1;

        m_force_joints_index_map = std::map<std::string, int>();
        m_force_joints_index_map.emplace("hip_1", 0);
        m_force_joints_index_map.emplace("ankle_1", 1);
        m_force_joints_index_map.emplace("hip_2", 2);
        m_force_joints_index_map.emplace("ankle_2", 3);
        m_force_joints_index_map.emplace("hip_3", 4);
        m_force_joints_index_map.emplace("ankle_3", 5);
        m_force_joints_index_map.emplace("hip_4", 6);
        m_force_joints_index_map.emplace("ankle_4", 7);
    }

    PyBulletAntPlugin::~PyBulletAntPlugin() {
        m_nh.shutdown();
    }

    void PyBulletAntPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
        // Store the pointer to the model
        m_model = parent;

        m_force_joints = std::vector<physics::JointPtr>();
        m_links = std::vector<physics::LinkPtr>();
        m_foot_contact_sensors = std::vector<sensors::ContactSensorPtr>();

        std::string part_names[] = {"front_left_leg::base", "torso", "hip_1", "aux_1::base", "ankle_1",
                                    "front_left_foot::base",
                                    "front_right_leg::base", "hip_2", "aux_2::base", "ankle_2",
                                    "front_right_foot::base",
                                    "back_left_leg::base", "hip_3", "aux_3::base", "ankle_3", "back_left_foot::base",
                                    "back_right_leg::base", "hip_4", "aux_4::base", "ankle_4", "back_right_foot::base"};

        for (const std::string &part : part_names) {
            if (m_model->GetLink(part)) {
                physics::LinkPtr link = m_model->GetLink(part);
                if (link->GetName() == "torso") {
                    m_torso_link = link;
                    if (m_initial_z == -1) {
                        m_initial_z = m_torso_link->WorldPose().Pos().Z();
                    }
                }
                m_links.emplace_back(link);
            } else if (m_model->GetJoint(part)) {
                physics::JointPtr joint = m_model->GetJoint(part);
                createForceController(joint);
                m_force_joints.emplace_back(joint);
            } else {
                ROS_WARN("Did not find a link nor a joint with the name %s", part.c_str());
            }
        }

        if (m_force_joints.size() != 8 || m_links.size() != 13 || m_torso_link == nullptr) {
            ROS_WARN("Base objects of this controller do not have the right amount of links and or joints.");
            ROS_WARN("Further use of this controller is not recommended.");
            ROS_WARN("Are you sure you are using the PyBullet Ant in combination with this controller?");
        }
        const std::string model_name = m_model->GetName();

        const std::string observation_topic_name = model_name + "/observation";
        m_observation_pub = m_nh.advertise<std_msgs::Float64MultiArray>(observation_topic_name, 50);

        m_obs.layout.dim.resize(1);
        m_obs.layout.dim[0].label = "observations";
        m_obs.layout.dim[0].size = 28;
        m_obs.data.resize(28);

        const std::string reward_topic_name = model_name + "/reward";
        m_reward_pub = m_nh.advertise<std_msgs::Float64>(reward_topic_name, 1);
        m_reward.data = 0.0;

        const std::string done_topic_name = model_name + "/done";
        m_done_pub = m_nh.advertise<std_msgs::Bool>(done_topic_name, 1);
        m_done.data = false;

        if (m_force_joints.size() == 8) {
            createForceAllController();
        } else {
            ROS_WARN("Could not create the controller for all force joints because too few force joints have been recognized.");
        }

        const std::string max_step_size_topic_name = model_name + "/set_max_step_size";
        m_helper_publisher_vec.push_back(m_nh.subscribe<std_msgs::Float64>(max_step_size_topic_name, 1, boost::bind(
                &PyBulletAntPlugin::setMaxStepSizeCB, this, _1)));

        const std::string publish_observation_topic_name = model_name + "/publish_observation";
        m_helper_publisher_vec.push_back(m_nh.subscribe<std_msgs::Bool>(publish_observation_topic_name, 1, boost::bind(
                &PyBulletAntPlugin::publishObservationCB, this, _1)));
        m_model->SaveControllerActuatorRosTopics(publish_observation_topic_name, "std_msgs/Bool");

        const std::string step_simulation_topic_name = model_name + "/step_simulation";
        m_helper_publisher_vec.push_back(m_nh.subscribe<std_msgs::Bool>(step_simulation_topic_name, 1, boost::bind(
                &PyBulletAntPlugin::stepSimulationCB, this, _1)));
        m_model->SaveControllerActuatorRosTopics(step_simulation_topic_name, "std_msgs/Bool");

        const std::string publish_reward_and_done_topic_name = model_name + "/publish_reward_and_done";
        m_helper_publisher_vec.push_back(m_nh.subscribe<std_msgs::Bool>(publish_reward_and_done_topic_name, 1, boost::bind(
                &PyBulletAntPlugin::publishRewardAndDoneCB, this, _1)));

        sensors::SensorManager *sensor_manager = sensors::SensorManager::Instance();

        if (sensor_manager) {
            auto all_sensors = sensor_manager->GetSensors();
            for (auto &sensor : all_sensors) {
                if (sensor->Type() == "contact") {
                    sensors::ContactSensorPtr foot_sensor =
                            std::dynamic_pointer_cast<sensors::ContactSensor, sensors::Sensor>(sensor);
                    m_foot_contact_sensors.emplace_back(foot_sensor);
                    foot_sensor->Init();
                }
            }
        } else {
            ROS_WARN("The SensorManager does not exist. Could not create the foot contact sensors.");
        }

        // Controller time control.
        this->lastControllerUpdateTime = this->m_model->GetWorld()->SimTime();

        int numJoints = m_joints.size();
        m_js.header.stamp.sec = this->lastControllerUpdateTime.sec;
        m_js.header.stamp.nsec = this->lastControllerUpdateTime.nsec;
        m_js.name.resize(numJoints);
        m_js.position.resize(numJoints);
        m_js.velocity.resize(numJoints);
        m_js.effort.resize(numJoints);

        // Listen to the update event. This event is broadcast every simulation iteration.
        m_updateConnection = event::Events::ConnectBeforePhysicsUpdate(
                boost::bind(&PyBulletAntPlugin::OnUpdate, this, _1));

        const std::string joint_states_topic_name = model_name + "/joint_states";

        this->m_joint_state_pub = m_nh.advertise<sensor_msgs::JointState>(joint_states_topic_name, 10);

        m_ja.layout.dim.resize(1);
        m_ja.layout.dim[0].label = "accelerations";
        m_ja.layout.dim[0].size = numJoints;
        m_ja.layout.dim[0].stride = 1;
        m_ja.data.resize(numJoints);

        const std::string joint_accel_topic_name = model_name + "/joint_accel";

        this->m_joint_accel_pub = m_nh.advertise<std_msgs::Float32MultiArray>(joint_accel_topic_name, 10);

        // Initialization of Ant specific variables
        m_dt = m_model->GetWorld()->Physics()->GetMaxStepSize() * 4.0;

        // Publish an initial observation, which also sets some member attributes to a reasonable value
        publish_observation();

        // Calculate the initial potential based on the initial observation
        calculate_potential();
    }

// Called by the world update start event
    void PyBulletAntPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {
        std::lock_guard<std::mutex> lock(this->mutex);

        gazebo::common::Time curTime = this->m_model->GetWorld()->SimTime();

        if (curTime > this->lastControllerUpdateTime) {
            m_js.header.stamp.sec = curTime.sec;
            m_js.header.stamp.nsec = curTime.nsec;
            // Update the control surfaces and publish the new state.
            int curr_ind = 0;
            for (auto joint_iter = m_joints.begin();
                 joint_iter != m_joints.end(); ++joint_iter, ++curr_ind) {
                physics::JointPtr joint = joint_iter->second;

                // If its a fixed joint reading some data results in an error
                if (!joint->HasType(physics::Base::FIXED_JOINT)) {
                    m_js.name[curr_ind] = joint->GetName();
                    m_js.position[curr_ind] = joint->Position();
                    m_js.velocity[curr_ind] = joint->GetVelocity(0);
                    m_js.effort[curr_ind] = joint->GetForce(0);
                    m_ja.data[curr_ind] = joint->GetAcceleration(0);
                }
            }

            m_joint_state_pub.publish(m_js);
            m_joint_accel_pub.publish(m_ja);
        }
        this->lastControllerUpdateTime = curTime;
    }

    //////////////////////////////////////// Helper functions //////////////////////////////////////////

    void PyBulletAntPlugin::apply_force(const double &data, const physics::JointPtr &joint) {
        // Clip data to -1, 1
        double effort(boost::algorithm::clamp(data, -1, 1));

        // Power changes per PyBullet environment, power_coefficient is a variable from the joints in PyBullet
        double power = 2.5;
        double power_coefficient = 100.0;
        joint->SetForce(0, power * power_coefficient * effort);
        ROS_DEBUG("Amplified force %f and applied it to joint %s", effort, joint->GetName().c_str());
    }

    bool PyBulletAntPlugin::publish_observation() {
        ROS_DEBUG("Observation call back called");

        m_joint_speeds.clear();
        m_joints_at_limit = 0;

        int joint_array_size = m_force_joints.size();
        int link_array_size = m_links.size();
        if (joint_array_size == 8 && link_array_size == 13) {
            ROS_DEBUG("Force joints and message have the right size. Continuing.");
            auto j = std::vector<double>();

            for (const physics::JointPtr &joint : m_force_joints) {
                auto name = joint->GetName();
                // current relative position
                double pos = joint->Position(0);
                double vel = joint->GetVelocity(0);
                double lower_limit = joint->LowerLimit(0);
                double upper_limit = joint->UpperLimit(0);

                double pos_mid = 0.5 * (lower_limit + upper_limit);

                double joint_pos = 2 * (pos - pos_mid) / (upper_limit - lower_limit);
                double joint_vel = 0.1 * vel;

                j.emplace_back(joint_pos);
                j.emplace_back(joint_vel);

                m_joint_speeds.emplace_back(joint_vel);
                if (std::abs(joint_pos) > 0.99) {
                    ++m_joints_at_limit;
                }
            }

            ignition::math::Pose3d body_pose = m_torso_link->WorldPose();

            // The order of the parts is different to the order in PyBullet for the parts_xyz array but this should not
            // matter since we use this only to calculate the median
            auto parts_xyz = std::vector<double>();

            for (const physics::LinkPtr &link : m_links) {
                auto curr_pos = link->RelativePose().Pos();
                parts_xyz.emplace_back(curr_pos.X());
                parts_xyz.emplace_back(curr_pos.Y());
                parts_xyz.emplace_back(curr_pos.Z());
            }

            for (const physics::JointPtr &joint : m_force_joints) {
                auto curr_pos = joint->WorldPose().Pos();
                parts_xyz.emplace_back(curr_pos.X());
                parts_xyz.emplace_back(curr_pos.Y());
                parts_xyz.emplace_back(curr_pos.Z());
            }

            // In PyBullet a part "floor" is added which always has the position (0, 0, 0)
            parts_xyz.emplace_back(0.0);
            parts_xyz.emplace_back(0.0);
            parts_xyz.emplace_back(0.0);

            namespace bo = boost::accumulators;

            bo::accumulator_set<double, bo::stats<bo::tag::mean>> acc_x;
            bo::accumulator_set<double, bo::stats<bo::tag::mean>> acc_y;

            int idx = 0;
            for (double curr_coordinate : parts_xyz) {
                if (idx % 3 == 0) {
                    acc_x(curr_coordinate);
                } else if (idx % 3 == 1) {
                    acc_y(curr_coordinate);
                }
                ++idx;
            }

            // Take the mean x and y values of all parts and the z value of the torso as the overall body pose
            double body_xyz[] = {bo::mean(acc_x), bo::mean(acc_y), body_pose.Pos().Z()};
            m_body_rpy = body_pose.Rot().Euler();

            double z = body_xyz[2];

            if (m_initial_z == -1) {
                m_initial_z = z;
            }

            double r = m_body_rpy.X();
            double p = m_body_rpy.Y();
            double yaw = m_body_rpy.Z();

            double delta_x = m_walk_target_x - body_xyz[0];
            double delta_y = m_walk_target_y - body_xyz[1];

            double walk_target_theta = std::atan2(delta_y, delta_x);

            m_walk_target_dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));

            double angle_to_target = walk_target_theta - yaw;

            double rot_speed[3][3] = {
                    {std::cos(-yaw), -(std::sin(-yaw)), 0},
                    {std::sin(-yaw), std::cos(-yaw),    0},
                    {0,              0,                 1}
            };

            ignition::math::Vector3d robot_body_speed = m_torso_link->WorldLinearVel();

            double vx = 0, vy = 0, vz = 0;

            for (int i = 0; i < 3; ++i) {
                double temp = 0;
                for (int k = 0; k < 3; ++k) {
                    temp += rot_speed[i][k] * robot_body_speed[k];
                }
                if (i == 0) {
                    vx = temp;
                } else if (i == 1) {
                    vy = temp;
                } else {
                    vz = temp;
                }
            }

            double more[] = {
                    z - m_initial_z,
                    std::sin(angle_to_target),
                    std::cos(angle_to_target),
                    0.3 * vx,
                    0.3 * vy,
                    0.3 * vz,
                    r,
                    p
            };

            double feet_contact[] = {0.0, 0.0, 0.0, 0.0};
            idx = 0;
            // Could be that the sensors have not been added to the list then the feet_contact will stay on zero
            for (const sensors::ContactSensorPtr &foot_contact_sensor : m_foot_contact_sensors) {
                foot_contact_sensor->Update(true);
                auto recognized_contacts = foot_contact_sensor->Contacts();
                int contact_sizes = recognized_contacts.contact_size();

                for (int i = 0; i < contact_sizes; ++i) {
                    auto collision_object_name = recognized_contacts.contact(i).collision2();
                    if (collision_object_name == "ground_plane::link::collision" ||
                        collision_object_name == "ground::link::collision") {
                        feet_contact[idx] = 1;
                        break;
                    }
                }
                ++idx;

                if (idx > 3) {
                    break;
                }
            }

            idx = 0;
            for (double value : more) {
                m_obs.data[idx] = boost::algorithm::clamp(value, -5, 5);
                ++idx;
            }

            for (double value : j) {
                m_obs.data[idx] = boost::algorithm::clamp(value, -5, 5);
                ++idx;
            }

            for (double value : feet_contact) {
                m_obs.data[idx] = boost::algorithm::clamp(value, -5, 5);
                ++idx;
            }

            m_observation_pub.publish(m_obs);
            ROS_DEBUG("Published a new observation.");
            return true;
        } else {
            ROS_DEBUG("Cannot calculate the observation because the necessary links and joints are not present.");
            return false;
        }
    }

    bool PyBulletAntPlugin::publish_reward_and_done() {
        double alive = -1.0;
        bool done = false;

        // If the Ant touches the ground with its torso (a sphere) than the environment is done
        if ((m_initial_z == -1) || (m_obs.data[0] + m_initial_z > 0.26)) {
            alive = 1.0;
        } else {
            done = true;
        }

        double potential_old = m_potential;
        m_potential = calculate_potential();
        double progress = m_potential - potential_old;

        namespace bo = boost::accumulators;
        bo::accumulator_set<double, bo::stats<bo::tag::mean>> acc;
        bo::accumulator_set<double, bo::stats<bo::tag::mean>> acc_square;

        int action_array_size = sizeof(m_actions)/ sizeof(double);

        if (m_joint_speeds.size() == 8 && action_array_size == 8) {
            int index = 0;
            for (double joint_speed : m_joint_speeds) {
                double current_action = m_actions[index];
                acc(std::abs(current_action * joint_speed));
                acc_square(current_action * current_action);
                ++index;
            }
        }

        double electricity_cost = (m_electricity_cost * bo::mean(acc)) + (m_stall_torque_cost * bo::mean(acc_square));
        double joints_at_limit_cost = m_joints_at_limit_cost * m_joints_at_limit;

        m_reward.data = alive + progress + electricity_cost + joints_at_limit_cost;
        m_reward_pub.publish(m_reward);

        m_done.data = done;
        m_done_pub.publish(m_done);

        ROS_DEBUG("Published a new reward and if the environment is done or not!");
        return true;
    }

    double PyBulletAntPlugin::calculate_potential() {
        return -m_walk_target_dist / m_dt;
    }

//////////////////////////////////////// Controller construction //////////////////////////////////////////

    void PyBulletAntPlugin::createForceController(const physics::JointPtr &joint) {
        // generate joint topic name using the model name as prefix
        std::string topic_name = m_model->GetName() + "/" + joint->GetName() + "/cmd_force";

        // Add ROS topic for force control
        m_force_sub_vec.push_back(m_nh.subscribe<std_msgs::Float64>(topic_name, 1,
                                                                    boost::bind(&PyBulletAntPlugin::forceCB, this,
                                                                                _1, joint)));
        // Store information of Actuator in Model class
        m_model->SaveControllerActuatorRosTopics(topic_name, "std_msgs/Float64");

        ROS_INFO("Added new force controller for joint %s", joint->GetName().c_str());
    }

    void PyBulletAntPlugin::createForceAllController() {
        if (m_force_joints.size() == 8) {
            // generate joint topic name using the model name as prefix
            std::string topic_name = m_model->GetName() + "/force_all/cmd_force";

            // Add ROS topic for force control
            m_force_all_sub_vec.push_back(m_nh.subscribe<std_msgs::Float64MultiArray>(
                    topic_name, 1, boost::bind(&PyBulletAntPlugin::forceAllCB, this, _1)
            ));
            // Store information of Actuator in Model class
            m_model->SaveControllerActuatorRosTopics(topic_name, "std_msgs/Float64MultiArray");

            ROS_INFO("Added new controller for all force joints.");
        } else {
            ROS_WARN("Could not create the force controller for all joints because too few joints have been indexed.");
        }
    }

//////////////////////////////////////// ROS topic callback functions //////////////////////////////////////////

    void PyBulletAntPlugin::forceCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint) {
        ROS_DEBUG("forceCB called! Joint name = %s, joint force = %f", joint->GetName().c_str(), msg->data);
        apply_force(msg->data, joint);
        int index = -1;
        try {
            index = m_force_joints_index_map.at(joint->GetName());

            if (index >= 0 && index <= 7) {
                m_actions[index] = msg->data;
            }
        } catch (const std::out_of_range&) {
            // Joint is not present in the index map so don't save the current action
        }
    }

    void PyBulletAntPlugin::forceAllCB(const std_msgs::Float64MultiArray::ConstPtr &msg) {
        ROS_DEBUG("forceAllCB called!");
        int action_array_size = sizeof(m_actions)/ sizeof(double);

        if (msg->data.size() == 8 && m_force_joints.size() == 8 && action_array_size == 8) {
            ROS_DEBUG("forceAllCB - Message size and force size is 8. Applying the force on all force joints.");
            int data_index = 0;
            for (const physics::JointPtr &joint : m_force_joints) {
                double current_action = msg->data[data_index];
                apply_force(current_action, joint);
                m_actions[data_index] = current_action;
                ++data_index;
            }
        }
    }

    void PyBulletAntPlugin::publishObservationCB(const std_msgs::Bool::ConstPtr &msg) {
        ROS_DEBUG("publishObservationCB called!");

        if (msg->data) {
            publish_observation();
            ROS_DEBUG("Observation published!");
        }
    }

    void PyBulletAntPlugin::stepSimulationCB(const std_msgs::Bool::ConstPtr &msg) {
        ROS_DEBUG("stepSimulationCB called!");

        if (msg->data) {
            m_model->GetWorld()->Step(1);
        }
    }

    void PyBulletAntPlugin::setMaxStepSizeCB(const std_msgs::Float64::ConstPtr &msg) {
        double new_max_step_size = msg->data;
        if (new_max_step_size > 0) {
            m_model->GetWorld()->Physics()->SetMaxStepSize(new_max_step_size);

            // Update delta time with new max step size
            m_dt = m_model->GetWorld()->Physics()->GetMaxStepSize() * 4.0;
            ROS_INFO("Set new max step size to %f", new_max_step_size);
        } else {
            ROS_WARN("Cannot set new max step size for value %f. It must be greater than 0.", new_max_step_size);
        }
    }

    void PyBulletAntPlugin::publishRewardAndDoneCB(const std_msgs::Bool::ConstPtr &msg) {
        ROS_DEBUG("publishRewardAndDoneCB called!");

        if(msg->data) {
            publish_reward_and_done();
            ROS_DEBUG("Published the current reward and done value!");
        }
    }

// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PyBulletAntPlugin)

} // namespace gazebo