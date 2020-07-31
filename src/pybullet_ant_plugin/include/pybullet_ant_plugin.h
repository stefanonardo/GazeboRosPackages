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
#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/ContactSensor.hh>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <algorithm>
#include <mutex>

#include <boost/assert.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/bind.hpp>

namespace gazebo {

    typedef std::map<std::string, physics::JointPtr> JointMap;

    class PyBulletAntPlugin : public ModelPlugin {

    public:

        PyBulletAntPlugin();

        ~PyBulletAntPlugin();

        // Load the plugin and initialize all controllers
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

        // Simulation update callback function
        void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:

        // Calculates and publishes an array of observation of the Ant. The calculations are directly taken from
        // PyBullet and translated to be used within Gazebo
        bool publish_observation();

        // Calculates and publishes the reward from the current state of the Ant. This should be called after
        // publish_observation was called because it calculates the reward based on variables created there. Otherwise
        // a reward of 0 is published. It also checks if the environment is "done", that means if it falls over. If
        // so, true will be published on the done topic, else false.
        bool publish_reward_and_done();

        // Calculates the potential like in the "calc_potential()" method from PyBullet
        double calculate_potential();

        // Applies the torque provided in the message to the joints in the m_force_joints vector
        void apply_force(const double &data, const physics::JointPtr &joint);

        // Method for creating a force controller
        void createForceController(const physics::JointPtr &joint);

        // Method for creating a controller which applies force (torque) to all joints in m_force_joints
        void createForceAllController();

        // Callback function for a force controller
        void forceCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint);

        // Callback function for the force controller which applies the force (torque) to all joints in m_force_joints
        void forceAllCB(const std_msgs::Float64MultiArray::ConstPtr &msg);

        // Callback function for the publish_observation topic. If the message contains a truth value the
        // current observation will be calculated and published in the observation topic
        void publishObservationCB(const std_msgs::Bool::ConstPtr &msg);

        // Callback function for the step_simulation topic. If the message contains a truth value the simulation will
        // proceed one step
        void stepSimulationCB(const std_msgs::Bool::ConstPtr &msg);

        // Callback function for the set_max_step_size topic. If the message contains a Float64 which is larger
        // than 0, the world max step size will be set to this value.
        void setMaxStepSizeCB(const std_msgs::Float64::ConstPtr &msg);

        // Callback function for the publish_reward_and_done topic. If the message contains a truth value the
        // current reward will be calculated and published on the reward topic. Also it will be checked if the
        // Ant is "done", that means if the Ant reached a state where it will no longer be seen as stable. The
        // resulting boolean value will be published on the done topic.
        void publishRewardAndDoneCB(const std_msgs::Bool::ConstPtr &msg);

        // ROS node handle
        ros::NodeHandle m_nh;

        // Pointer to the model
        physics::ModelPtr m_model;

        // Map of joint pointers
        JointMap m_joints;
        std::vector<physics::JointPtr> m_force_joints;
        std::vector<physics::LinkPtr> m_links;

        physics::LinkPtr m_torso_link = nullptr;

        // Pointer to the update event connection
        event::ConnectionPtr m_updateConnection;

        // ROS subscriber for joint control values
        std::vector<ros::Subscriber> m_helper_publisher_vec;
        std::vector<ros::Subscriber> m_force_sub_vec;
        std::vector<ros::Subscriber> m_force_all_sub_vec;

        /// \brief keep track of controller update sim-time.
        gazebo::common::Time lastControllerUpdateTime;

        /// \brief Controller update mutex.
        std::mutex mutex;

        // ROS joint state publisher
        ros::Publisher m_joint_state_pub;
        sensor_msgs::JointState m_js;

        ros::Publisher m_joint_accel_pub;
        std_msgs::Float32MultiArray m_ja;

        ros::Publisher m_observation_pub;
        std_msgs::Float64MultiArray m_obs;

        ros::Publisher m_reward_pub;
        std_msgs::Float64 m_reward;

        ros::Publisher m_done_pub;
        std_msgs::Bool m_done;

        // Ant specific values
        double m_initial_z;
        int m_walk_target_x;
        int m_walk_target_y;
        double m_walk_target_dist;

        ignition::math::Vector3d m_body_rpy;
        double m_potential;
        std::vector<double> m_joint_speeds;
        double m_joints_at_limit;
        double m_actions[8] = {0.0};
        std::map<std::string, int> m_force_joints_index_map;

        double m_dt;

        // Watch out when changing this size, this can cause an error
        float m_electricity_cost;
        float m_stall_torque_cost;
        float m_joints_at_limit_cost;

        // Contact sensor which will check if the feet of the Ant are on the ground or not
        std::vector<sensors::ContactSensorPtr> m_foot_contact_sensors;
    };

} // namespace gazebo