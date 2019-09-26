
#include "tigrillo_plugin.h"

namespace gazebo
{
	TigrilloPlugin::TigrilloPlugin()
	{
		kill_sim_ = false;
		this->spinner_thread_ = new std::thread(&TigrilloPlugin::spin, this);
	}

	TigrilloPlugin::~TigrilloPlugin()
	{
		event::Events::DisconnectWorldUpdateBegin(this->UpdateConnection);
		
		this->ros_node->shutdown();
		kill_sim_ = true;
		this->spinner_thread_->join();
		delete this->spinner_thread_;
	}

	void TigrilloPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		ROS_INFO("Loading Tigrillo Plugin");
		ROS_INFO_STREAM("Gazebo is using the physics engine: " <<
		_model->GetWorld()->GetPhysicsEngine()->GetType());

		// Set Timer parameters
		delay_s_ = 1.0 / 10.0;
		if (_sdf->HasElement("freq") && _sdf->Get<double>("freq") > 0) {
			delay_s_ = 1.0 / _sdf->Get<double>("freq");
		}

		// Set PID parameters
		double p = 5;
		double i = 0;
		double d = 0;
		if (_sdf->HasElement("p")) {
			p = _sdf->Get<double>("p");
		}
		if (_sdf->HasElement("i")) {
			i = _sdf->Get<double>("i");
		}
		if (_sdf->HasElement("d")) {
			d = _sdf->Get<double>("d");
		}
		ROS_INFO_STREAM("PID values are set to: " << p << "   " << i << "   " << d);

		// When the PID is only proportional equal to 1, control the motors in position directly
		if (p == 1 && i == 0 && d == 0)
			this->no_pid = true;

		// Safety check
		if (_model->GetJointCount() == 0)
		{
			ROS_ERROR("Invalid joint count, plugin not loaded");
			return;
		}

		// Store the model pointer for convenience.
		this->model = _model;

		// // Print all joints
		// std::vector<physics::JointPtr> jvec = this->model->GetJoints();
		// for(std::vector<physics::JointPtr>::iterator it = jvec.begin(); it != jvec.end(); ++it) {
		// 	std::cerr << (*it)->GetName() << "\n";
		// }
		// std::cout << this->model->GetName() + "::" + this->name_shoulder_L << std::endl;

		// Get the four actuated joints
		this->joint_shoulder_L = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_shoulder_L);
		this->joint_shoulder_R = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_shoulder_R);
		this->joint_hip_L = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_hip_L);
		this->joint_hip_R = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_hip_R);

		// Get the four passive joints
		this->joint_elbow_L = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_elbow_L);
		this->joint_elbow_R = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_elbow_R);
		this->joint_knee_L = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_knee_L);
		this->joint_knee_R = this->model->GetJoint(this->model->GetName() +
			"::" + this->name_knee_R);

		// Setup a P-controller, with a gain of 0.1.
		this->pid_shoulder_L = common::PID(p, i, d);
		this->pid_shoulder_R = common::PID(p, i, d);
		this->pid_hip_L = common::PID(p, i, d);
		this->pid_hip_R = common::PID(p, i, d);

		// Apply the P-controller to the joint.
		this->model->GetJointController()->SetPositionPID(
			this->joint_shoulder_L->GetScopedName(), this->pid_shoulder_L);
		this->model->GetJointController()->SetPositionPID(
			this->joint_shoulder_R->GetScopedName(), this->pid_shoulder_R);
		this->model->GetJointController()->SetPositionPID(
			this->joint_hip_L->GetScopedName(), this->pid_hip_L);
		this->model->GetJointController()->SetPositionPID(
			this->joint_hip_R->GetScopedName(), this->pid_hip_R);

		// Default to zero position
		float position[4] = {0, 0, 0, 0};
		this->SetPositionTarget(position);

		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->UpdateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&TigrilloPlugin::OnUpdate, this));

		// Initialize ROS
		this->RosInit();
	}


	void TigrilloPlugin::SetPositionTarget(float _pos[])
	{
		// Set the joint's target position.
		float p_pos_0 = this->joint_shoulder_L->GetAngle(0).Degree();
		float p_pos_1 = this->joint_shoulder_R->GetAngle(0).Degree();
		float p_pos_2 = this->joint_hip_L->GetAngle(0).Degree();
		float p_pos_3 = this->joint_hip_R->GetAngle(0).Degree();
		// ROS_INFO_STREAM("Updating position. Previous " << p_pos_0 << " "
		// 	<< p_pos_1 << " " << p_pos_2 << " " << p_pos_3 << " and new: "
		// 	<< _pos[0] << " " <<_pos[1] << " " << _pos[2] << " " << _pos[3]);

		if (this->no_pid) {
			this->model->GetJointController()->SetJointPosition(
				this->joint_shoulder_L->GetScopedName(), _pos[0] * 0.01745329251);
			this->model->GetJointController()->SetJointPosition(
				this->joint_shoulder_R->GetScopedName(), _pos[1] * 0.01745329251);
			this->model->GetJointController()->SetJointPosition(
				this->joint_hip_L->GetScopedName(), _pos[2] * 0.01745329251);
			this->model->GetJointController()->SetJointPosition(
				this->joint_hip_R->GetScopedName(), _pos[3] * 0.01745329251);
		} else {
			this->model->GetJointController()->SetPositionTarget(
				this->joint_shoulder_L->GetScopedName(), _pos[0] * 0.01745329251);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_shoulder_R->GetScopedName(), _pos[1] * 0.01745329251);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_hip_L->GetScopedName(), _pos[2] * 0.01745329251);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_hip_R->GetScopedName(), _pos[3] * 0.01745329251);
		}
	}
	

	void TigrilloPlugin::GetSensors(float _sens[])
	{
		// Get joint position
		_sens[0] = this->joint_elbow_L->GetAngle(0).Degree() + this->knee_angle;
		_sens[1] = this->joint_elbow_R->GetAngle(0).Degree() + this->knee_angle;
		_sens[2] = this->joint_knee_L->GetAngle(0).Degree() + this->knee_angle;
		_sens[3] = this->joint_knee_R->GetAngle(0).Degree() + this->knee_angle;

		// Add noise?
	}


	void TigrilloPlugin::GetMotors(float _mot[])
	{
		// Get joint position
		_mot[0] = this->joint_shoulder_L->GetAngle(0).Degree();
		_mot[1] = this->joint_shoulder_R->GetAngle(0).Degree();
		_mot[2] = this->joint_hip_L->GetAngle(0).Degree();
		_mot[3] = this->joint_hip_R->GetAngle(0).Degree();
	}


	void TigrilloPlugin::OnRosMsg(const tigrillo_2_plugin::MotorsConstPtr &_msg)
	{
		// Convert message to array of four floats 
		float position[4];

		position[0] = _msg->FL;
		position[1] = _msg->FR;
		position[2] = _msg->BL;
		position[3] = _msg->BR;

		this->SetPositionTarget(position);
	}

	void TigrilloPlugin::RosInit()
	{
		// Initialize ros, if it has not already been initialized.
		if (!ros::isInitialized())
		{

		ROS_INFO_STREAM("Create ROS Node: /" << this->ros_node_name);
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, this->ros_node_name,
				ros::init_options::NoSigintHandler);
		}

		// Create our ROS node. This acts in a similar manner to
		// the Gazebo node
		this->ros_node.reset(new ros::NodeHandle(this->ros_node_name));

		// Create a named topic, and subscribe to it
		ROS_INFO_STREAM("Subscribe to topic: " + this->ros_node_name + "/" + this->ros_sub_name);
		this->ros_sub = this->ros_node->subscribe(this->ros_sub_name, 1, &TigrilloPlugin::OnRosMsg, this);

		// Advertise topics for NRP
		this->model->SaveControllerActuatorRosTopics(this->ros_node_name + "/" + this->ros_sub_name, "tigrillo_2_plugin/Motors");
		this->model->SaveSensorRosTopicNames("", this->ros_node_name + "/" + this->ros_pub_name_s, "tigrillo_2_plugin/Sensors");
		this->model->SaveSensorRosTopicNames("", this->ros_node_name + "/" + this->ros_pub_name_m, "tigrillo_2_plugin/Motors");

		// Create a named topic for publication
		ROS_INFO_STREAM("Publish sensors in topic: " + this->ros_node_name + "/" + this->ros_pub_name_s);
		this->ros_pub_s = 
			this->ros_node->advertise<tigrillo_2_plugin::Sensors>(this->ros_pub_name_s, 1);

		// Create a named topic for publication
		ROS_INFO_STREAM("Publish motors in topic: " + this->ros_node_name + "/" + this->ros_pub_name_m);
		this->ros_pub_m = 
			this->ros_node->advertise<tigrillo_2_plugin::Motors>(this->ros_pub_name_m, 1);
	}
	
	void TigrilloPlugin::OnUpdate()
	{
		double current_time = this->model->GetWorld()->GetSimTime().Double();
		
		if (current_time - last_on_update_time_ < delay_s_)
		{
			return;
		}

		last_on_update_time_ = current_time;

		// Sensors
		float sensors[4];
		this->GetSensors(sensors);
		// Send over ROS
		tigrillo_2_plugin::Sensors sensorsMsg;
		sensorsMsg.FL = sensors[0];
		sensorsMsg.FR = sensors[1];
		sensorsMsg.BL = sensors[2];
		sensorsMsg.BR = sensors[3];
		sensorsMsg.run_time = this->model->GetWorld()->GetSimTime().Double();
		// ROS_DEBUG_STREAM("Publishing sensors: FL: " << sensorsMsg.FL << " FR: " << 
		// 	sensorsMsg.FR << " BL: " << sensorsMsg.BL << " BR: " << sensorsMsg.BR );
		this->ros_pub_s.publish(sensorsMsg);

		// Motors
		// Get sensors values
		float motors[4];
		this->GetMotors(motors);
		// Send over ROS
		tigrillo_2_plugin::Motors motorsMsg;
		motorsMsg.FL = motors[0];
		motorsMsg.FR = motors[1];
		motorsMsg.BL = motors[2];
		motorsMsg.BR = motors[3];
		motorsMsg.run_time = this->model->GetWorld()->GetSimTime().Double();
		// ROS_DEBUG_STREAM("Publishing motors: FL: " << msg.FL << " FR: " << 
		// 	msg.FR << " BL: " << msg.BL << " BR: " << msg.BR );
		this->ros_pub_m.publish(motorsMsg);

	}

	void TigrilloPlugin::spin()
	{
		while(ros::ok() && !kill_sim_) ros::spinOnce();
	}
	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(TigrilloPlugin);
}
