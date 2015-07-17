#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <map>
#include <string>

using namespace std;

namespace gazebo
{
  class Init_iCub : public ModelPlugin
  {
  public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->jointControl = this->model->GetJointController();
      this->joints = this->jointControl->GetJoints();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Init_iCub::OnUpdate, this, _1));
      
      common::PID posPID = this->jointControl->GetPositionPIDs().at(this->model->GetName() + "::torso_yaw");
      common::PID velPID = this->jointControl->GetVelocityPIDs().at(this->model->GetName() + "::torso_yaw");;

      // Torso
      posPID.SetPGain(500.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(2.0);
      velPID.SetDGain(0.1);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::torso_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::torso_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::torso_yaw", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::torso_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::torso_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::torso_roll", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::torso_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::torso_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::torso_pitch", 0.0);
      
      //Head
      posPID.SetPGain(50.0);
      posPID.SetIGain(7.0);
      posPID.SetDGain(0.1);
//       velPID.SetPGain(500.0);
//       velPID.SetIGain(2.0);
//       velPID.SetDGain(0.1);
      velPID.SetPGain(25.0);
      velPID.SetIGain(25.0);
      velPID.SetDGain(0.00);
      //velPID.SetDGain(0.001);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::neck_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::neck_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::neck_pitch", 0.0);
//       this->jointControl->SetVelocityTarget(this->model->GetName() + "::neck_pitch", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::neck_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::neck_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::neck_roll", 0.0);
//       this->jointControl->SetVelocityTarget(this->model->GetName() + "::neck_roll", 0.1);
      this->jointControl->SetPositionPID(this->model->GetName() + "::neck_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::neck_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::neck_yaw", 0.0);
//       this->jointControl->SetVelocityTarget(this->model->GetName() + "::neck_yaw", 0.1);
      
      velPID.SetPGain(10.0);
      velPID.SetIGain(2.0);
      velPID.SetDGain(0.01);

      this->jointControl->SetPositionPID(this->model->GetName() + "::eye_tilt", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::eye_tilt", common::PID(velPID));
//       this->jointControl->SetPositionTarget(this->model->GetName() + "::eye_tilt", 0.0);
      this->jointControl->SetVelocityTarget(this->model->GetName() + "::eye_tilt", 0.0);
      
      velPID.SetPGain(2.0);
      velPID.SetIGain(0.1);
      velPID.SetDGain(0.003);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::left_eye_pan", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::left_eye_pan", common::PID(velPID));
//       this->jointControl->SetPositionTarget(this->model->GetName() + "::left_eye_pan", 0.0);
      this->jointControl->SetVelocityTarget(this->model->GetName() + "::left_eye_pan", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::right_eye_pan", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::right_eye_pan", common::PID(velPID));
//       this->jointControl->SetPositionTarget(this->model->GetName() + "::right_eye_pan", 0.0);
      this->jointControl->SetVelocityTarget(this->model->GetName() + "::right_eye_pan", 0.0);
      
      //left leg
      posPID.SetPGain(1000.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(20.0);
      velPID.SetDGain(10.1);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_hip_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_hip_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_hip_pitch", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_hip_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_hip_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_hip_roll", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_hip_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_hip_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_hip_yaw", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_knee", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_knee", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_knee", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_ankle_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_ankle_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_ankle_pitch", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_ankle_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_ankle_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_ankle_roll", 0.0);
      
      //right leg
//       posPID.SetPGain(1000.0);
//       posPID.SetIGain(10.0);
//       posPID.SetDGain(10.0);
//       velPID.SetPGain(500.0);
//       velPID.SetIGain(20.0);
//       velPID.SetDGain(10.1);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_hip_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_hip_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_hip_pitch", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_hip_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_hip_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_hip_roll", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_hip_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_hip_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_hip_yaw", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_knee", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_knee", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_knee", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_ankle_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_ankle_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_ankle_pitch", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_ankle_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_ankle_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_ankle_roll", 0.0);
      
      //left arm
      posPID.SetPGain(100.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(2.0);
      velPID.SetDGain(0.1);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_shoulder_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_shoulder_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_shoulder_pitch", -0.52);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_shoulder_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_shoulder_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_shoulder_roll", 0.52);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_shoulder_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_shoulder_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_shoulder_yaw", 0.0);
      
      velPID.SetPGain(300.0);
      velPID.SetIGain(0.1);
      velPID.SetDGain(0.01);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_elbow", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_elbow", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_elbow", 0.785);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_wrist_prosup", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_wrist_prosup", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_wrist_prosup", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_wrist_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_wrist_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_wrist_pitch", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_wrist_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_wrist_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::l_wrist_yaw", 0.698);

      //right arm
      posPID.SetPGain(100.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(2.0);
      velPID.SetDGain(0.1);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_shoulder_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_shoulder_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_shoulder_pitch", -0.52);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_shoulder_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_shoulder_roll", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_shoulder_roll", 0.52);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_shoulder_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_shoulder_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_shoulder_yaw", 0.0);
      
      velPID.SetPGain(300.0);
      velPID.SetIGain(0.1);
      velPID.SetDGain(0.01);
      
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_elbow", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_elbow", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_elbow", 0.785);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_wrist_prosup", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_wrist_prosup", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_wrist_prosup", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_wrist_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_wrist_pitch", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_wrist_pitch", 0.0);
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_wrist_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_wrist_yaw", common::PID(velPID));
      this->jointControl->SetPositionTarget(this->model->GetName() + "::r_wrist_yaw", 0.698);
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
    }

    // Pointer to the model
  private: 
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    gazebo::physics::JointControllerPtr jointControl;
    std::map<std::string, gazebo::physics::JointPtr> joints;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Init_iCub)
} 
