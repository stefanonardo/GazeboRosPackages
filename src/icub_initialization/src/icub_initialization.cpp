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
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->jointControl = this->model->GetJointController();
      this->joints = this->jointControl->GetJoints();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Init_iCub::OnUpdate, this, _1));

      // control method
      if (!_sdf->HasElement("control_method"))
      {
        std::cout << "Missing parameter <control_method> in icub_initialization, default to position" << std::endl;
        this->startingPolicy = true;
      }
      else if (_sdf->Get<std::string>("control_method") == "true")
      {
        this->startingPolicy = true;
      }
      else if (_sdf->Get<std::string>("control_method") == "false")
      {
        this->startingPolicy = false;
      }
      else
      {
        std::cout << "Wrong value for the parameter <control_method> in icub_initialization, default to position" << std::endl;
        this->startingPolicy = true;
      }

      common::PID posPID = this->jointControl->GetPositionPIDs().at(this->model->GetName() + "::torso_yaw");
      common::PID velPID = this->jointControl->GetVelocityPIDs().at(this->model->GetName() + "::torso_yaw");

      // Torso
      posPID.SetPGain(1000.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(100);
      velPID.SetIGain(50.0);
      velPID.SetDGain(0.0005);
      posPID.SetIMax(10.0);
      posPID.SetIMin(-10.0);
      velPID.SetIMax(10.0);
      velPID.SetIMin(-10.0);

      this->jointControl->SetPositionPID(this->model->GetName() + "::torso_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::torso_yaw", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::torso_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::torso_roll", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::torso_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::torso_pitch", common::PID(velPID));

      //Head
      posPID.SetPGain(50.0);
      posPID.SetIGain(7.0);
      posPID.SetDGain(0.1);
      velPID.SetPGain(50);
      velPID.SetIGain(50.0);
      velPID.SetDGain(0.0005);

      this->jointControl->SetPositionPID(this->model->GetName() + "::neck_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::neck_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::neck_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::neck_roll", common::PID(velPID));

      velPID.SetPGain(12);
      velPID.SetIGain(10.0);
      velPID.SetDGain(0.0001);

      this->jointControl->SetPositionPID(this->model->GetName() + "::neck_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::neck_yaw", common::PID(velPID));

      velPID.SetPGain(0.0);
      velPID.SetIGain(2.0);
      velPID.SetDGain(0.01);

      this->jointControl->SetPositionPID(this->model->GetName() + "::eye_tilt", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::eye_tilt", common::PID(velPID));

      velPID.SetPGain(2.0);
      velPID.SetIGain(0.1);
      velPID.SetDGain(0.003);

      this->jointControl->SetPositionPID(this->model->GetName() + "::left_eye_pan", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::left_eye_pan", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::right_eye_pan", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::right_eye_pan", common::PID(velPID));

      //left leg
      posPID.SetPGain(1000.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(20.0);
      velPID.SetDGain(10.1);

      this->jointControl->SetPositionPID(this->model->GetName() + "::l_hip_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_hip_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_hip_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_hip_roll", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_hip_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_hip_yaw", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_knee", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_knee", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_ankle_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_ankle_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_ankle_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_ankle_roll", common::PID(velPID));

      //right leg
      posPID.SetPGain(1000.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(20.0);
      velPID.SetDGain(10.1);

      this->jointControl->SetPositionPID(this->model->GetName() + "::r_hip_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_hip_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_hip_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_hip_roll", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_hip_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_hip_yaw", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_knee", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_knee", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_ankle_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_ankle_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_ankle_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_ankle_roll", common::PID(velPID));

      //left arm
      posPID.SetPGain(100.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(2.0);
      velPID.SetDGain(0.1);

      this->jointControl->SetPositionPID(this->model->GetName() + "::l_shoulder_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_shoulder_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_shoulder_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_shoulder_roll", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_shoulder_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_shoulder_yaw", common::PID(velPID));

      velPID.SetPGain(300.0);
      velPID.SetIGain(0.1);
      velPID.SetDGain(0.01);

      this->jointControl->SetPositionPID(this->model->GetName() + "::l_elbow", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_elbow", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_wrist_prosup", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_wrist_prosup", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_wrist_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_wrist_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::l_wrist_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::l_wrist_yaw", common::PID(velPID));

      //right arm
      posPID.SetPGain(100.0);
      posPID.SetIGain(10.0);
      posPID.SetDGain(10.0);
      velPID.SetPGain(500.0);
      velPID.SetIGain(2.0);
      velPID.SetDGain(0.1);

      this->jointControl->SetPositionPID(this->model->GetName() + "::r_shoulder_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_shoulder_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_shoulder_roll", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_shoulder_roll", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_shoulder_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_shoulder_yaw", common::PID(velPID));

      velPID.SetPGain(300.0);
      velPID.SetIGain(0.1);
      velPID.SetDGain(0.01);

      this->jointControl->SetPositionPID(this->model->GetName() + "::r_elbow", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_elbow", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_wrist_prosup", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_wrist_prosup", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_wrist_pitch", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_wrist_pitch", common::PID(velPID));
      this->jointControl->SetPositionPID(this->model->GetName() + "::r_wrist_yaw", common::PID(posPID));
      this->jointControl->SetVelocityPID(this->model->GetName() + "::r_wrist_yaw", common::PID(velPID));

      if (this->startingPolicy)
      {
        for (std::map<std::string, gazebo::physics::JointPtr>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
        {
          if (("l_shoulder_pitch" == it->second->GetName()) || ("r_shoulder_pitch" == it->second->GetName()))
            this->jointControl->SetPositionTarget(it->first, -0.52);
          else if (("l_shoulder_roll" == it->second->GetName()) || ("r_shoulder_roll" == it->second->GetName()))
            this->jointControl->SetPositionTarget(it->first, 0.52);
          else if (("l_elbow" == it->second->GetName()) || ("r_elbow" == it->second->GetName()))
            this->jointControl->SetPositionTarget(it->first, 0.785);
          else if (("l_wrist_yaw" == it->second->GetName()) || ("r_wrist_yaw" == it->second->GetName()))
            this->jointControl->SetPositionTarget(it->first, 0.698);
          else
            this->jointControl->SetPositionTarget(it->first, 0.0);
        }
      }
      else
      {
        for (std::map<std::string, gazebo::physics::JointPtr>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
        {
          this->jointControl->SetVelocityTarget(it->first, 0.0);
        }
      }
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
//       for (std::map<std::string, gazebo::physics::JointPtr>::iterator it = this->joints.begin(); it != this->joints.end(); ++it)
//       {
//         if ("neck_pitch" == it->second->GetName())
//             cout << it->second->GetName() << " "
//                  << it->second->GetAngle(0).Radian() << " "
//                  << it->second->GetVelocity(0) << " "
//                  << it->second->GetForce(0) << endl;
//       }
    }

    // Pointer to the model
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    gazebo::physics::JointControllerPtr jointControl;
    std::map<std::string, gazebo::physics::JointPtr> joints;

    // control method
    bool startingPolicy;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Init_iCub)
}
