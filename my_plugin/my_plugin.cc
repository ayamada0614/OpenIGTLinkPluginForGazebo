#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

#include <iostream>
#include <math.h>
#include <cstdlib>

#include "igtlOSUtil.h"
#include "igtlTransformMessage.h"
#include "igtlPositionMessage.h"
#include "igtlClientSocket.h"

namespace gazebo
{   
  class MobileBasePlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {

      // Store the pointer to the model
      this->model = _parent;

      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateStart(
            boost::bind(&MobileBasePlugin::OnUpdate, this));
      }

      // OpenIGTLink
      char* hostname = "10.240.104.169";
      int    port     = 18945;
      double fps      = 1000;
      this->interval = (int) (1000.0 / fps);

      this->transMsg = igtl::TransformMessage::New();
      this->transMsg->SetDeviceName("Tracker");

      this->socket = igtl::ClientSocket::New();
      this->r = socket->ConnectToServer(hostname, port);

      if (this->r < 0)
      {
        std::cerr << "Cannot connect to the server." << std::endl;
        exit(0);
      }else{
        std::cerr << "Could connect to the server." << std::endl;
      }

    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (this->FindJointByParam(_sdf, this->arm_base_joint_,
                             "arm_base_joint") && this->FindJointByParam(_sdf, this->arm_shoulder_pan_joint_, "arm_shoulder_pan_joint"))
        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

          if (this->socket.IsNotNull()) // if client connected
          {

	    // input to model
      	    //this->arm_base_joint_->SetForce(0, 0.2);
      	    this->arm_shoulder_pan_joint_->SetForce(0, 0.01);
	    
            // receive states of the robot from the model
            float angle = this->arm_shoulder_pan_joint_->GetAngle(0).GetAsRadian();

	    igtl::Matrix4x4 matrix;
	    GetTestMatrix(matrix,angle);
	    this->transMsg->SetMatrix(matrix);
	    this->transMsg->Pack();
	    this->socket->Send(this->transMsg->GetPackPointer(), this->transMsg->GetPackSize());
	    igtl::Sleep(this->interval); // wait

	  }
    }

    public: void GetTestMatrix(igtl::Matrix4x4& matrix,float theta)
    {
  	  float position[3];
  	  float orientation[4];

	  // position
	  static float phi = 0.0;
	  position[0] = 1.0 * cos(phi);
	  position[1] = 1.0 * sin(phi);
	  position[2] = 1.0 * cos(phi);

	  // orientation
  	  orientation[0]=0.0;
  	  orientation[1]=cos(theta);
  	  orientation[2]=0.0;
  	  orientation[3]=sin(theta);

	  //igtl::Matrix4x4 matrix;
	  igtl::QuaternionToMatrix(orientation, matrix);

	  matrix[0][3] = position[0];
	  matrix[1][3] = position[1];
	  matrix[2][3] = position[2];
  
	  //igtl::PrintMatrix(matrix);
     }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr arm_base_joint_;
    private: physics::JointPtr arm_shoulder_pan_joint_;

    // for OpenIGTLink
    int interval;
    int r;
    igtl::ClientSocket::Pointer socket;
    igtl::PositionMessage::Pointer positionMsg;
    igtl::TransformMessage::Pointer transMsg;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
