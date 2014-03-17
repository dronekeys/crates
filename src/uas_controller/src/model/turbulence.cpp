/* 
  This gazebo plugin provides a simple model for linear wind. 
*/

// Required to bind to non-static methods
#include <boost/bind.hpp>

// Gazebo API
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

// Useful includes
#define METERS_TO_FEET 3.28083990000
#define FEET_TO_METERS 0.30479999953
#define WIND_Z0        0.15000000000

namespace uas_controller
{
  class Turbulence : public gazebo::ModelPlugin
  {

  private:

    // Pointer to the current model
    gazebo::physics::ModelPtr         modelPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr      connectionPtr;

    // Transport-related stuff
    gazebo::transport::NodePtr        nodePtr;
    gazebo::transport::SubscriberPtr  subPtrWind;

  public: 

    // On initial plugin load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save pointer to the model
      this->modelPtr = _model;

      node(new gazebo::transport::Node());

      // This plugin binds to the gazebo server and subscribes to the /wind
      // topic. The
      subPtrWind  = node->Subscribe("~/wind", boost::bind(&Shear::Update, this, _1));


      // Set up callback for updating the model
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Shear::Update, this, _1));
    }


    void Update(ConstWorldStatisticsPtr &_msg)
    {
      std::cout << _msg->DebugString();
    }

    // Second-order Runge-Kutta dynamics
    void UpdateDynamics(const gazebo::common::UpdateInfo & _info)
    {
      // Time over which dynamics must be updated
      double dT = _info.simTime.Double() - _time;

      /*
      // Extract the state from gazebo
      X[_x]       = model->GetWorldPose().pos.x;
      X[_y]       = model->GetWorldPose().pos.y;
      X[_z]       = model->GetWorldPose().pos.z;
      X[_roll]    = model->GetWorldPose().rot.GetAsEuler().x;
      X[_pitch]   = model->GetWorldPose().rot.GetAsEuler().y;
      X[_yaw]     = model->GetWorldPose().rot.GetAsEuler().z; 
      X[_u]       = model->GetWorldLinearVel().x;
      X[_v]       = model->GetWorldLinearVel().y;
      X[_w]       = model->GetWorldLinearVel().z;
      X[_p]       = model->GetWorldAngularVel().x;
      X[_q]       = model->GetWorldAngularVel().y;
      X[_r]       = model->GetWorldAngularVel().z;

      // Set some artificial control
      U[0] = -0.0127;
      U[1] = -0.0071;
      U[2] =  1.0;
      U[3] = -0.0180;
      U[4] = 12.0000;

      // RK: First iteration
      for (int i = 0; i < NUMX; i++)
          tmp[i] = X[i];            
      GeneticODE(RK1, tmp, U, dT);
      
      // RK:Second iteration
      for (int i = 0; i < NUMX; i++)
          tmp[i] = X[i] + RK1[i] * dT;
      GeneticODE(RK2, tmp, U, dT);
      
      // Final estimate
      for (int i = 0; i < NUMX; i++)
          X[i] = X[i] + 0.5 * (RK1[i] + RK2[i]) * dT;
      */

      
      this->model->SetLinearAccel(
        gazebo::math::Vector3(
          0, 
          0, 
          9.8
        )
      );

      this->model->SetAngularVel(
        gazebo::math::Vector3(
          0, 
          0, 
          0
        )
      );
      // Save the current time
      _time = _info.simTime.Double();
    }


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Turbulence)
}