/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 

//  Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

namespace uas_controller
{
  class Aerodynamics : public gazebo::ModelPlugin
  {

  private:

    // Pointer to the current model
    gazebo::physics::ModelPtr     modPtr;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr  conPtr;
    
    // Dynamic model parameters
    double _LOW_THROTT, _MAX_ANGVEL, _G,  _MASS_KG;         // Platform config
    double _pq0, _pq1, _pq2, _r0, _r1;                      // Rotational params
    double _Cth0, _Cth1, _Cth2, _Cvb0, _Cvb1, _tau0, _tau1; // Thrust params
    double _kuv, _kw;                                       // Drag parameters

  public: 

    // Get a parametr value
    double GetParameter(sdf::ElementPtr _sdf, const char* name, double val)
    {
      double ret = val;
      if (_sdf->HasElement(name))
      {
        _sdf->GetElement(name)->GetValue()->Get(ret);
        std::cout << "Found parameter " << name << " with value " <<  ret << std::endl;
      }
      return val;
    }

    // On initial load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save pointer to the model
      modPtr = _model;

      // Get the parameters
      _LOW_THROTT = GetParameter(_sdf, "low_throttle",  300.0);
      _MAX_ANGVEL = GetParameter(_sdf, "max_angvel",    2.617993877991494);
      _pq0        = GetParameter(_sdf, "pq0",          -3.25060e-04);
      _pq1        = GetParameter(_sdf, "pq1",           1.79797e+02);
      _pq2        = GetParameter(_sdf, "pq2",          -24.3536);
      _r0         = GetParameter(_sdf, "r0",           -4.81783e-03);
      _r1         = GetParameter(_sdf, "r1",           -5.08944);
      _Cth0       = GetParameter(_sdf, "Cth0",          6.63881e-01);
      _Cth1       = GetParameter(_sdf, "Cth1",          7.44649e-04);
      _Cth2       = GetParameter(_sdf, "Cth2",          2.39855e-06);
      _Cvb0       = GetParameter(_sdf, "Cvb0",         -18.0007);
      _Cvb1       = GetParameter(_sdf, "Cvb1",          4.23754);
      _tau0       = GetParameter(_sdf, "tau0",          3.07321);
      _tau1       = GetParameter(_sdf, "tau1",          46.8004);
      _kuv        = GetParameter(_sdf, "kuv",          -4.97391e-01);
      _kw         = GetParameter(_sdf, "kw",           -1.35341);

      // Set up callback for updating the model dynamics (at physics rate)
      conPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Aerodynamics::Update, this, _1));

      // Add a force to oppose gravity
      modPtr->GetLink("body")->AddTorque(
        gazebo::math::Vector3(
          2.0,
          0.0,
          0.0
        )
      );

    }

    // Second-order Runge-Kutta dynamics
    void Update(const gazebo::common::UpdateInfo & _info)
    {
      // Time over which dynamics must be updated
      double dT = _info.simTime.Double() - _time;

      // Save the current time
      _time = _info.simTime.Double();

      /*
      // Set some artificial control (should gently raise quadcopter)
      U[0] = 0.0        * -2291.83118052329;
      U[1] = 0.0        * -2291.83118052329;
      U[2] = 0.6        *  4097;
      U[3] = -0.0180    * -460.597254433196;
      U[4] = 12.0000;

      // Get the n-frame position and Euler rotation
      gazebo::math::Vector3 n_pos = model->GetWorldPose().pos;
      gazebo::math::Vector3 n_rot = model->GetWorldPose().rot.GetAsEuler();

      // Get the b-frame velocity and angular velocity
      gazebo::math::Vector3 b_vel = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        model->GetWorldLinearVel()
      );
      gazebo::math::Vector3 b_angvel = modPtr->GetWorldPose().rot.GetInverse().RotateVector(
        model->GetWorldAngularVel()
      );

      // Get the threust force
      double Fth = 0;

      // This is what we want to work out
      gazebo::math::Vector3 b_acc;
      gazebo::math::Vector3 b_angacc;
      
      ////////////////////////////////////////////
      // Calculate b-frame angular acceleration //
      ////////////////////////////////////////////

      // X
      b_acc.x = _pq1*(_pq0*ctl.r - n_rot.x) + _pq2*b_angvel.x;    
      if ((b_angvel.x > _MAX_ANGVEL && (b_acc.x > 0)) || (b_angvel.x < -_MAX_ANGVEL && (b_acc.x < 0)))
          b_acc.x = 0;

      // Y
      b_acc.y = _pq1*(_pq0*ctl.p - n_rot.y) + _pq2*b_angvel.y;    
      if ((b_angvel.y > _MAX_ANGVEL && (b_acc.y > 0)) || (b_angvel.y < -_MAX_ANGVEL && (b_acc.y < 0)))
          b_acc.y = 0;

      // Z
      b_acc.z = _r0*ctl.y + _r1*b_angvel.z;

      ////////////////////////////
      // Calculate thrust force //
      ////////////////////////////

      // Update thrust force
      double dFth = ((_Cth0 + _Cth1*ctl.t + _Cth2*ctl.t*ctl.t) - Fth);
      if (ctl.t < _LOW_THROTT)
        dFth = _tau0 * dFth;
      else
      {
        double tau  = 0;
        if (abs(dFth) < (_tau1*dT))
          tau = dFth / dT;
        else
          tau = (dFth > 0 ? _tau1 : -_tau1);
        
        if ((Fth + tau*dT) > (_Cvb0 + _Cvb1*ctl.v))
          dFth = (_Cvb0 + _Cvb1*ctl.v - Fth) / dT;
        else
          dFth = tau;
      }

      ////////////////////////////////////////
      // Update b-frame linear acceleration //
      ////////////////////////////////////////

      // Calculate instantaneous acceleration = angvel x vel + drag * (vel - wind)
      pos_acc = (b_angvel.cross(b_vel) + drag*(b_vel - b_wnd));


       * (modPtr->GetLink("body")->GetInertial()->GetMass()+0.002*4);

      // Add a force to oppose gravity
      modPtr->GetLink("body")->AddForce(
        gazebo::math::Vector3(
          -(modPtr->GetLink("body")->GetInertial()->GetMass()+0.002*4) * modPtr->GetWorld()->GetPhysicsEngine()->GetGravity()
        )
      );

      // Add the thrust acceleration (a = F/m) orthogonal 
      pos.acc.z += ((Fth + dFth * dT) / mass

      // Add a force representing
      modPtr->GetLink("body")->AddForce(dFth);
      modPtr->GetLink("body")->AddTorque(dFth);
      */
    
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Aerodynamics)
}

