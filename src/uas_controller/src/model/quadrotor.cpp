//  Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/gazebo.hh"

// Indices for anchors, state and control
typedef enum
{
  _x = 0,   // Position X
  _y,       // Position Y
  _z,       // Position Z
  _roll,    // Roll
  _pitch,   // Pitch
  _yaw,     // Yaw
  _u,       // Velocity X
  _v,       // Velocity Y
  _w,       // Velocity Z
  _p,       // Angular Velocity Roll
  _q,       // Angular Velocity Pitch
  _r,       // Angular Velocity Yaw
  _Fth,     // Thrust force
  NUMX      // -- NUMBER OF ELEMENTS ---
} 
STATE;

// Indices for anchors, state and control
typedef enum
{
  _pt = 0,   // Pitch
  _rl,       // Roll
  _th,       // Thrust
  _ya,       // Yaw
  _vb,       // Battery voltage 
  NUMU       // -- NUMBER OF ELEMENTS ---
} 
CONTROL;

namespace uas_controller
{
  class Quadrotor : public gazebo::ModelPlugin
  {
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

    // Animate the motors
    std::map<std::string,gazebo::common::NumericAnimationPtr> MotorAnimation(const char* name, double rpm)
    {
      std::map<std::string,gazebo::common::NumericAnimationPtr> anim;
      anim[name].reset(new gazebo::common::NumericAnimation("motor_animation", 1.0, true));
      gazebo::common::NumericKeyFrame *key;
      key = anim[name]->CreateKeyFrame(0.0);  key->SetValue(rpm*0.00000000000);
      key = anim[name]->CreateKeyFrame(0.5);  key->SetValue(rpm*3.14159265359);
      key = anim[name]->CreateKeyFrame(1.0);  key->SetValue(rpm*6.28318530718);
      return anim;
    }

    // On initial load
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
      // Save pointer to the model
      this->model = _model;

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

      // Turn on motor animation
      this->model->SetJointAnimation(this->MotorAnimation("motor_n", 10));
      this->model->SetJointAnimation(this->MotorAnimation("motor_e",-10));
      this->model->SetJointAnimation(this->MotorAnimation("motor_s", 10));
      this->model->SetJointAnimation(this->MotorAnimation("motor_w",-10));

      // Set up callback for updating the model
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Quadrotor::UpdateDynamics, this, _1));
    }

    // Second-order Runge-Kutta dynamics
    void UpdateDynamics(const gazebo::common::UpdateInfo & _info)
    {
      // Time over which dynamics must be updated
      double dT = _info.simTime.Double() - _time;

      // Extract the pose and velocities from the system
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

      // Set some artificial control (should gently raise quadcopter)
      U[0] = 0.0        * -2291.83118052329;
      U[1] = 0.0        * -2291.83118052329;
      U[2] = 0.6        *  4097;
      U[3] = -0.0180    * -460.597254433196;
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

      // North-East-Down to North-West-Up conversion
      this->model->SetLinearVel(
        gazebo::math::Vector3(
          X[_u], 
          X[_v], 
          X[_w]
        )
      );

      this->model->SetAngularVel(
        gazebo::math::Vector3(
          X[_p], 
          X[_q], 
          X[_r]
        )
      );

      // Save the current time
      _time = _info.simTime.Double();
    }

    // Ordinary Differential Equation
    void GeneticODE(double Xdot[NUMX], double X[NUMX], double U[NUMU], double dT)
    {
      // Euler angles to current rotation (RHS ZYX convention)
      sph = sin(X[_roll]); 
      cph = cos(X[_roll]);
      sth = sin(X[_pitch]); 
      cth = cos(X[_pitch]); 
      tth = sth/cth;
      sps = sin(X[_yaw]); 
      cps = cos(X[_yaw]);
      dcm[_x][_x]  = cth*cps;
      dcm[_x][_y]  = -cph*sps+sph*sth*cps;
      dcm[_x][_z]  = sph*sps+cph*sth*cps;
      dcm[_y][_x]  = cth*sps;
      dcm[_y][_y]  = cph*cps+sph*sth*sps;
      dcm[_y][_z]  = -sph*cps+cph*sth*sps;
      dcm[_z][_x]  = -sth;
      dcm[_z][_y]  = sph*cth;
      dcm[_z][_z]  = cph*cth;

      // Update n-frame orientation
      Xdot[_roll]  = X[_p] + X[_q]*sph*tth + X[_r]*cph*tth;
      Xdot[_pitch] = X[_q]*cph - X[_r]*sph;
      Xdot[_yaw]   = X[_q]*sph/cth + X[_r]*cph/cth;

      // Update b-frame angular velocity
      Xdot[_p] = _pq1*(_pq0*U[_rl] - X[_roll]) + _pq2*X[_p];    
      if ((X[_p] > _MAX_ANGVEL && (Xdot[_p] > 0)) || (X[_p] < -_MAX_ANGVEL && (Xdot[_p] < 0)))
          Xdot[_p] = 0;
      Xdot[_q] = _pq1*(_pq0*U[_pt] - X[_pitch]) + _pq2*X[_q];    
      if ((X[_q] > _MAX_ANGVEL && (Xdot[_q] > 0)) || (X[_q] < -_MAX_ANGVEL && (Xdot[_q] < 0)))
          Xdot[_q] = 0;
      Xdot[_r] = _r0*U[_ya] + _r1*X[_r];

      // Update position (navigation frame)
      Xdot[_x] =  X[_u]*dcm[_x][_x] + X[_v]*dcm[_x][_y] + X[_w]*dcm[_x][_z];
      Xdot[_y] =  X[_u]*dcm[_y][_x] + X[_v]*dcm[_y][_y] + X[_w]*dcm[_y][_z];
      Xdot[_z] =  X[_u]*dcm[_z][_x] + X[_v]*dcm[_z][_y] + X[_w]*dcm[_z][_z];

      // Update thrust force
      tau  = 0;
      dFth = ((_Cth0 + _Cth1*U[_th] + _Cth2*U[_th]*U[_th]) - X[_Fth]);
      if (U[_th] < _LOW_THROTT)
        Xdot[_Fth] = _tau0 * dFth;
      else
      {
        if (abs(dFth) < (_tau1*dT))
          tau = dFth / dT;
        else
          tau = (dFth > 0 ? _tau1 : -_tau1);
        if ((X[_Fth] + tau*dT) > (_Cvb0 + _Cvb1*U[_vb]))
          Xdot[_Fth] = (_Cvb0 + _Cvb1*U[_vb] - X[_Fth]) / dT;
        else
          Xdot[_Fth] = tau;
      }

      // Get the mass of the rigid body
      double mass = this->model->GetLink("body")->GetInertial()->GetMass();

      // Acceleration (body frame)
      Xdot[_u] = -X[_q]*X[_w] + X[_r]*X[_v] + _kuv*X[_u] + dcm[_z][_x]*_G;
      Xdot[_v] = -X[_r]*X[_u] + X[_p]*X[_w] + _kuv*X[_v] + dcm[_z][_y]*_G;
      Xdot[_w] = -X[_p]*X[_v] + X[_q]*X[_u] +  _kw*X[_w] + dcm[_z][_z]*_G - ((X[_Fth] + Xdot[_Fth] * dT) / mass);
    }

  private:

    // Pointer to the current model
    gazebo::physics::ModelPtr model;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr updateConnection;
    
    // Parameters
    double _time;                                           // Simulated time 
    double _LOW_THROTT, _MAX_ANGVEL, _G,  _MASS_KG;         // Platform config
    double _pq0, _pq1, _pq2, _r0, _r1;                      // Rotational params
    double _Cth0, _Cth1, _Cth2, _Cvb0, _Cvb1, _tau0, _tau1; // Thrust params
    double _kuv, _kw;                                       // Drag parameters

    // Used to update the state
    double X[NUMX], U[NUMU], dcm[3][3], RK1[NUMX], RK2[NUMX], tmp[NUMX];
    double tau, dFth, sph ,cph, sth, cth, tth, sps, cps;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Quadrotor)
}