#ifndef SIM_DYNAMICS_PROPULSION_H
#define SIM_DYNAMICS_PROPULSION_H

// For processing control messages
#include <hal_quadrotor/Control.h>

// Basic sensor functionality
#include "Dynamics.h"

namespace gazebo
{
  class Propulsion : public Dynamics
  {
  private:

    // Control parameters
    double _srs, _sps, _sys, _sts;
    double _srl, _spl, _syl, _stl;
    double _sru, _spu, _syu, _stu;

    // Dynamics parameters
    double _LOW_THROTT, _MAX_ANGVEL;
    double _pq0, _pq1, _pq2, _r0, _r1;
    double _Cth0, _Cth1, _Cth2, _Cvb0, _Cvb1, _tau0, _tau1; 

    // Running parameters
    double thrust, roll, pitch, yaw, throttle;

    // Clamp a value to lie within a range
    static double Clamp(const double& val, const double& minval, const double& maxval)

  public:

    // All sensors must be configured using the current model information and the SDF
    bool Configure(sdf::ElementPtr root);

    // All sensors must be resettable
    void Reset();

    // Get the current altitude
    void Update(physics::LinkPtr linkPtr, double dt);

    // CUSTOM FUNCTIONALITY //////////////////////////////////////////////////////////

    // Get the remaining energy (mAh)
    double GetThrustForce();

    // Set the remaining energy (mAh)
    void SetThrustForce(double val); 

    // Copy in some control 
    void SetControl(const hal_quadrotor::Control& control);

  };

}

#endif