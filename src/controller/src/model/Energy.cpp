/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "Energy.h"

using namespace uas_controller;

// Default constructor
Energy::Energy() : tot(2.2), rem(2.2), cb(0.2), cf(6.6), kill(0.1), warn(0.2) {}

//  Configure the energy engine
void Energy::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
  // Extract data
  tot  = GetSDFDouble(root,"total",tot);
  rem  = GetSDFDouble(root,"remaining",rem);
  cb   = GetSDFDouble(root,"consumption.base",cb);
  cf   = GetSDFDouble(root,"consumption.flight",cf);
  kill = GetSDFDouble(root,"limits.kill",kill);
  warn = GetSDFDouble(root,"limits.warn",warn);

  // Reset the energy
  Reset();
}

// Update the model based on the time step (and internal control)
void Energy::Update(const double &dt)
{
  remaining -= dt * (cb + cf * throttle) / 3600;
}

// Reset the propulsion engine
void Energy::Reset()
{
  remaining = rem;
}

// EXTRA METHODS

// Set the throttle value
void Energy::SetThrottle(const double &th)
{
  throttle = th;
}

// Predict voltage -- this should definitely be improved (not linear in reality)
double Energy::GetVoltage()
{
  return 9 + (remaining / tot) * 3.5;
}
