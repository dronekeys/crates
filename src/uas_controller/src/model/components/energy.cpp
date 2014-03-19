/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "energy.h"

using namespace uas_controller;

// Default constructor
Energy::Energy() : tot(2.2), rem(2.2), cb(0.2), cf(6.6), kill(0.1), warn(0.2), th(0.001) {}

// Configure from plugin sdf
void Energy::Configure(sdf::ElementPtr sdf)
{
  /*
      <energy>
        <battery>2.2</battery>
        <consumption>
          <base>0.2</base>
          <flight>6.6</flight>
        </consumption>
        <limits>
          <kill>0.1</kill>
          <warn>0.2</warn>
        </limits>
      </energy>           */

  tot  = GetSDFDouble(sdf,"energy.total",tot);
  rem  = GetSDFDouble(sdf,"energy.remaining",rem);
  cb   = GetSDFDouble(sdf,"energy.consumption.base",cb);
  cf   = GetSDFDouble(sdf,"energy.consumption.flight",cf);
  kill = GetSDFDouble(sdf,"energy.limits.kill",kill);
  warn = GetSDFDouble(sdf,"energy.limits.warn",warn);
  th   = GetSDFDouble(sdf,"energy.throttle",th);

}

// Checks whether the energy is too low to continue flying
bool Energy::IsCriticallyLow()
{
  return (rem < kill);
}

// Update the energy 
void Energy::Update(const double &thrust, const double &dt)
{
  if (thrust > 0)
    rem -= dt * (cb + cf) / 3600;
  else
    rem -= dt * cb / 3600;
}

// The amount by which to reduce thrust to offer a gentle landing
double Energy::GetThrustReduction()
{
  return th;
}

// Predict voltage -- this should definitely be improved (not linear in reality)
double Energy::GetVoltage()
{
  return 9 + (rem / tot) * 3.5;
}

