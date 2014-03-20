/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "energy.h"

using namespace uas_controller;

// Default constructor
Energy::Energy() : tot(2.2), rem(2.2), cb(0.2), cf(6.6), kill(0.1), warn(0.2) {}

// Configure from plugin sdf
void Energy::Configure(sdf::ElementPtr root)
{
  /*****************************
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
      </energy>           

  *******************************/

  tot  = GetSDFDouble(root,"energy.total",tot);
  rem  = GetSDFDouble(root,"energy.remaining",rem);
  cb   = GetSDFDouble(root,"energy.consumption.base",cb);
  cf   = GetSDFDouble(root,"energy.consumption.flight",cf);
  kill = GetSDFDouble(root,"energy.limits.kill",kill);
  warn = GetSDFDouble(root,"energy.limits.warn",warn);
}

// Reset state
void Energy::Reset()
{
  remaining = rem;
}

// Checks whether the energy is too low to continue flying
bool Energy::IsLow()
{
  return (remaining < warn);
}

// Checks whether the energy is too low to continue flying
bool Energy::IsCriticallyLow()
{
  return (remaining < kill);
}

// Update the energy 
double Energy::Update(const double &thrust, const double &dt)
{
  // Incur some energy cost
  if (thrust > 0)
    remaining -= dt * (cb + cf) / 3600;
  else
    remaining -= dt * cb / 3600;

  // Return the voltage
  return GetVoltage(); 
}

// Predict voltage -- this should definitely be improved (not linear in reality)
double Energy::GetVoltage()
{
  return 9 + (remaining / tot) * 3.5;
}

