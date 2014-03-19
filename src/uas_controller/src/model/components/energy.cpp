/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "shear.h"

using namespace uas_controller;

// Default constructor
Shear::Shear() : _mA(0.05), _z0(0.15) {}

// Change the wind parameters 
Shear::SetParameters(const double &mA, const double &z0)
{
  _mA = mA;
  _z0 = z0;
}

// Set the speed (m) and direction (degrees) of the winf
void Shear::SetGlobalField(const double &speed,const double &direction)
{
  // Wind always blows orthogonal to the down direction
  d20.x = -cos(DEGREES_TO_RADIANS * direction);
  d20.y = -sin(DEGREES_TO_RADIANS * direction);
  d20.z = 0.0;

  // Get the speed at 20ft
  s20 = METERS_TO_FEET * speed;
}

// Get the wind vector based on the 
gazebo::math::Vector3 Shear::GetGlobalVelocity(const double &alt)
{
  // Calculate the wind vector, taking into account shear
  if (alt > _mA)
    return FEET_TO_METERS * s20 * (log(METERS_TO_FEET*alt/_z0)/log(20.0/_z0)) * d20;
  
  // If we get here, then we don't have any wind
  return 0.0 * d20;
}