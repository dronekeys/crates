/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "shear.h"

using namespace uas_controller;

// Default constructor
Shear::Shear() : mA(0.05), z0(0.15), s20(0), d20(0,0,0), wind(0,0,0) {}

// Default constructor takes configuration + pointer to link
void Shear::Configure(sdf::ElementPtr root)
{
 /*************************************
      <shear>
        <enabled>true</enabled>
        <
      </shear>                       

  *************************************/

  // Direct parameters
  mA = GetSDFDouble(root,"shear.floor",mA);
  z0 = GetSDFDouble(root,"shear.z0",z0);
  
  // Speed and direction
  double s = GetSDFDouble(root,"shear.speed",0.0);
  double d = GetSDFDouble(root,"shear.direction" ,0.0);
  
  // Set from SI -> MIL units
  SetWind(s,d);
}

// Set the speed (m) and direction (degrees) of the winf
void Shear::SetWind(const double &speed, const double &direction)
{
  // Wind always blows orthogonal to the down direction
  d20.x = -cos(DEGREES_TO_RADIANS * direction);
  d20.y = -sin(DEGREES_TO_RADIANS * direction);
  d20.z = 0.0;

  // Get the speed at 20ft
  s20 = METERS_TO_FEET * speed;
}

// Get the wind vector based on the 
gazebo::math::Vector3 Shear::Update(const double &alt)
{
  // Calculate the wind vector, taking into account shear
  if (alt > mA)
    wind = FEET_TO_METERS * s20 * (log(METERS_TO_FEET*alt/z0)/log(20.0/z0)) * d20;
  else
    wind.Set(0,0,0);

  // Return the velocity
  return GetVelocity();
}

// Get the wind vector based on the 
gazebo::math::Vector3 Shear::GetVelocity()
{
  return wind;
}