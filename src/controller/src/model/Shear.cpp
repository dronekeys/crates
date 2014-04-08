/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "Shear.h"

// Basic constants 
#define METERS_TO_FEET    3.2808399
#define FEET_TO_METERS    0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

using namespace controller;

// Default constructor
Shear::Shear() : mA(0.05), z0(0.15), speed(0), dir(0), kuv(-4.97391e-01), kw(-1.35341) {}

// REQUIRED METHODS

//  Configure the propulsion engine
void Shear::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
  // save the model pointer
  modPtr = model;

  // GET PARAMETERS FROM THE SDF //////////////////////////////////////

  // Speed and direction
  speed = GetSDFDouble(root,"speed", speed);
  dir   = GetSDFDouble(root,"direction", dir);
  z0    = GetSDFDouble(root,"const", z0);
  mA    = GetSDFDouble(root,"floor", mA);

  // Aerodynamic drag
  kuv = GetSDFDouble(root,"drag.kuv", kuv);
  kw  = GetSDFDouble(root,"drag.kw", kw);

  // SET SOME IMPORTANT VARIABLES ////////////////////////////////////

  // Drag vector
  drag.Set(kuv,kuv,kw);
  drag *= modPtr->GetLink("body")->GetInertial()->GetMass();

  // Call a Reset()
  Reset();
}

// Update the model based on the time step (and internal control)
void Shear::Update(const double &dt)
{
  // Extract the altitude and orientation from the state
  a = modPtr->GetLink("body")->GetWorldPose().pos.z;
  q = modPtr->GetLink("body")->GetWorldPose().rot;

  // Calculate the wind vector, taking into account shear
  if (a > mA)
    wind = FEET_TO_METERS * s20 * (log(METERS_TO_FEET*a/z0)/log(20.0/z0)) * d20;
  else
    wind.Set(0,0,0);

  // Add the force to the body
  modPtr->GetLink("body")->AddRelativeForce(
    -drag*(q.RotateVector(wind))
  );
}

// EXTRA METHODS

// Set the speed (m) and direction (degrees) of the winf
void Shear::SetWind(const double& speed, const double& direction)
{
  // Wind always blows orthogonal to the down direction
  d20.x = -cos(DEGREES_TO_RADIANS * direction);
  d20.y = -sin(DEGREES_TO_RADIANS * direction);
  d20.z = 0.0;

  // Get the speed at 20ft
  s20 = METERS_TO_FEET * speed;
}

// Reset the propulsion engine
void Shear::Reset()
{
  // Create a wind vector
  SetWind(speed, dir);
}
