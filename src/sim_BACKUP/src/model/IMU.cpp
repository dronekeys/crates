//  Boost includes
#include "IMU.h"

using namespace controller;

// Default constructor
IMU::IMU() :  t(273.0) {}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void IMU::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
  modPtr = model;
}

// All sensors must be resettable
void IMU::Reset()
{

}

// EXTRA METHODS

// Set the pressure and height at ground level
void IMU::SetMeteorological( const double &temperature,
    const double &Gx, const double &Gy, const double &Gz)
{
  // Set the ground variables
  t = temperature;

  // Magnetic and gravitational fields
  grav.Set(Gx,Gy,Gz);
}

// Set the pressure and height at ground level
gazebo::math::Vector3 IMU::GetAngularVelocity()
{
  // Rotate angular velocity into body frame
  return modPtr->GetWorldPose().rot.GetInverse().RotateVector(
    modPtr->GetWorldAngularVel()
  );
}

// Set the pressure and height at ground level
gazebo::math::Vector3 IMU::GetLinearAcceleration()
{
  // Rotate linear acceleration (specific force + gravity) into body frame
  return modPtr->GetWorldPose().rot.GetInverse().RotateVector(
    modPtr->GetWorldLinearAccel() + grav
  );
}