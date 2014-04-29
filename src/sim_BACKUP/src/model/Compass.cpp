//  Boost includes
#include "Compass.h"

using namespace controller;

// Default constructor
Compass::Compass() : t(273.0) {}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void Compass::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
	modPtr = model;
}

// All sensors must be resettable
void Compass::Reset()
{

}

// EXTRA METHODS

// Set the pressure and height at ground level
void Compass::SetMeteorological( const double &temperature,
    const double &Bx, const double &By, const double &Bz)
{
  // Set the ground variables
  t = temperature;

  // Magnetic field
  mag.Set(Bx,By,Bz);
}

// Set the pressure and height at ground level
gazebo::math::Vector3 Compass::GetMagneticField()
{
	// Rotate world magnetic vector from global to local
	return modPtr->GetWorldPose().rot.GetInverse().RotateVector(mag);
}