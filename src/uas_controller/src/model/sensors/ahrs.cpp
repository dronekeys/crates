//  Boost includes
#include "ahrs.h"

using namespace uas_controller;

// Default constructor
AHRS::AHRS() : uas_hal::Orientation("ahrs") {}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void AHRS::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
  modPtr = model;
}

// All sensors must be resettable
void AHRS::Reset()
{

}

// EXTRA METHODS

// Set the pressure and height at ground level
gazebo::math::Vector3 AHRS::GetOrientation()
{
  return modPtr->GetWorldPose().rot.GetAsEuler();
}