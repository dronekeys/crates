//  Boost includes
#include "gnss.h"

using namespace uas_controller;

// Default constructor
GNSS::GNSS() : uas_hal::Position("gnss") {}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void GNSS::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
  modPtr = model;
}

// All sensors must be resettable
void GNSS::Reset()
{

}

// EXTRA METHODS

// Set the pressure and height at ground level
gazebo::math::Vector3 GNSS::GetPosition()
{
	return modPtr->GetLink("body")->GetWorldPose().pos;
}

// Set the pressure and height at ground level
gazebo::math::Vector3 GNSS::GetVelocity()
{
	return modPtr->GetLink("body")->GetWorldLinearVel();
}