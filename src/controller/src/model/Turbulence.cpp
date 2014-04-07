/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "Turbulence.h"

// Basic constants 
#define METERS_TO_FEET 		3.2808399
#define FEET_TO_METERS 		0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

using namespace uas_controller;

// Default constructor
Turbulence::Turbulence() : bsiter(1000), bstime(0.02), speed(0), dir(0), kuv(-4.97391e-01), kw(-1.35341) {}

//  Configure the propulsion engine
void Turbulence::Configure(sdf::ElementPtr root, gazebo::physics::ModelPtr model)
{
	// save the model pointer
	modPtr = model;

	// GET PARAMETERS FROM THE SDF //////////////////////////////////////

	// Speed and direction
	bsiter = GetSDFInteger(root,"bsiter",bsiter);
	bstime = GetSDFDouble(root,"bstime",bstime);

	// Speed and direction
	speed = GetSDFDouble(root,"speed", speed);
	dir   = GetSDFDouble(root,"direction", gazebo::math::Rand::GetDblUniform(-MATH_PI,MATH_PI));

	// Aerodynamic drag
	kuv = GetSDFDouble(root,"drag.kuv", kuv);
	kw  = GetSDFDouble(root,"drag.kw", kw);

	// SET SOME IMPORTANT VARIABLES /////////////////////////////////////

	// Drag vector
	drag.Set(kuv,kuv,kw);
	drag *= modPtr->GetLink("body")->GetInertial()->GetMass();

	// Boostrap the wind
	Reset();
}

// Update the model based on the time step (and internal control)
void Turbulence::Update(const double &dt)
{
	// Extract the altitude and orientation from the state
	d = METERS_TO_FEET * modPtr->GetLink("body")->GetWorldLinearVel().GetLength() * dt;
	a = METERS_TO_FEET * modPtr->GetLink("body")->GetWorldPose().pos.z;
	q = modPtr->GetLink("body")->GetWorldPose().rot;

	// optimization
	k = 0.177 + 0.000823 * a;

	// sigma
	s.z = s20 * 0.1;
	s.y = s.z / pow(k, 0.4);
	s.x = s.y;
	
	// length scale
	l.z = a;
	l.y = l.z / pow(k, 1.2);
	l.x = l.y;
	
	// Gust X component
	wind.x = gazebo::math::Rand::GetDblNormal(
		(1-d/l.x) *  wind.x,	// Mean
		sqrt(2*d/l.x) * s.x		// Standard deviation
	);

	// Gust Y component
	wind.y = gazebo::math::Rand::GetDblNormal(
		(1-d/l.y) *  wind.y,	// Mean
		sqrt(2*d/l.y) * s.y		// Standard deviation
	);

	// Gust Z component
	wind.z = gazebo::math::Rand::GetDblNormal(
		(1-d/l.z) *  wind.z,	// Mean
		sqrt(2*d/l.z) * s.z		// Standard deviation
	);

	// Add the force to the body
	modPtr->GetLink("body")->AddRelativeForce
	(
		-drag*(q.RotateVector(d20.GetInverse().RotateVector(FEET_TO_METERS * wind)))
	);
}

// Reset the propulsion engine
void Turbulence::Reset()
{
	// Wind direction expressed as a rotation quaternion
	d20.SetFromEuler(0.0, 0.0, DEGREES_TO_RADIANS * dir);

	// Get the speed at 20ft
	s20 = METERS_TO_FEET * speed;

	// Boostrap the gust model, so the wind doesn't start at zero
	wind = gazebo::math::Vector3(0,0,0);
	for (int i = 0; i < bsiter; i++)
		Update(bstime);
}