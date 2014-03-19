/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "shear.h"

using namespace uas_controller;

// Default constructor
Turbulence::Turbulence() : _biter(1000), _bdt(0.02) {}

// Set the speed (m) and direction (degrees) of the winf
void Turbulence::Init(const double &speed)
{
	// Select a random direction and initlize
	Initialize(speed, gazebo::math::Rand::GetDblUniform(-MATH_PI,MATH_PI));
}

// Set the speed (m) and direction (degrees) of the winf
void Turbulence::Init(const double &speed, const double &direction)
{
	// Wind always blows orthogonal to the down direction
	d20.SetFromEuler(0.0, 0.0, DEGREES_TO_RADIANS * direction);

	// Get the speed at 20ft
	s20 = METERS_TO_FEET * speed;

	// Boostrap the gust model, so the wind doesn't start at zero
	gust = gazebo::math::Vector3(0,0,0);
	for (int i = 0; i < _biter; i++)
		GetGlobalVelocity(_bdt);
}

// Get the wind vector based on the 
gazebo::math::Vector3 Turbulence::GetGlobalVelocity(const double &alt, const double &speed, const double &dt)
{
	// wind displacement in feet over this timestep
	d = METERS_TO_FEET * speed * dt;
	
	// current altitude
	h = METERS_TO_FEET * alt;

	// optimization
	k = 0.177 + 0.000823 * h;

	// sigma
	s.z = s20 * 0.1;
	s.y = s.z / pow(k, 0.4);
	s.x = s.y;
	
	// length scale
	l.z = h;
	l.y = hl.z / pow(k, 1.2);
	l.x = l.y;
	
	// Gust X component
	gust.x = gazebo::math::Rand::GetDblNormal(
		(1-d/l.x) *  gust.x,	// Mean
		sqrt(2*d/l.x) * s.x		// Standard deviation
	);

	// Gust Y component
	gust.y = gazebo::math::Rand::GetDblNormal(
		(1-d/l.y) *  gust.y,	// Mean
		sqrt(2*d/l.y) * s.y		// Standard deviation
	);

	// Gust Z component
	gust.z = gazebo::math::Rand::GetDblNormal(
		(1-d/l.z) *  gust.z,	// Mean
		sqrt(2*d/l.z) * s.z		// Standard deviation
	);

	// Turbulence is aligned with the main turbulence direction. So,
	// we need to align it with the navigation frame
	return d20.Invert().RotateVector(FEET_TO_METERS * gust);
}