/* This gazebo model plugin implements a second order model for quadrotor dynamics */ 
#include "shear.h"

using namespace uas_controller;

// Default constructor
Turbulence::Turbulence() : bsiter(1000), bstime(0.02), s20(0), d20(0,0,0), wind(0,0,0) {}

// Default constructor takes configuration + pointer to link
void Turbulence::Configure(sdf::ElementPtr _sdf, const double &alt, const double &speed)
{
	/*
	    <turbulence>
	        <bsiter>1000</bsiter>
	        <bstime>0.02</bstime>
	        <direction>90.0</direction>
	        <speed>1.0</speed>
      	</turbulence>                */

	// Speed and direction
	bsiter = GetSDFInteger(sdf, "turbulence.bsiter",  bsiter);
	bstime = GetSDFDouble(sdf, "turbulence.bstime",  bstime);

	// Speed and direction (random if needed)
	double s = GetSDFDouble(sdf, "turbulence.speed", 0);
	double d = GetSDFDouble(sdf, "turbulence.direction" , 
		gazebo::math::Rand::GetDblUniform(-MATH_PI,MATH_PI));
	  
	// Set from SI -> MIL units
	SetWind(s, d);

	// Select a random direction and initlize
	Reset(alt, speed);	
}

// Set the speed (m) and direction (degrees) of the winf
void Turbulence::SetWind(const double &speed, const double &direction)
{
  // Wind direction expressed as a rotation quaternion
  d20.setFromEuler(0.0, 0.0, DEGREES_TO_RADIANS * direction);

  // Get the speed at 20ft
  s20 = METERS_TO_FEET * speed;
}

// Set the speed (m) and direction (degrees) of the winf
void Turbulence::Reset(const double &alt, const double &speed)
{
	// Boostrap the gust model, so the wind doesn't start at zero
	wind = gazebo::math::Vector3(0,0,0);
	for (int i = 0; i < bsiter; i++)
		Update(alt, speed, bstime);
}

// Get the wind vector based on the 
void Turbulence::Update(const double &alt, const double &speed, const double &dt)
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
}

// Get the wind vector based on the 
void Turbulence::GetVelocity()
{
	// Turbulence is aligned with the main turbulence direction. So,
	// we need to align it with the navigation frame
	return d20.Invert().RotateVector(FEET_TO_METERS * wind);
}