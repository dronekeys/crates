// For base noise type
#include "Dryden.h"

using namespace gazebo;

#define METERS_TO_FEET      3.2808399
#define FEET_TO_METERS      0.3048000
#define INIT_ITERATIONS     1000
#define INIT_DT             0.02
#define PARAMETER_WINDSPEED 0

Dryden::Dryden(double white) :
{
    // Bootstrap the turbulence model
    for (int i = 0; i < INIT_ITERATIONS; i++)
        DrawVector(INIT_DT);
}

Dryden::Reset()
{
	// Extract the altitude and orientation from the state
	double d = METERS_TO_FEET * link->GetWorldLinearVel().GetLength() * dt;
    double a = METERS_TO_FEET * link->GetWorldPose().pos.z;
	double k = 0.177 + 0.000823 * a;

    // Initialise sigma
    s.x = 1.0 / pow(k,0.4);
    s.y = 1.0 / pow(k,0.4);
    s.z = 1.0;
    s  *= 0.1 * param[PARAMETER_WINDSPEED];
    
    // Initialise length scale
    l.x = 1.0 / pow(k,1.2);
    l.y = 1.0 / pow(k,1.2);
    l.z = 1.0;
    l  *= a;

    // Bootstrap the turbulence model
    for (int i = 0; i < INIT_ITERATIONS; i++)
        DrawVector(INIT_DT);
}

// Sample the random process
gazebo::Vector3 Dryden::DrawVector(double dt)
{
	// Extract the altitude and orientation from the state
	double d = METERS_TO_FEET * link->GetWorldLinearVel().GetLength() * dt;
    double a = METERS_TO_FEET * link->GetWorldPose().pos.z;
	double k = 0.177 + 0.000823 * a;

	// sigma
	s.z = param[PARAMETER_WINDSPEED] * 0.1;
	s.y = s.z / pow(k, 0.4);
	s.x = s.y;

	// length scale
	l.z = a;
	l.y = l.z / pow(k, 1.2);
	l.x = l.y;

	// Gust X component
	turbulence.x = math::Rand::GetDblNormal(
		(1-d/l.x) *  turbulence.x,    // Mean
		sqrt(2*d/l.x) * s.x           // Stddev
	);

	// Gust Y component
	turbulence.y = math::Rand::GetDblNormal(
	(1-d/l.y) *  turbulence.y,    // Mean
	sqrt(2*d/l.y) * s.y           // Stddev
	);

	// Gust Z component
	turbulence.z = math::Rand::GetDblNormal(
		(1-d/l.z) *  turbulence.z,    // Mean
		sqrt(2*d/l.z) * s.z           // Stddev
	);

	// Return the turbulence vector
	return turbulence;
}

// Sample the random process
double Dryden::Sample(double dt)
{
	return ((math::Vector3)Sample(dt)).GetLength();
}