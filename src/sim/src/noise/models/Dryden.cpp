// For base noise type
#include "Dryden.h"

using namespace gazebo;

Dryden::Dryden(double white) :
{
    // Do nothing
}

double Dryden::Sample(double dt)
{
	// Extract the altitude and orientation from the state
	double d = METERS_TO_FEET * modPtr->GetLink("body")->GetWorldLinearVel().GetLength() * dt;
	double k = 0.177 + 0.000823 * a;

	// sigma
	s.z = s20 * 0.1;
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
}
