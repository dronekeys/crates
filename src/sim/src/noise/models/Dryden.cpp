// For base noise type
#include "Dryden.h"

using namespace gazebo;

#define METERS_TO_FEET      3.2808399
#define FEET_TO_METERS      0.3048000
#define INIT_ITERATIONS     1000
#define INIT_DT             0.02

// Configure using the given SDF
Dryden::Dryden() : Noise(), ready(false)
{
	// Set all speeds and altitudes to zero
	for (int i = 0; i < MAX_PARS; i++)
		pars[i] = 0.0;
}

// Destructor
Dryden::~Dryden()
{
	// Do nothing
}

// Reset the nois stream
void Dryden::Reset()
{
	// Reset time
	tim = 0;

	// reset variables
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = 0;

	// Not initialised yet
	ready = false;
}

// Sample the random process
void Dryden::Bootstrap()
{
	// Extract parameters
	double a = METERS_TO_FEET * pars[DRYDEN_PARS_ALTITUDE];
	double w = METERS_TO_FEET * pars[DRYDEN_PARS_WNDSPEED];

	// For efficiency
	double k = 0.177 + 0.000823 * a;

    // Initialise sigma
    s.x = 1.0 / pow(k,0.4);
    s.y = 1.0 / pow(k,0.4);
    s.z = 1.0;
    s  *= 0.1 * w;
    
    // Initialise length scale
    l.x = 1.0 / pow(k,1.2);
    l.y = 1.0 / pow(k,1.2);
    l.z = 1.0;
    l  *= a;

	// reset variables
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = 0;

    // Bootstrap the turbulence model
    for (int i = 1; i <= INIT_ITERATIONS; i++)
        Equations(INIT_DT);
}

// Sample the random process
void Dryden::Equations(double dt)
{
	// Extract parameters
	double a = METERS_TO_FEET * pars[DRYDEN_PARS_ALTITUDE];
	double w = METERS_TO_FEET * pars[DRYDEN_PARS_WNDSPEED];
	double d = METERS_TO_FEET * pars[DRYDEN_PARS_AIRSPEED] * dt;
	double k = 0.177 + 0.000823 * a;

	// sigma
	s.z = w * 0.1;
	s.y = s.z / pow(k, 0.4);
	s.x = s.y;

	// length scale
	l.z = a;
	l.y = l.z / pow(k, 1.2);
	l.x = l.y;

	// Gust X component
	vars[0] = math::Rand::GetDblNormal(
		(1-d/l.x) *  vars[0],    // Mean
		sqrt(2*d/l.x) * s.x        // Stddev
	);

	// Gust Y component
	vars[1] = math::Rand::GetDblNormal(
		(1-d/l.y) *  vars[1],    // Mean
		sqrt(2*d/l.y) * s.y        // Stddev
	);

	// Gust Z component
	vars[2] = math::Rand::GetDblNormal(
		(1-d/l.z) *  vars[2],    // Mean
		sqrt(2*d/l.z) * s.z        // Stddev
	);
}

// Sample the random process
void Dryden::Sample(double t)
{
	// Ramp up the estimate to get to steady state
	if (!ready)
	{
		// Bootstrap
		Bootstrap();
	
	    // We are now bootstrapped
	    ready = true;
	}

	// Process equations
	if (t - tim > 0)
		Equations(t - tim);

	// Time lag
	tim = t;
}

