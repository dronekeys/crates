// For base noise type
#include "Zero.h"

using namespace gazebo;

// Configure using the given SDF
Zero::Zero() : Noise()
{
    for (int i = 0; i < MAX_VARS; i++)
        vars[i] = 0.0;
}

// Destructor
Zero::~Zero()
{
	// Do nothing
}

void Zero::Reset()
{
	// Do nothing
}

void Zero::Sample(double dt)
{
    // Do nothing
}