// For base noise type
#include "Zero.h"

using namespace gazebo;

// Configure using the given SDF
Zero::Zero(std::string name, sdf::ElementPtr root) : Noise(name)
{
    for (int i = 0; i < MAX_VARS; i++)
        vars[i] = 0.0;
}

void Zero::Reset()
{
	// Do nothing
}

void Zero::Sample(double dt)
{
    // Do nothing
}