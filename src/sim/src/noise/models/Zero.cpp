// For base noise type
#include "Zero.h"

using namespace gazebo;

// Configure using the given SDF
Zero::Zero(std::string name, sdf::ElementPtr root) : Noise(name)
{
    Reset();

}

void Zero::Reset(double dt)
{
    for (int i = 0; i < MAX_VARS; i++)
        vars[i] = 0.0;
}

void Zero::Sample(double dt)
{
    // Do nothing
}