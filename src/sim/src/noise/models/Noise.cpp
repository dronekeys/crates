// For base noise type
#include "Noise.h"

using namespace gazebo;

// Constructor
Noise::Noise(std::string inval) : name(inval) {}

// Allow up to two double parameters to be configured online
void Noise::Configure(int idx, double val)
{
    if (idx < MAX_PARS)
        pars[idx] = val;
}

// Sample a scalar from the random distribution
double Noise::Get(int idx)
{
    if (enabled && idx < MAX_VARS)
        return vars[idx];
    return 0.0;
}

// Enable and disable this noise stream,
void Noise::Toggle(bool val)
{
    enabled = val;
}

// Enable and disable this noise stream,
std::string Noise::GetName()
{
    return name;
}

// Draw a scalar from the random distribution
double Noise::DrawScalar(double t)
{
    // Sample
    Sample(t);

    // Return first value
    return Get(0);
}

// Draw a vector from the random distribution
math::Vector3 Noise::DrawVector(double t)
{
    // Sample
    Sample(t);

    // Get vector
    return math::Vector3(
        Get(0),
        Get(1),
        Get(2)
    );
}