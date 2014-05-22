// For base noise type
#include "Noise.h"

using namespace gazebo;

// Incoming noise message
void Noise::Receive(NosiePtr noise)
{
    // Enable or disable the noise
    enabled = noise->enabled();
}

// Configure the noise distributino
void Noise::Configure(int num, ...)
{
    // A place to store the list of arguments
    va_list arguments;
    va_start(arguments, num);
    for (int x = 0; x < num; x++)
        params[x] = va_arg(arguments,double);
    va_end(arguments);

    // Issue a reset after recalibration
    Reset();
}

// Draw a vector from the distribution
gazebo::Vector3 Noise::DrawVector(physics::linkPtr link, double dt = 0)
{
    if (enabled)
        return Sample(linkPtr, dt);
    return gazebo::Vector3(0,0,0);
}

// Draw a scalar from the distribution
double Noise::DrawScalar(physics::linkPtr link, double dt = 0)
{
    if (enabled)
        return Sample(linkPtr, dt);
    return 0;
}