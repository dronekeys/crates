// For base noise type
#include "Ornstein.h"

using namespace gazebo;

Ornstein::Ornstein(double theta, double bias, double white) 
    : Noise(), _theta(theta), _bias(bias), _white(white)
{
    current = math::Rand::GetDblNormal(0.0, _bias);
}

double Ornstein::Sample(double dt)
{
    // Update internal state
    current = current * exp(-_theta * dt) + math::Rand::GetDblNormal(0, _bias);

    // Return the sample
    return current + math::Rand::GetDblNormal(0, _white);
}