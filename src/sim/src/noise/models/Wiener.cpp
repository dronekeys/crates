// For base noise type
#include "Wiener.h"

using namespace gazebo;

Wiener::Wiener(double bias, double white)
    : Noise(), _bias(bias), _white(white)
{
    // Do nothing
    current = math::Rand::GetDblNormal(0.0,_bias);
}

double Wiener::Sample(double dt)
{
    // Do nothing
    current = current + math::Rand::GetDblNormal(0.0, _bias);

    // Return the value
    return math::Rand::GetDblNormal(0.0,_white) + current;
}
