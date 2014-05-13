// For base noise type
#include "Uniform.h"

using namespace gazebo;

Uniform::Uniform(double lower, double upper)
    : Noise(),_lower(lower), _upper(upper)
{
    // Do nothing
}

double Uniform::Sample(double dt)
{
    return math::Rand::GetDblUniform(_lower, _upper);
}
