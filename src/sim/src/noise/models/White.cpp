// For base noise type
#include "White.h"

using namespace gazebo;

White::White(double white) 
    : Noise(), _white(white)
{
    // Do nothing
}

double White::Sample(double dt)
{
    return math::Rand::GetDblNormal(0.0, _white);
}
