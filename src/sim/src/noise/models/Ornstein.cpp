// For base noise type
#include "Ornstein.h"

using namespace gazebo;

// Configure using the given SDF
Ornstein::Ornstein(std::string name, sdf::ElementPtr root) : Noise(name)
{

}

void Ornstein::Reset()
{
	// Do nothing
}

void Ornstein::Sample(double dt)
{
    // Do nothing
}
