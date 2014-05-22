// For base noise type
#include "White.h"

using namespace gazebo;

// Configure using the given SDF
White::White(std::string name, sdf::ElementPtr root) : Noise(name)
{
	Reset();
}

void White::Reset()
{
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = 0.0;
}

void White::Sample(double dt)
{
    // Do nothing
}
