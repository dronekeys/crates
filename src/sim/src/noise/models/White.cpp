// For base noise type
#include "White.h"

using namespace gazebo;

// Configure using the given SDF
White::White(double sigma) : Noise(), _sigma(sigma)
{
	// Do nothing
}

// Destructor
White::~White()
{
	// Do nothing
}

void White::Reset()
{
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = 0.0;
}

void White::Sample(double t)
{
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = math::Rand::GetDblNormal(0,_sigma);
}
