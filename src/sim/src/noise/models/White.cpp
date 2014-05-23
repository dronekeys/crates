// For base noise type
#include "White.h"

using namespace gazebo;

// Configure using the given SDF
White::White(std::string name, sdf::ElementPtr root) : Noise(name)
{
	// Obtain the parameter tokens
	std::string tokenstring;
	root->GetValue()->Get(tokenstring);
}

void White::Reset()
{
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = 0.0;
}

void White::Sample(double dt)
{
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = math::Rand::GetDblNormal(0,cfg[i]);
}
