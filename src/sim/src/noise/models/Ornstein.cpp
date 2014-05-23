// For base noise type
#include "Ornstein.h"

// Used to tokenize the parameters
#include <boost/tokenizer.hpp>

using namespace gazebo;

// Configure using the given SDF
Ornstein::Ornstein(std::string name, sdf::ElementPtr root) : Noise(name)
{
	// Obtain the parameter tokens
	std::string tokenstring;
	root->GetValue()->Get(tokenstring);
}

void Ornstein::Reset()
{
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = math::Rand::GetDblNormal(0,cfg[i][IDX_SIGMA]);
}

void Ornstein::Sample(double dt)
{
    // Sample!
    for (int i = 0; i < MAX_VARS; i++)
    	vars[i] = vars[i] * exp(-cfg[i][IDX_BETA]*dt) + math::Rand::GetDblNormal(0,cfg[i][IDX_SIGMA]);
}
