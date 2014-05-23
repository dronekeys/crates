// For base noise type
#include "Ornstein.h"

// Used to tokenize the parameters
#include <boost/tokenizer.hpp>

using namespace gazebo;

// Configure using the given SDF
Ornstein::Ornstein(double beta, double sigma) : Noise(), _beta(beta), _sigma(sigma)
{
	// Do nothing
}

// Destructor
Ornstein::~Ornstein()
{
	// Do nothing
}

void Ornstein::Reset()
{
	for (int i = 0; i < MAX_VARS; i++)
		vars[i] = math::Rand::GetDblNormal(0,_sigma);
}

void Ornstein::Sample(double dt)
{
    // Sample!
    for (int i = 0; i < MAX_VARS; i++)
    	vars[i] = vars[i] * exp(-_beta*dt) + math::Rand::GetDblNormal(0,_sigma);
}
