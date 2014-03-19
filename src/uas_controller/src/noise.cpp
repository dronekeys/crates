#include "noise.h"

using namespace uas_controller;

// Boostrap with all possible noise 
Noise::Noise()
{
	types["gaussian"] 	= TYPE_GAUSSIAN;	// Gaussian
	types["grw"] 		= TYPE_GRW;			// Gaussian Random Walk
	types["oe"]			= TYPE_OE;			// Ornstein-Uhlenbeck
}

Noise::Noise(sdf::ElementPtr _sdf)
{
	// We should expect to find a single <noise> tag at the root

}