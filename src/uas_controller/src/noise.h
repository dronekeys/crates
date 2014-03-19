#ifndef UAS_CONTROLLER_NOISE_H
#define UAS_CONTROLLER_NOISE_H

// Required for the maths functions
#include <gazebo/gazebo.hh>

// Naximum number of parameters and variables for noise models
#define MAX_PARS 	8
#define MAX_VARS 	8

namespace uas_controller
{
	// All possible noise types
	typedef enum
	{
		NOISE_GAUSSIAN = 1,
		NOISE_GRW 	   = 2,
		NOISE_OE
	}
	NoiseType;

	// A model
	typedef struct
	{
		NoiseType	type;
		double 		pars[MAX_PARS];
		double 		pars[MAX_VARS];
	} 
	NoiseModel;

	// Manages noise models for the simulation
    class Noise
    {

    private:

		// All possible noise types
		boost::unordered_map<std::string,NoiseType> 	types;

		// Maintains a list of noise models
		boost::unordered_map<std::string,NoiseModel> 	vars;

    public:

    	// Default constructor
    	Noise();

    	// Constructor takes a <noise></noise> SDF block
    	Noise(sdf::ElementPtr _sdf);

    	// Draw a random error perturbation from the distribution,
    	// given the fact that a certain time has elapsed
    	double Draw(const double &dt);

    };
}

#endif