#include "NoiseFactory.h"
#include <ros/ros.h>
using namespace gazebo;

// Forward declaration of statics
ProcessVec               NoiseFactory::processes;

// Initialise the noise factory
void NoiseFactory::Toggle(bool enabled)
{
	// Configure the noise processes
	for (ProcessVec::iterator i = processes.begin(); i != processes.end(); i++)
		(*i)->Toggle(enabled);
}

// Initialise the noise factory
void NoiseFactory::Init()
{
	// Do nothing
}

// Initialise the noise factory
void NoiseFactory::Destroy()
{
	// Delete all factory-created processes
	for (ProcessVec::iterator i = processes.begin(); i != processes.end(); i++)
		delete (*i);
}

Noise* NoiseFactory::Create(sdf::ElementPtr root)
{
	// Get the name and type
	std::string type = root->GetFirstElement()->GetName();
	
	// Create the noise distribution
	Noise *noise; 
	if (type.compare("dryden")==0)
	{
		// Create the noise process
		noise = (Noise*) new Dryden();
	}
	else if (type.compare("white")==0)
	{
		// Get parameters
		double sigma;
		root->GetFirstElement()->GetElement("sigma")->GetValue()->Get(sigma);

		// ROS_WARN("WHITE: %f", sigma);

		// Create the noise process
		noise = (Noise*) new White(sigma);
	}
	else if (type.compare("ou")==0)
	{
		// Get parameters
		double beta, sigma;
		root->GetFirstElement()->GetElement("beta")->GetValue()->Get(beta);
		root->GetFirstElement()->GetElement("sigma")->GetValue()->Get(sigma);

		// ROS_WARN("OU: %f %f", beta, sigma);

		// Create the noise process
		noise = (Noise*) new Ornstein(beta, sigma);
	}
	else
	{
		// Create the noise process
		noise = (Noise*) new Zero();
	}

	// Destroy this on
	processes.push_back(noise);

	// Success!
	return noise;
}