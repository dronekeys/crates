#include "NoiseFactory.h"

using namespace gazebo;

// A list of all noise processes
std::map<std::string,Noise*> NoiseFactory::processes;

NoiseType NoiseFactory::Lookup(std::string &name)
{
	// Check to see if the type exists in the container
	if (types.find(name) == types.end())
		return UNKNOWN;

	// Check to
	return types[name];
}

// Initialise the noise factory
void NoiseFactory::Receive(NoisePtr msg)
{
	// If the model is empty and doesn't match, don't do anything
	if (msg->model().compare(model->GetName()) !=0 && !msg->model().empty())
		return;

	// Configure the noise processes
	for (ProcessVec::iterator i = processes.begin(); i != processes.end(); i++)
		if (msg->process().compare(i->first->GetName())==0 || msg->process().empty())
			i->second->Toggle(msg->enabled());
}

// Initialise the noise factory
void NoiseFactory::Init(physics::ModelPtr model)
{
	// Add the various types
	types["dryden"] 	= DRYDEN;
	types["white"] 		= WHITE;
	types["ou"] 		= ORNSTEIN;

    // Initialise a node pointer
    nodePtr = transport::NodePtr(new transport::Node());
    nodePtr->Init(model->GetWorld()->GetName());

    // Subscribe to messages about wind conditions
    subPtr = nodePtr->Subscribe("~/noise", &NoiseFactory::Receive);
}

// Initialise the noise factory
void NoiseFactory::Destroy()
{
	// Delete all factory-created processes
	for (ProcessVec::iterator i = processes.begin(); i != processes.end(); i++)
		delete i->second;
}

Noise* NoiseFactory::Create(sdf::ElementPtr root)
{
	// Get the name and type
	std::string name = root->GetName();
	std::string type = root->GetFirstElement()->GetName();

	// Check to see the root name
	Noise* noise;
	switch (Lookup(type))
	{
		case WHITE:
			noise = (Noise*) new White(name, root->GetFirstElement());
			break;

		case ORNSTEIN:
			noise = (Noise*) new Ornstein(name, root->GetFirstElement());
			break;

		case DRYDEN:
			noise = (Noise*) new Dryden(name, root->GetFirstElement());
			break;

		case UNKNOWN:
			noise = (Noise*) new Zero(name, root->GetFirstElement());
			break;
	}

	// Destroy this on
	processes.push_back(noise);

	// Success!
	return noise;
}