#include "NoiseFactory.h"

using namespace gazebo;

// A list of all noise processes
std::map<std::string,Noise*> NoiseFactory::processes;

NoiseFactory::NoiseFactory()
{
	types["dryden"] 	= DRYDEN;
	types["white"] 		= WHITE;
	types["ou"] 		= ORNSTEIN;
}

NoiseFactory::~NoiseFactory()
{
	// Delete all factory-created processes
	for (std::map<std::string,Noise*>::iterator i = processes.begin(); i != processes.end(); i++)
		delete i->second;
}

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
		if (msg->process().compare(i->first)==0 || msg->process().empty())
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

	// Check to see if the type exists in the container
	if (processes.find(name) != processes.end())
		return false;

	// Check to see the root name
	double p1, p2, p3;
	switch (Lookup(type))
	{
		case WHITE:
			root->GetFirstElement()->GetElement("white")->GetValue()->Get(p1);
			processes[name] = (Noise*) new White(p1);
			break;

		case ORNSTEIN:
			root->GetFirstElement()->GetElement("lambda")->GetValue()->Get(p1);
			root->GetFirstElement()->GetElement("bias")->GetValue()->Get(p2);
			root->GetFirstElement()->GetElement("white")->GetValue()->Get(p3);
			processes[name] = (Noise*) new Ornstein(p1, p2, p3);
			break;

		case DRYDEN:
			root->GetFirstElement()->GetElement("lower")->GetValue()->Get(p1);
			root->GetFirstElement()->GetElement("upper")->GetValue()->Get(p2);
			processes[name] = (Noise*) new Dryden(p1, p2);
			break;

		case UNIFORM:
			root->GetFirstElement()->GetElement("lower")->GetValue()->Get(p1);
			root->GetFirstElement()->GetElement("upper")->GetValue()->Get(p2);
			processes[name] = (Noise*) new Uniform(p1, p2);
			break;

        case UNKNOWN:
        	return NULL;
	}

	// Success!
	return processes[name];
}

double NoiseFactory::Sample(std::string& name, double dt)
{
	// Todo: throw and error
	if (processes.find(name) == processes.end())
		return 0.0;

	// Sample the process
	return processes[name]->Sample(dt);
}
