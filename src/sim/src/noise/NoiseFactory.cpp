#include "NoiseFactory.h"

using namespace gazebo;


NoiseFactory::NoiseFactory()
{
	types["white"] 		= WHITE;
	types["wiener"] 	= WIENER;
	types["ou"] 		= ORNSTEIN;
	types["uniform"] 	= UNIFORM;
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

bool NoiseFactory::Create(sdf::ElementPtr root)
{
	/* 
		This is what will be passed in as 'root'...
	    <variable>
	      <gaussian>
	        <mean>1.005</mean>
	        <sdev>0.003</sdev>
	      </gaussian>
	    </orbits>
	*/

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

		case WIENER:
			root->GetFirstElement()->GetElement("bias")->GetValue()->Get(p1);
			root->GetFirstElement()->GetElement("white")->GetValue()->Get(p2);
			processes[name] = (Noise*) new Wiener(p1, p2);
			break;

		case ORNSTEIN:
			root->GetFirstElement()->GetElement("lambda")->GetValue()->Get(p1);
			root->GetFirstElement()->GetElement("bias")->GetValue()->Get(p2);
			root->GetFirstElement()->GetElement("white")->GetValue()->Get(p3);
			processes[name] = (Noise*) new Ornstein(p1, p2, p3);
			break;

		case UNIFORM:
			root->GetFirstElement()->GetElement("lower")->GetValue()->Get(p1);
			root->GetFirstElement()->GetElement("upper")->GetValue()->Get(p2);
			processes[name] = (Noise*) new Uniform(p1, p2);
			break;

        case UNKNOWN:
        	return false;
	}

	// Success!
	return true;
}

double NoiseFactory::Sample(std::string& name, double dt)
{
	// Todo: throw and error
	if (processes.find(name) == processes.end())
		return 0.0;

	// Sample the process
	return processes[name]->Sample(dt);
}
