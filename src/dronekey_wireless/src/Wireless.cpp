#include <dronkey_wireless/Wireless.h>
#include <string>
#include <sim/Wireless.h>

using namespace dronkey;

int Wireless::areaNodes()
{
	return 0;
}

Wireless::Wireless()
{
	ROS_INFO("HERE WE GO!!");
}


void Wireless::SetWorld(gazebo::physics::WorldPtr &world)
{
	gWorld = world;
}

bool Wireless::Send()
{

	gazebo::physics::Model_V allModels = gWorld->GetModels();	
	for(std::vector<gazebo::physics::ModelPtr>::iterator it = allModels.begin(); it != allModels.end(); it++)
	{
		gazebo::physics::Model &md = *(*it);
		ROS_INFO("Model Names: %s - Type: %d", md.GetName().c_str(), md.GetType());
	}
	return true;
}

