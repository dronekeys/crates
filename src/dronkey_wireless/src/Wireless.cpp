#include <dronkey_wireless/Wireless.h>

#include <sim/Wireless.h>

using namespace dronkey;

Wireless::Wireless(gazebo::physics::WorldPtr &world)
{
	gWorld = world;
	ROS_INFO("HERE WE GO!!");
	Send();
}

bool Wireless::Send()
{
	ROS_INFO("HOLAA!! - %d", gWorld->GetModelCount());
	return true;
}

