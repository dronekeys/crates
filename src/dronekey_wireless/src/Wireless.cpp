#include <dronekey_wireless/Wireless.h>
#include <string>

using namespace dronekey;

int Wireless::areaNodes()
{
	return 0;
}

Wireless::Wireless() {}


void Wireless::SetWorld(gazebo::physics::WorldPtr &world)
{
	gWorld = world;
}

bool Wireless::isWireless(std::string &modelName)
{
	std::string serviceURI;
	serviceURI = "/hal/"+modelName+"/wireless/receiver";
	return ros::service::exists(serviceURI, false);
}

bool Wireless::Send()
{
	ROS_INFO("SEND!!");
	//Get All Models
	gazebo::physics::Model_V allModels = gWorld->GetModels();
	//Parse all models and if not tracked add to list!!
	for(std::vector<gazebo::physics::ModelPtr>::iterator it = allModels.begin(); it != allModels.end(); it++)
	{
		//type Cast Model and Get Name
		gazebo::physics::Model &md = *(*it);
		std::string modelName = md.GetName();
		if(wirelessModels.find(modelName) == wirelessModels.end() && isWireless(modelName) == true){
			ROS_INFO("NOT FOUND!!");
			//
			//add to map
		
			//wirelessModels[modelName] = modelName;
		}
		ROS_INFO("Model Names: %s - Type: %d", md.GetName().c_str(), md.GetType());
	}
	return true;
}

