#include <dronekey_wireless/Wireless.h>
#include <string>

using namespace dronekey;

int Wireless::areaNodes()
{
	return 0;
}

Wireless::Wireless() {}


void Wireless::SetWireless(gazebo::physics::WorldPtr &world, boost::shared_ptr<ros::NodeHandle> rosNode)
{
	gWorld = world;
	gRosNode = rosNode;
}

bool Wireless::isWireless(std::string &serviceURI)
{
	return ros::service::exists(serviceURI, false);
}

bool Wireless::Send()
{
	ROS_INFO("SEND!!");
	//Get All Models
	gazebo::physics::Model_V allModels = gWorld->GetModels();
	//Model URI
	std::string serviceURI;
	

	//Parse all models and if not tracked add to list!!
	for(std::vector<gazebo::physics::ModelPtr>::iterator it = allModels.begin(); it != allModels.end(); it++)
	{
		//type Cast Model and Get Name
		gazebo::physics::Model &md = *(*it);
		std::string modelName = md.GetName();
		serviceURI = "/hal/"+modelName+"/wireless/Receive";
		if(wirelessModels.find(modelName) == wirelessModels.end() && isWireless(serviceURI) == true){
			
			ROS_INFO("NOT FOUND!!");
			hal_quadrotor::Packet pkt;

			ros::ServiceClient srvWireless = gRosNode->serviceClient<hal_quadrotor::Packet>(serviceURI);
			if(!srvWireless.call(pkt)){
				ROS_FATAL("Packet Receiver!!");
			}
			
			//add to map
			//wirelessModels[modelName] = modelName;
		}
		ROS_INFO("Model Names: %s - Type: %d", md.GetName().c_str(), md.GetType());
	}
	return true;
}

