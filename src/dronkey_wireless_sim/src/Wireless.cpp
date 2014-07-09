#include <dronkey_wireless/wireless.h>

using namespace dronkey;

Wireless::Wireless()
{
	//Send Packet
	ros::AdvertiseServiceOptions adSendPacket = ros::AdvertiseServiceOptions::create<dronkey_wireless::Packet>(
		"Send",boost::bind(&Wireless::Send,this,_1,_2),ros::VoidPtr(), &queue
	);
	serviceSend = rosNode->advertiseService(adSendPacket);
}

bool Wireless::Send(dronkey_wireless::Packet::Request &req, dronkey_wireless::Packet::Response &res)
{

}

