#include <hal_quadrotor/Communication.h>

using namespace hal::quadrotor;

void Communication::Init(ros::NodeHandle nh)
{
	srvReceive = nh.advertiseService("wireless/Receive", &Communication::RcvPacket, this);
}

bool Communication::RcvPacket(
	hal_quadrotor::Packet::Request &req,
	hal_quadrotor::Packet::Response &res)
{
	ROS_INFO("PACKET RECEIVED!!: IP: %s MSG: %s", req.IP.c_str(), req.msg.c_str());
	return true;
}