// This component
#include <hal_quadrotor/configuration/Rate.h>

// Constraints
#define RATE_MIN 0
#define RATE_MAX 128

using namespace hal_quadrotor;

// Configure data broadcast at a given rate (<= 0.0 means disable)
bool Rate::Receive(ConfigRate::Request &req, ConfigRate::Response &res)
{
	if (req.rate < RATE_MIN || req.rate > RATE_MAX)
	{
		res.success = false;
		res.status  = "Rate must be between RATE_MIN and RATE_MAX";
	}
	else if (ConfigRate(req.name,req.rate))
	{
		res.success = true;
		res.status  = "Successfully updated topic broadcast rate";
	}
	else
	{
		res.success = false;
		res.status  = "Invalid topic, so rate not updated";
	}
}

// Constructor
Rate::Rate(ros::NodeHandle &node, std::string name)
{
 	// Advertise this message on the ROS backbone
    service = node.advertiseService(name.c_str(), &Rate::Receive, this);
}
