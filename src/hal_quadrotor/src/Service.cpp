// Standard libraries
#include <hal_quadrotor/Service.h>

using namespace hal_quadrotor;

// Constructor creates publisher handle and sets default rate
Service::Service(ros::NodeHandle &node, std::string name)
{
    // Advertise this message on the ROS backbone
    service = node.advertiseService(name.c_str(), &controller::Receive, this);
}
