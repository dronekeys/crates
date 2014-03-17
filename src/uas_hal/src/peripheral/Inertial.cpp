// Local library incldues
#include <uas_hal/peripheral/Inertial.h>

using namespace uas_hal;

Inertial::Inertial() : bound(false) {}

void Inertial::bind(ros::NodeHandle& nh)
{
	ROS_WARN("This is a dummy function that is not yet implemented");

	// Everything is now bound
	bound = true;
}
