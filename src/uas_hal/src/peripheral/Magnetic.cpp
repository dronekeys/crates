// Local library incldues
#include <uas_hal/peripheral/Magnetic.h>

using namespace uas_hal;

Magnetic::Magnetic() : bound(false) {}

void Magnetic::bind(ros::NodeHandle& nh)
{
	ROS_WARN("This is a dummy function that is not yet implemented");

	// Everything is now bound
	bound = true;
}
