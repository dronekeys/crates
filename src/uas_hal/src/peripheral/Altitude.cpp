// Local library incldues
#include <uas_hal/peripheral/Altitude.h>

using namespace uas_hal;

Altitude::Altitude() : bound(false) {}

void Altitude::bind(ros::NodeHandle& nh)
{
	ROS_WARN("This is a dummy function that is not yet implemented");

	// Everything is now bound
	bound = true;
}
