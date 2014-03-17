// Local library incldues
#include <uas_hal/peripheral/Radio.h>

using namespace uas_hal;

Radio::Radio() : bound(false) {}

void Radio::bind(ros::NodeHandle& nh)
{
	ROS_WARN("This is a dummy function that is not yet implemented");

	// Everything is now bound
	bound = true;
}
