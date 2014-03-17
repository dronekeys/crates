// Local library incldues
#include <uas_hal/peripheral/Position.h>

using namespace uas_hal;

Position::Position() : bound(false) {}

void Position::bind(ros::NodeHandle& nh)
{
	ROS_WARN("This is a dummy function that is not yet implemented");

	// Everything is now bound
	bound = true;
}
