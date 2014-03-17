// Local library incldues
#include <uas_hal/peripheral/Camera.h>

using namespace uas_hal;

Camera::Camera() : bound(false) {}

void Camera::bind(ros::NodeHandle& nh)
{
	ROS_WARN("This is a dummy function that is not yet implemented");

	// Everything is now bound
	bound = true;
}
