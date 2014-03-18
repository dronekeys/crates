// Local library incldues
#include <uas_hal/peripheral/Orientation.h>

using namespace uas_hal;

// Setup the altitude sensor
void Orientation::initialize(const char *name)
{
	// First, bind the ROS HAL
	bind(name);

	// Then, advertise altitude information with a max queue length of 10
	pub = getNodeHandle().advertise<uas_hal::MsgOrientation>("Orientation", 10);
}

// Send some 
void Orientation::post()
{
	if (!isBound())
		ROS_WARN("Cannot send Orientation message, because not bound to ROS");
	else
	{
		// Send the message
		pub.publish(msg);
	}
}