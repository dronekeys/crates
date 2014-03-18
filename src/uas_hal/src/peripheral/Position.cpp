// Local library incldues
#include <uas_hal/peripheral/Position.h>

using namespace uas_hal;

// Setup the altitude sensor
void Position::initialize(const char *name)
{
	// First, bind the ROS HAL
	bind(name);

	// Then, advertise altitude information with a max queue length of 10
	pub = getNodeHandle().advertise<uas_hal::MsgPosition>("Position", 10);
}

// Send some 
void Position::post()
{
	if (!isBound())
		ROS_WARN("Cannot send Position message, because not bound to ROS");
	else
	{
		// Send the message
		pub.publish(msg);
	}
}