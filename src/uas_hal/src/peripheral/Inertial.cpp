// Local library incldues
#include <uas_hal/peripheral/Inertial.h>

using namespace uas_hal;

// Setup the altitude sensor
void Inertial::initialize(const char *name)
{
	// First, bind the ROS HAL
	bind(name);

	// Then, advertise altitude information with a max queue length of 10
	pub = getNodeHandle().advertise<uas_hal::MsgInertial>("Inertial", 10);
}

// Send some 
void Inertial::post()
{
	if (!isBound())
		ROS_WARN("Cannot send Inertial message, because not bound to ROS");
	else
	{
		// Send the message
		pub.publish(msg);
	}
}