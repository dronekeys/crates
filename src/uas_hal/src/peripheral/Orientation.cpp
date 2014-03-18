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
void Orientation::post(const double &x, const double &y, const double &z)
{
	if (!isBound())
		ROS_WARN("Cannot send Orientation message, because not bound to ROS");
	else
	{
		// Set the message parameters
		msg.tick = ros::Time::now();
		msg.x = x;
		msg.y = y;
		msg.z = z;

		// Send the message
		pub.publish(msg);
	}
}

