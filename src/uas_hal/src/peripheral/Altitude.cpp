// Local library incldues
#include <uas_hal/peripheral/Altitude.h>

using namespace uas_hal;

// Setup the altitude sensor
void Altitude::initialize(const char *name)
{
	// First, bind the ROS HAL
	bind(name);

	// Then, advertise altitude information with a max queue length of 10
	pub = getNodeHandle().advertise<uas_hal::MsgAltitude>("Altitude", 10);
}

// Send some 
void Altitude::post(const double &height, const double &speed)
{
	if (!isBound())
		ROS_WARN("Cannot send Altitude message, because not bound to ROS");
	else
	{
		// Set the message parameters
		msg.tick   = ros::Time::now();
		msg.height = height;
		msg.speed  = speed;

		// Send the message
		pub.publish(msg);
	}
}