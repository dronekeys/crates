// Local library incldues
#include <uas_hal/peripheral/Orientation.h>

using namespace uas_hal;

// Setup the altitude sensor
Orientation::Orientation(const char *name) : HAL(name)
{
	// Then, advertise altitude information with a max queue length of 10
	pub = GetROSNode().advertise<uas_hal::MsgOrientation>("Orientation", 10);
}

// Send some 
void Orientation::Post(const double &x, const double &y, const double &z)
{
	// Set the message parameters
	msg.tick = ros::Time::now();
	msg.x = x;
	msg.y = y;
	msg.z = z;

	// Send the message
	pub.publish(msg);
}

