// Local library incldues
#include <uas_hal/peripheral/Altitude.h>

using namespace uas_hal;

// Setup the altitude sensor
Altitude::Altitude(const char *name) : HAL(name)
{
	// Then, advertise altitude information with a max queue length of 10
	pub = GetROSNode().advertise<uas_hal::MsgAltitude>("Altitude", 10);
}

// Send some 
void Altitude::Post(const double &height, const double &speed)
{
	// Set the message parameters
	msg.tick   = ros::Time::now();
	msg.height = height;
	msg.speed  = speed;

	// Send the message
	pub.publish(msg);
}