// Local library incldues
#include <uas_hal/peripheral/Inertial.h>

using namespace uas_hal;

// Setup the altitude sensor
Inertial::Inertial(const char *name) : HAL(name)
{
	// Then, advertise altitude information with a max queue length of 10
	pub = GetROSNode().advertise<uas_hal::MsgInertial>("Inertial", 10);
}

// Send some 
void Inertial::Post(
            const double &gyr_x,
            const double &gyr_y,
            const double &gyr_z,
            const double &acc_x,
            const double &acc_y,
            const double &acc_z)
{

	// Set the message parameters
	msg.tick  = ros::Time::now();
	msg.gyr_x = gyr_x;
	msg.gyr_y = gyr_y;
	msg.gyr_z = gyr_z;
	msg.acc_x = acc_x;
	msg.acc_y = acc_y;
	msg.acc_z = acc_z;

	// Send the message
	pub.publish(msg);
}