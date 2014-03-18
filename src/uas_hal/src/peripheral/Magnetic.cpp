// Local library incldues
#include <uas_hal/peripheral/Magnetic.h>

using namespace uas_hal;

// Setup the altitude sensor
void Magnetic::initialize(const char *name)
{
	// First, bind the ROS HAL
	bind(name);

	// Then, advertise altitude information with a max queue length of 10
	pub = getNodeHandle().advertise<uas_hal::MsgMagnetic>("Magnetic", 10);
}

// Send some 
void Magnetic::post(
            const double &x,    // Body-frame magnetic field X (Gauss)
            const double &y,    // Body-frame Magnetic field Y (Gauss)
            const double &z)    // Body-frame Magnetic field Z (Gauss)
{
	if (!isBound())
		ROS_WARN("Cannot send Magnetic message, because not bound to ROS");
	else
	{
		// Set the message parameters
		msg.tick = ros::Time::now();
		msg.mag_x = x;
		msg.mag_y = y;
		msg.mag_z = z;

		// Send the message
		pub.publish(msg);
	}
}