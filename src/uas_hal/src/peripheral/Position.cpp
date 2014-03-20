// Local library incldues
#include <uas_hal/peripheral/Position.h>

using namespace uas_hal;

// Setup the altitude sensor
Position::Position(const char *name) : HAL(name)
{
	// Then, advertise altitude information with a max queue length of 10
	pub = GetROSNode().advertise<uas_hal::MsgPosition>("Position", 10);
}

// Send some 
void Position::Post(
	const char *	source,
	const char *	status,
	const char *	cframe,
	const double & 	x,
	const double & 	y,
	const double & 	z,
	const double & 	dx,
	const double & 	dy,
	const double & 	dz,
	const double & 	err_x,
	const double & 	err_y,
	const double & 	err_z,
	const double & 	err_dx,
	const double & 	err_dy,
	const double & 	err_dz,
	const double & 	err_clk)
{

	// Set the message parameters
	msg.tick   	= ros::Time::now();
	msg.source 	= source;
	msg.status 	= status;
	msg.cframe  = cframe;
	msg.x  		= x;
	msg.y  		= y;
	msg.z  		= z;
	msg.dx 		= dx;
	msg.dy  	= dy;
	msg.dz 		= dz;
	msg.err_x  	= err_x;
	msg.err_y  	= err_y;
	msg.err_z  	= err_z;
	msg.err_dx 	= err_dx;
	msg.err_dy  = err_dy;
	msg.err_dz 	= err_dz;
	msg.err_clk = err_clk;

	// Send the message
	pub.publish(msg);
}