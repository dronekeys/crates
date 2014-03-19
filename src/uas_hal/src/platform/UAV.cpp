// Local library incldues
#include <uas_hal/platform/UAV.h>

// For managing actions
#include <actionlib/server/simple_action_server.h>

using namespace uas_hal;

// Setup the altitude sensor
void UAV::HalInit(const char *name)
{
	// Bind to ROS
	bind(name);

	// Publish state
	pubState = 
		getNodeHandle().advertise<uas_hal::MsgState>("State", 10);

	// Publish information
	pubInformation = 
		getNodeHandle().advertise<uas_hal::MsgInformation>("Information", 10);
}

// Send the current state 
void UAV::HalBroadcastState(
    const double &px, const double &py, const double &pz,
    const double &rx, const double &ry, const double &rz,
    const double &vx, const double &vy, const double &vz,
    const double &ax, const double &ay, const double &az,
    const double &thrust, const double &energy)
{
	if (!isBound())
		ROS_WARN("Cannot send Braodcast message, because not bound to ROS");
	else
	{
		// Set the message parameters
		msgState.tick   = ros::Time::now();
		msgState.px 	= px;
		msgState.py 	= py;
		msgState.pz 	= pz;
		msgState.rx 	= rx;
		msgState.ry 	= ry;
		msgState.rz 	= rz;
		msgState.vx 	= vx;
		msgState.vy 	= vy;
		msgState.vz 	= vz;
		msgState.ax 	= ax;
		msgState.ay 	= ay;
		msgState.az 	= az;
		msgState.thrust = thrust;
		msgState.energy = energy;

		// Send the message
		pubState.publish(msgState);
	}
}

// Send some general information
void UAV::HalBroadcastInformation(
    const char*     id,
    const char*     version,
    const double&   uptime)
{
	if (!isBound())
		ROS_WARN("Cannot send Information message, because not bound to ROS");
	else
	{
		// Set the message parameters
		msgInformation.tick 	= ros::Time::now();
		msgInformation.id 		= id;
		msgInformation.version 	= version;
		msgInformation.uptime 	= uptime;

		// Send the message
		pubInformation.publish(msgInformation);
	}
}