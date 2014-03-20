// Local library incldues
#include <uas_hal/platform/UAV.h>

using namespace uas_hal;

UAV::UAV(const char *name) : HAL(name),
	actAnglesHeight(HAL::GetROSNode(), "AnglesHeight", boost::bind(&UAV::cbAnglesHeight, this, _1), false)
{
	// Periodically, a UAV may wish to broadcast its state
	pubState = GetROSNode().advertise<uas_hal::MsgState>("State", 10);

	// Periodically, a UAV may wish to broadcast information about itself
	pubInformation = GetROSNode().advertise<uas_hal::MsgInformation>("Information", 10);
}

void UAV::cbAnglesHeight(const uas_hal::AnglesHeightGoalConstPtr &goal)
{

}

// Send the current state 
void UAV::PostState(
    const double &px, const double &py, const double &pz,
    const double &rx, const double &ry, const double &rz,
    const double &vx, const double &vy, const double &vz,
    const double &ax, const double &ay, const double &az,
    const double &thrust, const double &energy)
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

// Send some general information
void UAV::PostInformation(
    const char*     id,
    const char*     version,
    const double&   uptime)
{
	// Set the message parameters
	msgInformation.tick 	= ros::Time::now();
	msgInformation.id 		= id;
	msgInformation.version 	= version;
	msgInformation.uptime 	= uptime;

	// Send the message
	pubInformation.publish(msgInformation);
}