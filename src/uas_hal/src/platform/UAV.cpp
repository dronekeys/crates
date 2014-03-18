// Local library incldues
#include <uas_hal/platform/UAV.h>

// For managing actions
#include <actionlib/server/simple_action_server.h>

using namespace uas_hal;

// Setup the altitude sensor
void UAV::initialize(const char *name)
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

// Send some 
void UAV::postState()
{
	if (!isBound())
		ROS_WARN("Cannot send state message, because not bound to ROS");
	else
	{
		// Send the message
		pubState.publish(msgState);
	}
}

// Send some 
void UAV::postInformation()
{
	if (!isBound())
		ROS_WARN("Cannot send information message, because not bound to ROS");
	else
	{
		// Send the message
		pubInformation.publish(msgInformation);
	}
}
