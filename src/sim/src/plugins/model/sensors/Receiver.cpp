#include "Receiver.h"

using namespace gazebo;

Receiver::Receiver(){
	ROS_INFO("RECEIVER INIT!!");
}

bool Receiver::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{

	linkPtr = link;

	return true;
}

void Receiver::Reset()
{

}

bool Receiver::GetMeasurement(double t, hal_sensor_transceiver::Data& msg)
{
	msg.gain = 3.4;
	msg.power = 2.3;
	 
	return true;
}