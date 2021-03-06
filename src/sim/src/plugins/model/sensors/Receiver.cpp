#include "Receiver.h"

using namespace gazebo;

Receiver::Receiver() {}

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
	msg.gain = sensors::WirelessReceiver::GetGain();
	msg.power = sensors::WirelessReceiver::GetPower();
	return true;
}