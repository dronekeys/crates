#include "Transmitter.h"

using namespace gazebo;

Transmitter::Transmitter() {}

bool Transmitter::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	linkPtr = link;
	return true;
}

void Transmitter::Reset() {}

bool Transmitter::GetMeasurement(double t, hal_sensor_transceiver::TData& msg)
{
	msg.gain = sensors::WirelessTransmitter::GetGain();
	msg.power = sensors::WirelessTransmitter::GetPower();
	return true;
}