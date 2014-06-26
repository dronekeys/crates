#include "Receiver.h"

using namespace gazebo;

Receiver::Receiver(){}

bool Receiver::Configure(physics::LinkPtr link, sdf::ElementPtr root)
{
	return true;
}

void Receiver::Reset()
{

}

bool Receiver::GetMeasurement(double t, hal_sensor_transceiver::Data& msg)
{
	return true;
}