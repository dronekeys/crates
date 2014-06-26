#include <hal_sensor_transceiver/Receiver.h>

using namespace hal::sensor;

Receiver::Receiver() : hal::HAL()
{
	//DO NOTHING
}

void Receiver::OnInit()
{
	//DO NOTHING
}

bool Receiver::Configure(hal_sensor_transceiver::Configure::Request &req, hal_sensor_transceiver::Configure::Response &res)
{
	return false;
}

void Receiver::Broadcast(const ros::TimerEvent& event)
{

}

void Receiver::Sample(const ros::TimerEvent& event)
{

}