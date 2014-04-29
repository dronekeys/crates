// Library headers
#include <hal/sensor/Radio.h>

using namespace hal::sensor;

Radio::Radio(ros::NodeHandle& node) : Sensor<hal_sensor_radio::Data>(node, "radio")
{
    // Advertice the ability to receive a data packet
    service = rosNode.advertiseService("sensors/radio/Transmit", &Radio::Rx, this);

    // The client will call services on other nodes
	client  = rosNode.serviceClient<hal_sensor_radio::Packet>("transmit");
} 

bool Radio::Rx(hal_sensor_radio::Packet::Request &req, hal_sensor_radio::Packet::Response &res)
{
	res.success = true;
	res.status  = "Packet received succesfully";
	return true;
}
