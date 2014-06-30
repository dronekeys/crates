#ifndef SIM_SENSOR_RECEIVER_H
#define SIM_SENSOR_RECEIVER_H

// HAL functionality
#include <hal_sensor_transceiver/Receiver.h>

//Basic sensor functionality
#include "Sensor.h"

//Inherit Gazebo Receiver
//#include <gazebo/sensors/WirelessReceiver.hh>

namespace gazebo
{
	class Receiver : public Sensor
	//public gazebo::sensors::WirelessReceiver
	{
	private:

		physics::LinkPtr 	linkPtr;

		Noise 				*nLinAcc, *nAngVel;
	public:
		
		Receiver();

		bool Configure(physics::LinkPtr link, sdf::ElementPtr root);

		void Reset();

		bool GetMeasurement(double t, hal_sensor_transceiver::Data& msg);
	};
}

#endif