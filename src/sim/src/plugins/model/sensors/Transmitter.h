#ifndef SIM_SENSOR_TRANSMITTER_H
#define SIM_SENSOR_TRANSMITTER_H

#include <hal_sensor_transceiver/Transmitter.h>

#include "Sensor.h"

#include <gazebo/sensors/WirelessTransmitter.hh>

namespace gazebo
{
	class Transmitter: public Sensor,
	public gazebo::sensors::WirelessTransmitter
	{
	private:

		physics::LinkPtr 	linkPtr;

		Noise 				*nLinAcc, *nAngVel;
	public:

		Transmitter();

		bool Configure(physics::LinkPtr link, sdf::ElementPtr root);

		void Reset();

		bool GetMeasurement(double t, hal_sensor_transceiver::TData& msg);
	};
}

#endif