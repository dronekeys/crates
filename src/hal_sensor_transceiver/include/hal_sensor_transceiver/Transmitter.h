#ifndef HAL_SENSOR_TRANSMITTER_H
#define HAL_SENSOR_TRANSMITTER_H

#include <hal/HAL.h>

//At the moment Data and Config are same as Receiver!!
#include <hal_sensor_transceiver/TData.h>
#include <hal_sensor_transceiver/Configure.h>

namespace hal
{
	namespace sensor
	{
		class Transmitter : public hal::HAL
		{
		private:
			hal_sensor_transceiver::TData 	message;

			ros::Timer 						timerSamp, timerSend;

			ros::Publisher					publisher;

			ros::ServiceServer				service;

			void Broadcast(const ros::TimerEvent& event);

			void Sample(const ros::TimerEvent& event);

			bool Configure(
				hal_sensor_transceiver::Configure::Request &req,
				hal_sensor_transceiver::Configure::Response &res);
			
		protected:

			virtual bool GetMeasurement(hal_sensor_transceiver::TData& msg) = 0;

		public:

			Transmitter();

			void OnInit();
		};
	}
}

#endif