#ifndef HAL_SENSOR_RECEIVER_H
#define HAL_SENSOR_RECEIVER_H

#include <hal/HAL.h>

#include <hal_sensor_transceiver/Data.h>

#include <hal_sensor_transceiver/Configure.h>

namespace hal
{
	namespace sensor
	{
		class Receiver : public hal::HAL
		{
		private:
			hal_sensor_transceiver::Data	message;

			ros::Timer					timerSamp, timerSend;

			ros::Publisher 				publisher;

			ros::ServiceServer			service;

			void Broadcast(const ros::TimerEvent& event);

			void Sample(const ros::TimerEvent& event);

			bool Configure(
				hal_sensor_transceiver::Configure::Request &req,
				hal_sensor_transceiver::Configure::Response &res);

		protected:

			virtual bool GetMeasurement(hal_sensor_transceiver::Data& msg) = 0;

		public:

			Receiver();

			void OnInit();
		};
	}
}

#endif