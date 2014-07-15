#ifndef HAL_QUADROTOR_COMMUNICATION_H
#define HAL_QUADROTOR_COMMUNICATION_H

#include <hal_quadrotor/Packet.h>

#include <ros/ros.h>

namespace hal
{
	namespace quadrotor 
	{
		class Communication 
		{
		private:
			
			ros::ServiceServer srvReceive;

			bool RcvPacket(
				hal_quadrotor::Packet::Request &req,
				hal_quadrotor::Packet::Response &res);
		public:

			void Init(ros::NodeHandle nh);
		};
	}
}

#endif