#ifndef HAL_SENSOR_RADIO_H
#define HAL_SENSOR_RADIO_H

// Header libraries
#include <hal/sensor/Sensor.h>

// Message libraries
#include <hal_sensor_radio/Data.h>
#include <hal_sensor_radio/Packet.h>

#define MAX_PACKET_LENGTH 128

namespace hal
{
    namespace sensor
    {
        class Radio : public Sensor<hal_sensor_radio::Data>
        {   

    	private:

    		/// Local service storage
    		hal_sensor_radio::Packet srv;

    		/// The handle to manage reception of data
    		ros::ServiceServer service;

    		/// The handle to manage transmitting data 
    		ros::ServiceClient client;

        	//! Receive some bytes from the medium
		    /*!
		      \param req the request
		      \param res the response
		    */
			bool Rx(hal_sensor_radio::Packet::Request &req, hal_sensor_radio::Packet::Response &res);


        public:

        	/// Basic constructor
        	Radio(ros::NodeHandle& node);
		  
        };
    }
}

#endif