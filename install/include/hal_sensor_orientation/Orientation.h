#ifndef HAL_SENSOR_ORIENTATION_H
#define HAL_SENSOR_ORIENTATION_H

// Header libraries
#include <hal/HAL.h>

// Message libraries
#include <hal_sensor_orientation/Data.h>

// For blank requests
#include <hal_sensor_orientation/Configure.h>

namespace hal
{
    namespace sensor
    {
        class Orientation : public hal::HAL
        {     
        private:

            /// Templated message type
            hal_sensor_orientation::Data    message;

            /// Callback timer for status message updates
            ros::Timer                      timerSamp, timerSend;

            /// Used to broadcast Status message
            ros::Publisher                  publisher;
            
            /// Ued to receive sensor rate update requests
            ros::ServiceServer              service;

            //! Send the sensor data over the messagng system
            /*!
              \param event the Timer event passed from the callback
            */
            void Broadcast(const ros::TimerEvent& event);

            //! Sample message data from the flight control system
            /*!
              \param event the Timer event passed from the callback
            */
            void Sample(const ros::TimerEvent& event);

            //! Set the data rate of the sensor
            /*!
              \param req the rate request message
              \param res the rate response message
              \return whether the rate updated was accepted
            */
            bool Configure(
                hal_sensor_orientation::Configure::Request  &req, 
                hal_sensor_orientation::Configure::Response &res);

        protected:

            //! Called by HAL to obtain measurement from the FCS
            /*!
              \param msg the message to be populated
            */
            virtual bool GetMeasurement(hal_sensor_orientation::Data& msg) = 0;

        public:

        	/// Constructor
        	Orientation();

        	/// Called when HAL is initialised
        	void OnInit();

        };
    }
}

#endif