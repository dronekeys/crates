#ifndef HAL_SENSOR_ALTIMETER_H
#define HAL_SENSOR_ALTIMETER_H

// Header libraries
#include <hal/HAL.h>

// Message libraries
#include <hal_sensor_altimeter/Data.h>

// For blank requests
#include <hal_sensor_altimeter/Configure.h>

namespace hal
{
    namespace sensor
    {
        class Altimeter : public hal::HAL
        {     
        private:

            /// Templated message type
            hal_sensor_altimeter::Data      message;

            /// Callback timer for status message updates
            ros::Timer                      timer;

            /// Used to broadcast Status message
            ros::Publisher                  publisher;
            
            /// Ued to receive sensor rate update requests
            ros::ServiceServer              service;

            //! Create a new Platform HAL
            /*!
              \param event the Timer event passed from the callback
            */
            void Broadcast(const ros::TimerEvent& event);

            // SENSOR-SPECIFIC STUFF

            //! Called by HAL to obtain a compass measurement
            /*!
              \param msg the message to be populated
              \return whether the measurement was obtained successfully
            */
            virtual bool GetMeasurement(hal_sensor_altimeter::Data& msg) = 0;

            //! Set the data rate of the sensor
            /*!
              \param req the rate request message
              \param res the rate response message
              \return whether the rate updated was accepted
            */
            bool Configure(
                hal_sensor_altimeter::Configure::Request  &req, 
                hal_sensor_altimeter::Configure::Response &res);

        public:

            /// Constructor
            Altimeter();

            /// Called when HAL is initialised
            void OnInit();

        };
    }
}

#endif