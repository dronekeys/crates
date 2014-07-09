#ifndef HAL_SENSOR_COMPASS_H
#define HAL_SENSOR_COMPASS_H

// Header libraries
#include <hal/HAL.h>

// Message libraries
#include <hal_sensor_compass/Data.h>

// For blank requests
#include <hal_sensor_compass/Configure.h>

namespace hal
{
    namespace sensor
    {
        class Compass : public hal::HAL
        {
        private:

            /// Templated message type
            hal_sensor_compass::Data        message;

            /// Callback timer for status message updates
            ros::Timer                      timerSamp, timerSend;

            /// Used to broadcast Status message
            ros::Publisher                  publisher;

            /// Used to receive sensor rate update requests
            ros::ServiceServer              service;

            /// Used to broadcast Data message
            ros::ServiceServer              srvData;

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
                hal_sensor_compass::Configure::Request  &req,
                hal_sensor_compass::Configure::Response &res);

        protected:

            //! Called by HAL to obtain measurement from the FCS
            /*!
              \param msg the message to be populated
            */
            virtual bool GetMeasurement(hal_sensor_compass::Data& msg) = 0;

        public:

            /// Constructor
            Compass();

            /// Called when HAL is initialised
            void OnInit();

        };
    }
}

#endif
