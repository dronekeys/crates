#ifndef HAL_SENSOR_H
#define HAL_SENSOR_H

#include <string>
#include <ros/ros.h>
#include <hal/HAL.h>

#include <hal/SetRate.h>

#define DEFAULT_SENSOR_RATE   1.0

namespace hal
{
    namespace sensor
    {
        //!  A base class inherited by all sensors
        /*!
          This class provides the basic ability to buffer a message and broadcast
          it at a given rate. The rate can be adapted.
        */
        template <class DataMsgClass>
        class Sensor : hal::HAL
        {    

        private:
            
            /// Templated message type
            DataMsgClass message;

            /// Callback timer for status message updates
            ros::Timer timer;

            /// Used to broadcast Status message
            ros::Publisher publisher;
            
            /// Ued to receive sensor rate update requests
            ros::ServiceServer  service;

            //! Create a new Platform HAL
            /*!
              \param event the Timer event passed from the callback
            */
            void Broadcast(const ros::TimerEvent& event);

            //! Set the data rate of the sensor
            /*!
              \param req the rate request message
              \param res the rate response message
            */
            bool SetRate(hal::SetRate::Request &req, hal::SetRate::Response &res);

        public:

            //! Create a new Sensor HAL
            /*!
              \param name the name of the sensor
            */
            Sensor(const char *name);
        };

    }
}

#endif