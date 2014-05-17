#ifndef HAL_SENSOR_IMU_H
#define HAL_SENSOR_IMU_H

// Header libraries
#include <hal/HAL.h>

// Message libraries
#include <hal_sensor_imu/Data.h>

// For blank requests
#include <hal_sensor_imu/SetRate.h>

namespace hal
{
    namespace sensor
    {
        class IMU : public hal::HAL
        {     
        private:

            /// Templated message type
            hal_sensor_imu::Data message;

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

            //! Called by HAL to obtain a compass measurement
            /*!
              \param msg the message to be populated
              \return whether the measurement was obtained successfully
            */
            virtual bool GetMeasurement(hal_sensor_imu::Data& msg) = 0;

            //! Set the data rate of the sensor
            /*!
              \param req the rate request message
              \param res the rate response message
              \return whether the rate updated was accepted
            */
            bool SetRate(hal_sensor_imu::SetRate::Request &req, hal_sensor_imu::SetRate::Response &res);

        public:

        	/// Constructor
        	IMU();

        	/// Called when HAL is initialised
        	void OnInit();

        };
    }
}

#endif