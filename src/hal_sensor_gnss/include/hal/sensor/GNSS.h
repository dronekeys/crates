#ifndef HAL_SENSOR_GNSS_H
#define HAL_SENSOR_GNSS_H

// Header libraries
#include <hal/HAL.h>

// Message libraries
#include <hal_sensor_gnss/Data.h>

namespace hal
{
    namespace sensor
    {
        class GNSS : public hal::HAL
        {     
        private:

            /// Templated message type
            hal_sensor_gnss::Data message;

            /// Callback timer for status message updates
            ros::Timer timer;

            /// Used to broadcast Status message
            ros::Publisher publisher;

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
            virtual bool GetMeasurement(hal_sensor_gnss::Data& msg) = 0;

        public:

            /// Constructor
            GNSS();

            /// Called when HAL is initialised
            void OnInit();

        };
    }
}

#endif