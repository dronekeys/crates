#ifndef HAL_SENSOR_ALTIMETER_H
#define HAL_SENSOR_ALTIMETER_H

// Header libraries
#include <hal/HAL.h>

// Message libraries
#include <hal_sensor_altimeter/Data.h>

namespace hal
{
    namespace sensor
    {
        class Altimeter : public hal::HAL
        {     
        private:

            /// Templated message type
            hal_sensor_altimeter::Data message;

            /// Callback timer for status message updates
            ros::Timer timer;

            /// Used to broadcast Status message
            ros::Publisher publisher;

            //! Create a new Platform HAL
            /*!
              \param event the Timer event passed from the callback
            */
            void Broadcast(const ros::TimerEvent& event);

        public:

            /// Constructor
            Altimeter();

            /// Called when HAL is initialised
            void OnInit();

        };
    }
}

#endif