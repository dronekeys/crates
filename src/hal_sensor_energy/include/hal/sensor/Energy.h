#ifndef HAL_SENSOR_ENERGY_H
#define HAL_SENSOR_ENERGY_H

// Header libraries
#include <hal/HAL.h>

// Message libraries
#include <hal_sensor_energy/Data.h>

namespace hal
{
    namespace sensor
    {
        class Energy : public hal::HAL
        {     
        private:

            /// Templated message type
            hal_sensor_energy::Data message;

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
            virtual bool GetMeasurement(hal_sensor_energy::Data& msg) = 0;

        public:

            /// Constructor
            Energy();

            /// Called when HAL is initialised
            void OnInit();

        };
    }
}

#endif