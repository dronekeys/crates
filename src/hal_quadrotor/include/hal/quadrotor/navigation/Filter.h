#ifndef HAL_MODEL_QUADROTOR_FILTER_H
#define HAL_MODEL_QUADROTOR_FILTER_H

// State and control messages
#include <hal_model_quadrotor/State.h>

// The sensors used in the data fusion
#include <hal/sensor/Altimeter.h>
#include <hal/sensor/Compass.h>
#include <hal/sensor/IMU.h>
#include <hal/sensor/GNSS.h>
#include <hal/sensor/Orientation.h>

namespace hal
{
    namespace quadrotor
    {
        // An abstract class for modelling noise
        class Filter
        {     
        protected:

			// BASIC FUNCTIONS ///////////////////////////////////////////////////

            //! Reset the filter
            virtual void Reset() = 0;

            //! Each filter must provide the ability to retrieve a state estimate
            /*!
                \param state the current platform state
            */
            virtual void GetState(hal_model_quadrotor::State &state) = 0;

            //! Process orientatin information
            /*!
                \param state the current platform state
            */
            virtual void SetState(const hal_model_quadrotor::State &state) = 0;


            // SENSOR INPUTS /////////////////////////////////////////////////////

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            virtual void RcvAltimeter(const hal_sensor_orientation::Data &msg) = 0;

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            virtual void RcvCompass(const hal_sensor_orientation::Data &msg) = 0;

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            virtual void RcvIMU(const hal_sensor_orientation::Data &msg) = 0;

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            virtual void RcvGNSS(const hal_sensor_orientation::Data &msg) = 0;

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            virtual void RcvOrientation(const hal_sensor_orientation::Data &msg) = 0;

        public:

            

        };
    }
}

#endif