#ifndef HAL_MODEL_QUADROTOR_NAIVEESTIMATOR_H
#define HAL_MODEL_QUADROTOR_NAIVEESTIMATOR_H

// Base controller type
#include <hal/quadrotor/navigation/Filter.h>

namespace hal
{
    namespace quadrotor
    {
        //! A quadrotor Emergency controller
        /*!
          A more elaborate class description.
        */
        class NaiveEstimator : public Filter
        {

        private:

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

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            void RcvAltimeter(const hal_sensor_altimeter::Data &msg
            {
              state.z = msg.z;
            }

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            void RcvCompass(const hal_sensor_compass::Data &msg)
            {
              // Do nothing
            }

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            void RcvIMU(const hal_sensor_imu::Data &msg)
            {
              state.p = msg.p;
              state.q = msg.q;
              state.r = msg.r;
            }

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            void RcvGNSS(const hal_sensor_gnss::Data &msg)
            {
              state.x = msg.x;
              state.y = msg.y;
              state.z = msg.z;
              state.u = msg.u;
              state.v = msg.v;
              state.w = msg.w;
            }

            //! Process orientatin information
            /*!
                \param msg the orientation data
            */
            void RcvOrientation(const hal_sensor_orientation::Data &msg)
            {
              state.roll  = msg.roll;
              state.pitch = msg.pitch;
              state.yaw   = msg.yaw;
            }
        };
    }
}

#endif