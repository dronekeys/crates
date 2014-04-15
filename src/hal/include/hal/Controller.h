#ifndef HAL_CONTROLLER_H
#define HAL_CONTROLLER_H

#include <hal/HAL.h>

namespace hal
{
    namespace controller
    {
        template <class STATE, class CONTROL, class REQUEST, class RESPONSE>
        class Controller : public hal::HAL
        {

        protected: 

            /// Used to receive a new goal
            ros::ServiceServer service;

            //! Receive a control message from the ROS backbone
            /*!
              \param req the goal request
              \param res the goal response
              \return whether the control was accepted
            */
            virtual bool Receive(REQUEST &req, RESPONSE &res) = 0;

            //! Project a body-frame point to a navigation-frame point
            /*!
              \param rot the 3x3 rotation matrix
              \param vec the sample point
            */
            static void n2b(double rot[3], double vec[3]);

            //! Clamp a value to within a range
            /*!
              \param val the sample value
              \param minval the lower bound
              \param maxval the upper bound
              \return the resultant value
            */
            static double limit(const double& val, const double& minval, const double& maxval);

        public:

            //! Create a controller object
            /*!
              \param name name of this controller
              \return new object
            */
            Controller(const char* name);

            //! Clamp a value to within a range
            /*!
              \param state the current state value
              \param dt the discrete time step
              \return the resultant control
            */
            virtual CONTROL Update(const STATE &state, const double &dt) = 0;

            /// Reset the current state
            virtual void Reset() = 0;
        };

    }
}

#endif