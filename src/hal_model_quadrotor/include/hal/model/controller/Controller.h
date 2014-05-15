#ifndef HAL_MODEL_QUADROTOR_CONTROLLER_H
#define HAL_MODEL_QUADROTOR_CONTROLLER_H

// State and control messages
#include <hal_model_quadrotor/State.h>
#include <hal_model_quadrotor/Control.h>

namespace hal
{
    namespace model
    {
        // An abstract class for modelling noise
        class Controller
        {     

        protected:

            // Clamp a value to a given range
            static double limit(const double& val, const double& minval, const double& maxval)
            {
                if (val < minval) return minval;
                if (val > maxval) return maxval;
                return val;
            }

            // Rotation from body to navigation frame
            static void n2b(double rot[3], double vec[3])
            {
                double t[3], c[3], s[3];
                for (int i = 0; i < 3; i++)
                {
                    t[i] = vec[i];
                    c[i] = cos(rot[i]);
                    s[i] = sin(rot[i]);
                }
                vec[0] =                (c[1]*c[2])*t[0] +                (c[1]*s[2])*t[1] -      s[1]*t[2];
                vec[1] = (s[1]*s[0]*c[2]-s[2]*c[0])*t[0] + (s[1]*s[0]*s[2]+c[2]*c[0])*t[1] + c[1]*s[0]*t[2];
                vec[2] = (s[1]*c[0]*c[2]-s[2]*s[0])*t[0] + (s[1]*c[0]*s[2]+c[2]*s[0])*t[1] + c[1]*c[0]*t[2];
            }

        public:

            /// Constructor
            Controller();

            /// Destructor
            ~Controller();

            //! Each controller must provide the ability to generate control
            /*!
                \param state the current platform state
                \param dt the discrete time step
                \param control the output control from the controller
                \return if the state could be updated
            */
            virtual bool Update(const hal_model_quadrotor::State &state, 
                double dt, hal_model_quadrotor::Control &control) = 0;

            //! Goal reach implementations
            /*!
              \return Whether the goal has been reached
            */
            virtual bool HasGoalBeenReached() = 0;

            /// Reset the current state
            virtual void Reset() = 0;

        };
    }
}

#endif