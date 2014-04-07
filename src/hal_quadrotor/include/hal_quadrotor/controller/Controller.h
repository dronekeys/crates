#ifndef HAL_QUADROTOR_CONTROLLER_H
#define HAL_QUADROTOR_CONTROLLER_H

// System includes
#include <cmath>
#include <map>

// Basic ros functionality
#include <ros/ros.h>

// Messages
#include <hal_quadrotor/State.h>
#include <hal_quadrotor/Control.h>

namespace hal_quadrotor
{
    // Basic controller types
    enum ControllerType
    {
        EmergencyType,
        IdleType,
        TakeoffType,
        LandType,
        HoverType,
        ActionType
    };

    // Controller must have a service type attached to it
    class Controller
    {

    protected: 

        // USED BY THE CHILD CLASS //////////////////////////////////////////////////

        ros::ServiceServer  service;
        Control             control;
        ControllerType      type;

        // USED TO KEEP TRACK OF THE CURRENT CONTROLLER /////////////////////////////
        
        static Controller* current;

        // The important controller types to which one can switch
        static std::map<ControllerType,Controller*> types;

        // Flight logic
        static bool Logic(ControllerType next)
        {
            return true;
        }

        // Convert navigation frame to body frame
        static void n2b(double rot[3], double vec[3])
        {
            // Precompute for speed
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

        // Clamp a value to exist between two intervals
        static double limit(double val, double minval, double maxval)
        {
            if (val < minval) return minval;
            if (val > maxval) return maxval;
            return val;
        }

    public:

        // Default contructor
        Controller(const ControllerType &t) : type(t) 
        {
            // IMPORTANT: last constructed controller is 'current' (ORDER MATTERS!!!)
            current = this;

            // Store the core types for easy reference
            if (t!=ActionType)
                types[t] = this;
        }

        // IMPLEMENTED BY CHILDREN //////////////////////////////////////////////////////

        // Get new control from current state and time step
        virtual Control Update(const State &state, const double &dt) = 0;

        // Reset the current state
        virtual void Reset() = 0;

        // CALLED FROM HAL //////////////////////////////////////////////////////////////

        // Switch to one of the base control types
        static bool Switch(ControllerType &t)
        {
            if (types.find(t) != types.end())
            {
                current = types[t];
                return true;
            }
            return false;
        }

        // Get the control for the current type
        Control Get(const State &state, const double &dt)
        {
            return current->Update(state, dt);  
        }
    };
}

#endif