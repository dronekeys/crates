#ifndef UAS_HAL_CONTROL_H
#define UAS_HAL_CONTROL_H

// Binding capabilities
#include <boost/bind.hpp>

// Actual controller implementations
#include <uas_hal/controller/AnglesHeight.h>
#include <uas_hal/controller/Emergency.h>
#include <uas_hal/controller/Hover.h>
#include <uas_hal/controller/Idle.h>
#include <uas_hal/controller/Land.h>
#include <uas_hal/controller/Takeoff.h>
#include <uas_hal/controller/Velocity.h>
#include <uas_hal/controller/VelocityHeight.h>
#include <uas_hal/controller/Waypoint.h>

// Exploit resuable action interface
#include <actionlib/server/simple_action_server.h>

// Default peripheral data queue length before discard
#define DEFAULT_QUEUE_LENGTH      10
#define DEFAULT_PROGRESS_RATE    1.0
#define DEFAULT_UPDATE_RATE     50.0
#define DEFAULT_TIMEOUT_RATE     1.0

namespace uas_hal
{
    // T is the message type sent by the peripheral
    template <class C>  // Controller
    class Control
    {

    private:

        // STATIC MEMBERS //////////////////////////////////////////////////////////

        static Control* current;

        // INSTANCE MEMBERS ////////////////////////////////////////////////////////

        C controller;

        // CALLBACKS ///////////////////////////////////////////////////////////////

        void Switch()
        {
            // Preempt the goal which is currently running
            current->Preempt();

            // Set the current
            current = this;
       }

    protected:

        // Convert a navigation frame vector
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

            // Brute-force calculation of the following matrix time vec
            //     [          cy*cz,          cy*sz,            -sy]
            //     [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx] 
            //     [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
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

        // INSTANCE MEMBERS ////////////////////////////////////////////////////////
        
        Control(ros::NodeHandle &h, const char *name) : 
            controller(h, name, boost::bind(&Control::Switch,this,_1))
        {
        
        }

        // Stop persuing this goal immediately...
        void Preempt()
        {
            controller.Preempt();
        }

        // STATIC MEMBERS //////////////////////////////////////////////////////////
        
        // Forward the request to the current controller in use
        static const MsgControl& Update(const MsgState& state, const double& dt)
        {        
            return current->Update(state,dt);
        }

    };
}

#endif