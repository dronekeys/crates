#ifndef HAL_QUADROTOR_NAVIGATION_H
#define HAL_QUADROTOR_NAVIGATION_H

// Local library incldues
#include <hal_quadrotor/Altitude.h>
#include <hal_quadrotor/Inertial.h>
#include <hal_quadrotor/Position.h>
#include <hal_quadrotor/Magnetic.h>
#include <hal_quadrotor/Orientation.h>
#include <hal_quadrotor/State.h>

namespace hal_quadrotor
{
    // This is a simple filter
    class Navigation
    {

    private:

        // State estimate
        State           state;
        Orientation     orientation;

    public:

        // Constructor
        Navigation();

        // Feed measurents to the filter
        void Measurement(const Altitude &msg);
        void Measurement(const Inertial &msg);
        void Measurement(const Position &msg);
        void Measurement(const Magnetic &msg);
        void Measurement(const Orientation &msg);
        void Measurement(const State &msg);

        // Get the latest state estimate
        State GetState();
        
        // Get the latest state estimate
        Orientation GetOrientation();
    };
}

#endif