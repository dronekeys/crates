#ifndef UAS_HAL_CONTROLLER_HOVER_H
#define UAS_HAL_CONTROLLER_HOVER_H

// Hover is really just an implementation of 
#include <uas_hal/controller/Waypoint.h>

namespace uas_hal
{
    class Hover : public Waypoint
    {

    public:

        // Connstructor
        Hover();

        // Set the hover goal using the current state
        void SetGoal(State *state);

        // Pass the state and discrete time step and receive a control back
        void Update(State *state, double dt, Control *ctl);

        // Reset the controller
        void Reset();
    };
}

#endif