#ifndef SIM_DRYDEN_H
#define SIM_DRYDEN_H

// For base noise type
#include "Noise.h"

#define DRYDEN_PARS_AIRSPEED  0
#define DRYDEN_PARS_ALTITUDE  1
#define DRYDEN_PARS_WNDSPEED  2

namespace gazebo
{
    // An abstract class for modelling noise
    class Dryden : public Noise
    {     

    private:

        // Intermediate values
        math::Vector3 s, l;

    public:

        // Configure using the given SDF
        Dryden();

        // Destructor
        ~Dryden(); 

        // Reset using the given list of arguments
        void Reset();

        //! Sample a 3D vector from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
        */
        void Sample(double dt = 0);

    };
}

#endif