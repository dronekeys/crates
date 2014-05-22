#ifndef SIM_DRYDEN_H
#define SIM_DRYDEN_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class Dryden : public Noise
    {     

    private:

        // The current value
        math::Vector3 turbulence, s, l;

        //! Sample a 3D vector from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        gazebo::Vector3 Sample(double dt = 0);

        //! Sample a scalar from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        double Sample(double dt = 0);

    public:    

        //! Sample a scalar from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        Dryden(physics::linkPtr link);

        // Reset using the given list of arguments
        Reset();

    };
}

#endif