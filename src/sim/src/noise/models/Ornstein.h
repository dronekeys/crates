#ifndef SIM_ORNSTEIN_H
#define SIM_ORNSTEIN_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class Ornstein : public Noise
    {     

    private:

        // The distribution
        double _theta, _bias, _white;

        // The current value
        double current;

    public:    

        //! Create a new Ornstein process object
        /*!
            \param mu mean value
            \param sigma strength of perturbation
            \param theta decay rate
            \return a new Gaussian object
        */
        Ornstein(double theta = 1.0, double bias = 1.0, double white = 1.0);

        //! Sample the distribution
        /*!
            \param dt the discrete time step
            \return a sample
        */
        double Sample(double dt = 0);

    };
}

#endif