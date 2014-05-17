#ifndef SIM_UNIFORM_H
#define SIM_UNIFORM_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class Uniform : public Noise
    {     

    private:

        // The distribution
        double _lower, _upper;

    public:    

        //! Create a new Gaussian process object
        /*!
            \param mu mean
            \param sigma variance
            \return a new Gaussian object
        */
        Uniform(double lower = 0.0, double upper = 1.0);

        //! Sample the distribution
        /*!
            \param dt the discrete time step
            \return a sample
        */
        double Sample(double dt = 0);

    };
}

#endif