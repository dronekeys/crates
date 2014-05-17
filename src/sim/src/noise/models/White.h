#ifndef SIM_WHITE_H
#define SIM_WHITE_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class White : public Noise
    {     

    private:

        // The distribution
        double _white;

    public:    

        //! Create a new Gaussian process object
        /*!
            \param mu mean
            \param sigma variance
            \return a new Gaussian object
        */
        White(double white = 1.0);

        //! Sample the distribution
        /*!
            \param dt the discrete time step
            \return a sample
        */
        double Sample(double dt = 0);
    };
}

#endif