// Include statistical models
#include <gazebo/math/gzmath.hh>

#ifndef SIM_WIENER_H
#define SIM_WIENER_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class Wiener : public Noise
    {     

    private:

        // The distribution
        double _bias, _white;

        // The current value
        double current;

    public:    

        //! Create a new Wiener noise process object
        /*!
            \param biase variance (random walk)
            \param white variance (white component)
            \return a new Gaussian object
        */
        Wiener(double bias = 1.0, double white = 0.0);

        //! Sample the distribution
        /*!
            \param dt the discrete time step
            \return a sample
        */
        double Sample(double dt = 0);

    };
}

#endif