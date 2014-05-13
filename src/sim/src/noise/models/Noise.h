#ifndef SIM_NOISE_H
#define SIM_NOISE_H

// System includes
#include <map>
#include <string>

// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/math/gzmath.hh>

namespace gazebo
{
    // An abstract class for modelling noise
    class Noise
    {     

    public:

        //! Sample from the random distribution
        /*!
            \param dt the discrete time step
            \return the sampled variable
        */
        virtual double Sample(double dt = 0) = 0;

    };
}

#endif