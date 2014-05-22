#ifndef SIM_NOISE_H
#define SIM_NOISE_H

// System includes
#include <map>
#include <string>
#include <cstdarg>

// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/math/gzmath.hh>

// We need to know when noise is enabled and disabled
#include "noise.pb.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class Noise
    {     
    private:

        // This flag willdecide whether random numbers are enabled
        bool enabled;

    protected:

        // A vector of distribution parameters
        std::vector<double> params;

        // Child must implement a 3D vector stats function
        virtual gazebo::Vector3 Sample(physics::linkPtr link, double dt) = 0;

        // Child must implement a scalar stats function
        virtual double Sample(physics::linkPtr link, double dt) = 0;

    public:

        /// Reset the random stream
        virtual void Reset() = 0;

        /// Destructor
        virtual ~Noise() {}

        //! Configure the distribution
        /*!
            \param num number of arguments
        */
        void Configure(int num, ...);

        //! Sample a 3D vector from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        gazebo::Vector3 DrawVector(physics::linkPtr link, double dt = 0);

        //! Sample a scalar from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        double DrawScalar(physics::linkPtr link, double dt = 0);
    };
}

#endif
