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

// Maximum variables and parameters
#define MAX_VARS 3
#define MAX_PARS 3

namespace gazebo
{
    // An abstract class for modelling noise
    class Noise
    {     
    private:

        // This flag decide whether random numbers are enabled
        bool enabled;

    protected:

        // Variables and parameters specific to this distribution
        double vars[MAX_VARS];
        double pars[MAX_PARS];

    public:

        // Constructor
        Noise();

        /// Destructor
        ~Noise();

        /// Reset the random stream
        virtual void Reset() = 0;

        //! Sample a 3D vector from the random distribution
        /*!
            \param link the model link
            \param t sample time
        */
        virtual void Sample(double t) = 0;

        //! Allow up to two double parameters to be configured online
        /*!
            \param p1 first parameter
            \param p2 second parameter
        */
        void Configure(int idx, double val);

        //! Sample a scalar from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        double Get(int idx = 0);

        //! Enable and disable this noise stream,
        /*!
            \param enabled whether to enable the stream
        */
        void Toggle(bool enabled);

        //! Sample the random distribution and by default return the first value
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        double DrawScalar(double t = 0);

        //! Sample the random distribution and by default return the first three values
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        math::Vector3 DrawVector(double t = 0);

    };
}

#endif
