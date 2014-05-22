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
#define MAX_VARS 6
#define MAX_PARS 3

namespace gazebo
{
    // An abstract class for modelling noise
    class Noise
    {     
    private:

        // This flag decide whether random numbers are enabled
        bool            enabled;

        // Name of the noise stream
        std::string     name;

    protected:

        // Variables and parameters specific to this distribution
        double vars[MAX_VARS];
        double pars[MAX_PARS];

    public:

        /// Destructor
        virtual ~Noise() {};

        /// Reset the random stream
        virtual void Reset() = 0;

        //! Sample the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        virtual void Sample(double dt);

        //! Allow up to two double parameters to be configured online
        /*!
            \param p1 first parameter
            \param p2 second parameter
        */
        void Config(int idx p1, double val);

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
        
        //! Get the name of the noise stream
        /*!
            \returns the name of the noise stream
        */
        std::string GetName();

        //! Sample the random distribution and by default return the first value
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        double Draw(double dt = 0) = 0;

        //! Sample the random distribution and by default return the first three values
        /*!
            \param link the model link
            \param dt the discrete time step
            \return the sampled variable
        */
        std::Vector3 Draw(double dt = 0) = 0;

    };
}

#endif
