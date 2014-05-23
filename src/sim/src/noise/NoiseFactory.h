#ifndef SIM_NOISEFACTORY_H
#define SIM_NOISEFACTORY_H

// Required for gazebo math, messages and physics
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// System includes
#include <map>
#include <vector>
#include <string>

// Gazebo includes
#include "models/Dryden.h"
#include "models/Ornstein.h"
#include "models/White.h"
#include "models/Zero.h"

namespace gazebo
{
    // Environment messages
    typedef const boost::shared_ptr<const msgs::Noise> NoisePtr;

    // Vector of noise processes
    typedef std::vector<Noise*> ProcessVec;

    // An abstract class for modelling noise
    class NoiseFactory
    {     

    private:

        /// A list of noise processes
        static ProcessVec processes;

    public:    

        //! Turn noise on or off
        /*!
            \param a pointer to the world
        */
        static void Toggle(bool enabled);

        //! Destroy the noise factory
        static void Init();

        //! Destroy the noise factory
        static void Destroy();

        //! Obtain a pointer to the ROS node handle
        /*!
			\param root SDF defining the noise type
          	\return whether the random process could be created
        */
        static Noise* Create(sdf::ElementPtr root);
    };
}

#endif