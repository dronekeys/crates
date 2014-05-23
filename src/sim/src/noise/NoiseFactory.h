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

    // Define the gaussian types
    typedef enum 
    {
        WHITE,    /* Zero-mean Gaussian noise             */
        ORNSTEIN, /* Ornstein-Uhlenbeck process nouse     */
        DRYDEN,   /* Dryden process noise                 */
        UNKNOWN
    } 
    NoiseType;

    // Convenience
    typedef std::map<std::string,NoiseType>     TypeVec;
    typedef std::vector<Noise*>                 ProcessVec;

    // An abstract class for modelling noise
    class NoiseFactory
    {     

    private:

        // The name of this model
        static std::string              name;

        // Requirements for listening for Gazbeo messages
        static event::ConnectionPtr     conPtr;
        static transport::NodePtr       nodePtr;
        static transport::SubscriberPtr subPtr;

        /// A list of noise processes
        static TypeVec                  types;

        /// A list of noise processes
        static ProcessVec               processes;

        /// Find a distribution type
        static NoiseType Lookup(std::string &name);

    public:    

        //! Turn noise on or off
        /*!
            \param a pointer to the world
        */
        static void Receive(NoisePtr& noise);

        //! Initialize the noise factory
        /*!
            \param a pointer to the world
            \param a pointer to the name of this node
        */
        static void Init(physics::WorldPtr worldPtr, std::string inname);

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