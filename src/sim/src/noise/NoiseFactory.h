#ifndef SIM_NOISEFACTORY_H
#define SIM_NOISEFACTORY_H

// System includes
#include <map>
#include <string>

// Gazebo includes
#include "models/Ornstein.h"
#include "models/Uniform.h"
#include "models/White.h"
#include "models/Wiener.h"

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
    typedef std::map<std::string,NoiseType> TypeVec;
    typedef std::map<std::string,Noise*>    ProcessVec;

    // An abstract class for modelling noise
    class NoiseFactory
    {     

    private:

        // Requirements for listening for Gazbeo messages
        static event::ConnectionPtr     conPtr;
        static transport::NodePtr       nodePtr;
        static transport::SubscriberPtr subPtr;

        /// A list of noise processes
        static TypeVec      types;

        /// A list of noise processes
        static ProcessVec   processes;

    public:    

        //! Turn noise on or off
        /*!
            \param a pointer to the world
        */
        static void Receive(NoisePtr noise);

        //! Initialize the noise factory
        /*!
            \param a pointer to the world
        */
        static void Init(physics::ModelPtr model);

        //! Obtain a pointer to the ROS node handle
        /*!
			\param root SDF defining the noise type
          	\return whether the random process could be created
        */
        static bool Create(sdf::ElementPtr root);

        //! Obtain a pointer to the ROS node handle
        /*!
			\param name the name of the random stream
			\param dt the discrete time step
          	\return a sampled value
        */
        static double Sample(std::string& name, double dt = 0.0);

    };
}

#endif