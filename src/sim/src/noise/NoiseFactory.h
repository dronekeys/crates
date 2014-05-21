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
	// Define the gaussian types
	typedef enum 
	{
		WHITE,    /* Zeroi-mean Gaussian noise            */
		WIENER,   /* Weiner process noise                 */
		ORNSTEIN, /* Ornstein-Uhlenbeck process nouse     */
		UNIFORM,  /* Uniform noise                        */
        UNKNOWN
	} 
	NoiseType;

    // Environment messages
    typedef const boost::shared_ptr<const msgs::Noise> NoisePtr;

    // An abstract class for modelling noise
    class NoiseFactory
    {     

    private:

    	// A list of noise types
    	static std::map<std::string,NoiseType> types;

        /// A list of noise processes
        static std::map<std::string,Noise*> processes;

    protected:

        //! Obtain a noise type enumeration from a string
        /*!
            \param name the name of the noise type
            \return an enumeration of the noise type
        */
        static NoiseType Lookup(std::string &name);

    public:    

        /// Constructor
        NoiseFactory();

        /// Destructor
        ~NoiseFactory();

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