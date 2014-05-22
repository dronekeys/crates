#ifndef SIM_ORNSTEIN_H
#define SIM_ORNSTEIN_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class Ornstein : public Noise
    {     

    private:

       

    public:

        // Configure using the given SDF
        Ornstein(std::string name, sdf::ElementPtr root);

        // Reset using the given list of arguments
        Reset();

        //! Sample a 3D vector from the random distribution
        /*!
            \param link the model link
            \param dt the discrete time step
        */
        void Sample(double dt = 0);

    };
}

#endif