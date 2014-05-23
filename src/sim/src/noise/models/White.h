#ifndef SIM_WHITE_H
#define SIM_WHITE_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // An abstract class for modelling noise
    class White : public Noise
    {     

    private:

        // Configuration for each noise process
        double _sigma;

    public:
        
        // Configure using the given SDF
        White(double sigma);

        // Destructor
        ~White(); 

        // Reset using the given list of arguments
        void Reset();

        //! Sample a 3D vector from the random distribution
        /*!
            \param link the model link
            \param t sample time
        */
        void Sample(double t = 0);

    };
}

#endif