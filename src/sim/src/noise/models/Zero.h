#ifndef SIM_ZERO_H
#define SIM_ZERO_H

// For base noise type
#include "Noise.h"

namespace gazebo
{
    // A simple class to draw zero error values, in case another
    // class is not specified during configuration
    class Zero : public Noise
    {     
    private:

        

    public:

        // Configure using the given SDF
        Zero();

        // Destructor
        ~Zero(); 

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