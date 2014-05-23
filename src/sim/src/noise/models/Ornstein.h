#ifndef SIM_ORNSTEIN_H
#define SIM_ORNSTEIN_H

// For base noise type
#include "Noise.h"


#define IDX_BETA  0
#define IDX_SIGMA 1

namespace gazebo
{
    // An abstract class for modelling noise
    class Ornstein : public Noise
    {     

    private:

        // Time lag
        double tim;

        // Configuration for each noise process
        double _beta;
        double _sigma;

    public:

        // Configure using the given SDF
        Ornstein(double beta, double sigma);

        // Destructor
        ~Ornstein(); 

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