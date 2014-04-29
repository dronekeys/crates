#ifndef HAL_MODEL_H
#define HAL_MODEL_H

// Parent class for this HAL
#include <hal/HAL.h>

#define DEFAULT_SENSOR_RATE   1.0

namespace hal
{
    namespace model
    {
        //!  A base class inherited by all platforms
        /*!
          This class provides thbasic platform functionality.
        */
        class Model : public hal::HAL
        {    

        public:

            //! Create a new Platform HAL
            /*!
              \param name the name of the platform
            */
            Model(ros::NodeHandle& node, const char *name);

        };

    }
}

#endif