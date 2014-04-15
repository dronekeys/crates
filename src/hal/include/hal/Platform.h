#ifndef HAL_PLATFORM_H
#define HAL_PLATFORM_H

#include <hal/HAL.h>

#define DEFAULT_SENSOR_RATE   1.0

namespace hal
{
    namespace platform
    {
        //!  A base class inherited by all platforms
        /*!
          This class provides thbasic platform functionality.
        */
        class Platform : public hal::HAL
        {    

        public:

            //! Create a new Platform HAL
            /*!
              \param name the name of the platform
            */
            Platform(const char *name);

        };

    }
}

#endif