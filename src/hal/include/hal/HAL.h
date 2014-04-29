#ifndef HAL_H
#define HAL_H

// Standard library includes
#include <string>
#include <ros/ros.h>

// Messages used by this HAL
#include <hal/Status.h>

#define DEFAULT_QUEUE_LENGTH 10
#define DEFAULT_STATUS_RATE   1

namespace hal
{
    //! An enum.
    /*! More detailed enum description. */
    enum HardwareStatus
    {
        STATUS_NORMAL = 0,  /*!< Enum value Normal. */ 
        STATUS_WARNING,     /*!< Enum value Warning */ 
        STATUS_ERROR,       /*!< Enum value Error */ 
        STATUS_FATAL        /*!< Enum value TFatal */ 
    };

    class HAL
    {     

    private:

        /// Data storage for the status message
        hal::Status message;

        /// Callback timer for status message updates
        ros::Timer timer;

        /// Used to broadcast Status message
        ros::Publisher publisher;

        //! Create a new Platform HAL
        /*!
          \param event the Timer event passed from the callback
        */
        void Broadcast(const ros::TimerEvent& event);

    protected:

        /// Handle to a ROS node, shared by all children
        ros::NodeHandle rosNode;

    public:    

        //! Create a new HAL
        /*!
          \param node the ROS node to which the entity will be attached
          \param name the name of the entity
        */
        HAL(ros::NodeHandle& node, const char* name);

        //! Set the state of the sensor
        /*!
          \param status a numeric status indicator
          \param message a string representation of the current status
        */
        void SetStatus(const HardwareStatus &status, const std::string &msg);
        
    };
}

#endif