#ifndef HAL_H
#define HAL_H

#include <string>
#include <ros/ros.h>
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
        static ros::NodeHandle rosNode;

    public:    


        //! Create a new HAL
        /*!
          \param name the name of the entity
        */
        HAL(const char* name);

        //! Set the state of the sensor
        /*!
          \param status a numeric status indicator
          \param message a string representation of the current status
        */
        void SetStatus(const HardwareStatus &status, const std::string &msg);

        //! Initialise the to use a particular ROS node
        /*!
          \param node the ROS node to which the entity will be attached
        */
        void Init(ros::NodeHandle& node);

        //! Initialise the to create a new ROS node
        /*!
          \param name the name of the new ROS node
        */
        void Init(const char* name);
        
    };
}

#endif