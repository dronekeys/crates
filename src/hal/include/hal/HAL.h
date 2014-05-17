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
        hal::Status         message;

        /// Callback timer for status message updates
        ros::Timer          timer;

        /// Used to broadcast Status message
        ros::Publisher      publisher;

        /// Handle to a ROS node
        ros::NodeHandle*    rosNode;

        /// Do we get to delete the new node handle?
        bool                isManaged;    

        //! Create a new Platform HAL
        /*!
          \param event the Timer event passed from the callback
        */
        void Broadcast(const ros::TimerEvent& event);

    protected:

        //! Obtain a pointer to the ROS node handle
        /*!
          \return The current ros node handle
        */
        ros::NodeHandle* GetRosNodePtr();

        //! Initialise the HAL with an existing node handle (will not dealloc!)
        /*!
          \param nh the node handle
          \param manage should we dealloc the node handle memeory?
        */
        void Init(ros::NodeHandle* nh, bool manage = false);

        //! Create a new Platform HAL
        /*!
          \param event the Timer event passed from the callback
        */
        void Init(std::string name);

        /// Called by the intermediate class (sensor, controller, model, etc.)
        virtual void OnInit() = 0;

    public:    

        /// Create a new HAL
        HAL();

        /// Destroy a new HAL
        ~HAL();

        //! Set the state of the sensor
        /*!
          \param status a numeric status indicator
          \param message a string representation of the current status
        */
        void SetStatus(const HardwareStatus &status, const std::string &msg);
        
    };
}

#endif