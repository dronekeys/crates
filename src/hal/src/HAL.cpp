#include <hal/HAL.h>

using namespace hal;

// Obtain a pointer to the ROS node
ros::NodeHandle* HAL::GetRosNodePtr()
{
    if (!rosNode)
        ROS_FATAL("A ROS node has not been initialized");
    return rosNode;
}

// Initialise the HAL with an existing node handle
void HAL::Init(ros::NodeHandle* nh, bool manage)
{
    // First, make sure we cleanly shut down the last handle
    if (rosNode && isManaged)
        delete rosNode;

    // Copy over the new handle, and store whether memory shoudl be managed
    rosNode   = nh;
    isManaged = manage;

    // Advertise this message on the ROS backbone
    publisher = rosNode->advertise<hal::Status>("Status", DEFAULT_QUEUE_LENGTH);

    // Set the name of this device
    message.start = ros::Time::now();

    // Create a timer to broadcast the data
    timer = rosNode->createTimer(
        ros::Duration(1.0/DEFAULT_STATUS_RATE),     // Callback rate
        &HAL::Broadcast,                            // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );

    // Call skeleton type init
    OnInit();
}

// Initialise the HAL with a name, so allocate!
void HAL::Init(std::string name)
{
    HAL::Init(new ros::NodeHandle(name), true);
}

// Constructor
HAL::HAL() : isManaged(false)
{
    if (!ros::isInitialized())
        ROS_FATAL("ROS has not been initialized");
}

// Destructor
HAL::~HAL()
{
    // If we alloc'd the ROS node, clean up
    if (rosNode && isManaged)
    {
        // Shut down
        rosNode->shutdown();

        // Delete
        delete rosNode;
    }
}

// Set the status of this HAL
void HAL::SetStatus(const hal::HardwareStatus &status, const std::string &msg)
{                  
    message.status  = status;
    message.message = msg;
}

// Broadcast the status
void HAL::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}