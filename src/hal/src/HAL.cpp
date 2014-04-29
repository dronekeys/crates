#include <hal/HAL.h>

using namespace hal;

HAL::HAL(ros::NodeHandle& node, const char* name) : rosNode(node)
{
    // Maske sure that ROS actually started, or there will be some issues...
    if (!ros::isInitialized())
        ROS_FATAL("A ROS node has not been initialized");

    // Set the name of this device
    message.name  = name;
    message.start = ros::Time::now(); 

    // Advertise this message on the ROS backbone
    publisher = rosNode.advertise<hal::Status>("Status", DEFAULT_QUEUE_LENGTH);

    // Create a timer to broadcast the data
    timer = rosNode.createTimer(
        ros::Duration(1.0/DEFAULT_STATUS_RATE),  // Callback rate
        &HAL::Broadcast,                     // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

void HAL::SetStatus(const hal::HardwareStatus &status, const std::string &msg)
{                  
    message.status  = status;
    message.message = msg;
}

void HAL::Broadcast(const ros::TimerEvent& event)
{                  
    publisher.publish(message);
}