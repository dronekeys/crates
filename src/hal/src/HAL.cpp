#include <hal/HAL.h>

using namespace hal;

HAL::HAL(const char *name) : rosNode(ros::NodeHandle(name))
{
    // Maske sure that ROS actually started, or there will be some issues...
    if (!ros::isInitialized())
        ROS_FATAL("A ROS node has not been initialized");

    // Advertise this message on the ROS backbone
    publisher = rosNode.advertise<hal::Status>(name, DEFAULT_QUEUE_LENGTH);

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
