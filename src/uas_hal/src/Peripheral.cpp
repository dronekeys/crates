#include <uas_hal/Peripheral.h>

using namespace uas_hal;

// Send the data to the ROS messaging service, for collection by others
void Peripheral::Broadcast(const ros::TimerEvent& event)
{
    publisher.publish(message);
}

// Constructor
Peripheral::Peripheral(ros::NodeHandle &h, const char *name) : node(h)
{
    // Advertise this message
    publisher = node.advertise<T>(name,DEFAULT_QUEUE_LENGTH);

    // Configure mesaging
    Reset(DEFAULT_POST_RATE);
}

// Configure broadcast at a particular rate
void Peripheral::Reset(const double &rate)
{
    // Create a timer callback
    timer = node.createTimer(
        ros::Duration(1.0/rate), &Peripheral::Broadcast, this
    );
}

// Manually set the data in this message (usually called by FCS / simulation)
void Peripheral::Set(const T &msg)
{
    // Set the data
    message = msg;

    // Make the parent class aware of the new data
    Receive(message);
}
