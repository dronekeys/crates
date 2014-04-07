// Standard libraries
#include <hal_quadrotor/Topic.h>

using namespace hal_quadrotor;

// Send the data to the ROS messaging service, for collection by others
void TopicBase::Broadcast(const ros::TimerEvent& event)
{                    
    publisher.broadcast(message);
}

// Default callback 
void TopicBase::Receive(const T &msg) {}

// Constructor creates publisher handle and sets default rate
TopicBase::TopicBase(ros::NodeHandle &node, std::string name)
{
    // Advertise this message on the ROS backbone
    publisher = node.advertise<T>(name.c_str(), DEFAULT_QUEUE_LENGTH);

    // Create a timer to broadcast the data
    timer = node.createTimer(
        ros::Duration(1.0/DEFAULT_BROADCAST_RATE),  // Callback rate
        &Peripheral::Broadcast,                     // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );
}

// Configure data broadcast at a given rate (<= 0.0 means disable)
void TopicBase::Reset(const double &rate)
{
    // Stop publishing for now
    timer.stop();

    // Create a timer callback
    if (rate > 0)
    {
        // Update the timer period
        timer.setPeriod(1.0/rate);

        // Restart timer
        timer.restart();
    }
}

// Manually set the data in this message (usually called by FCS / simulation)
void TopicBase::Set(const T &msg)
{
    // Set the data locally for periodic transmission
    message = msg;

    // Rebroadcast data to parent class (if needed)
    Receive(message);
}

// Get a copy of the message
T TopicBase::Get()
{
    return message;
}


// Constructor
Topic::Topic(ros::NodeHandle &node, std::string name) : TopicBase(node, name)
{
    // Downcast for easy peripheral management
    sensors[name] = (SensingBase*) this;
}

// Dynamic configuration of broadcast rate
bool Topic::Config(const std::string &name, const double &rate)
{
    if (sensors.count(name) > 0)
        return sensors[name]->Reset(rate);
    return false;
}