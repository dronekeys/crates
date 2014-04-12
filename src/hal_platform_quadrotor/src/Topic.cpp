// Standard libraries
#include <hal_quadrotor/Topic.h>

using namespace hal_quadrotor;

// TOPICBASE ///////////////////////////////////////////////////////////

//! A normal member taking two arguments and returning an integer value.
/*!
  \param node an integer argument.
  \param name a constant character pointer.
  \return New TopicBase object
  \sa Test(), ~Test(), testMeToo() and publicVar()
*/
TopicBase::TopicBase(ros::NodeHandle &node, std::string name)
{
    // Create a timer to broadcast the data
    timer = node.createTimer(
        ros::Duration(1.0/DEFAULT_BROADCAST_RATE),  // Callback rate
        &TopicBase::Broadcast,                     // Callback
        this,                                       // Callee
        false,                                      // Oneshot
        true                                        // Autostart
    );

    // Downcast for easy peripheral management
    sensors[name] = this;
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
        timer.setPeriod(ros::Duration(1.0/rate));

        // Restart timer
        timer.start();
    }
}

// Dynamic configuration of broadcast rate
bool TopicBase::Config(const std::string& name, const double& rate)
{
    if (sensors.count(name) > 0)
    {
        sensors[name]->Reset(rate);
        return true;
    }
    return false;
}

// TOPIC ////////////////////////////////////////////////////////////////

// Constructor
template <class T>
Topic<T>::Topic(ros::NodeHandle &node, std::string name) 
    : TopicBase(node)
{
    // Advertise this message on the ROS backbone
    publisher = node.advertise<T>(name.c_str(), DEFAULT_QUEUE_LENGTH);
}

// Send the data to the ROS messaging service, for collection by others
template <class T>
void Topic<T>::Broadcast(const ros::TimerEvent& event)
{                  
    // Send the data  
    publisher.publish(message);
}

// Default callback 
template <class T>
void Topic<T>::Receive(const T &msg)
{
    // Nothing by default
}

// Manually set the data in this message (usually called by FCS / simulation)
template <class T>
void Topic<T>::Set(const T &msg)
{
    // Set the data locally for periodic transmission
    message = msg;

    // Rebroadcast data to parent class (if needed)
    Receive(message);
}

// Get a copy of the message
template <class T>
T Topic<T>::Get()
{
    return message;
}
