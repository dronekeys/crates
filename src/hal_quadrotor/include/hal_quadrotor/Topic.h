#ifndef HAL_QUADROTOR_TOPIC_H
#define HAL_QUADROTOR_TOPIC_H

// Standard libraries
#include <map>

// Basic ros functionality
#include <ros/ros.h>

// Default peripheral data queue length before discard
#define DEFAULT_QUEUE_LENGTH       10
#define DEFAULT_BROADCAST_RATE    0.0

namespace hal_quadrotor
{
    template <class T>
    class TopicBase 
    {

    private:

        // The actual message
        T message;

        // The handle used to publish the messade
        ros::Publisher  publisher;
        ros::Timer      timer;
        
        // Send the data to the ROS messaging service, for collection by others
        void Broadcast(const ros::TimerEvent& event);

    protected:

        // The flight control system should implement a receive method for this message,
        void Receive(const T &msg);
        
    public:

        // Constructor creates publisher handle and sets default rate
        TopicBase(ros::NodeHandle &node, std::string name);

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        void Reset(const double &rate);

        // Manually set the data in this message (usually called by FCS / simulation)
        void Set(const T &msg);

        // Get a copy of the message
        T Get();
    };

    // This is a Sensing class, which is designed to be inherited by a HAL. It provides
    // the ability to buffer and forward sensor data, an broadcast it to ROS periodically
    template<class S> 
    class Topic : public TopicBase<S>
    {

    private:

        // A registry for all sensors in the system
        static std::map<std::string,TopicBase*> sensors;

    public:

        // Constructor
        Topic(ros::NodeHandle &node, std::string name);

        // Dynamic configuration of broadcast rate
        static bool Config(const std::string &name, const double &rate);

    };
}

#endif