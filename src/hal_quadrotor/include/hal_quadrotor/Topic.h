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
    // This primarily allows Topic to be cast to a pointer, so the broadcast rates of
    // instances can be easily modified during runtime.
    class TopicBase 
    {

    private:

        // The handle used to publish the messade
        ros::Timer timer;

        // A registry for all sensors in the system
        static std::map<std::string,TopicBase*> sensors;
        
    protected:

        // Topic must implement this function to accept timer callback
        virtual void Broadcast(const ros::TimerEvent& event) = 0;

    public:

        // Constructor creates publisher handle and sets default rate
        TopicBase(ros::NodeHandle &node, std::string name);

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        void Reset(const double &rate);

        // Dynamic configuration of broadcast rate
        static bool Config(const std::string &name, const double &rate);

    };

    // This is a Sensing class, which is designed to be inherited by a HAL. It provides
    // the ability to buffer and forward sensor data, an broadcast it to ROS periodically
    template<class T> 
    class Topic : public TopicBase
    {

    private:

        // The actual message
        T message;

        // Used to broadcast messages
        ros::Publisher publisher;

        // Topic must implement this function to accept timer callback
        void Broadcast(const ros::TimerEvent& event);

    protected:

        // Child class (HAL) can receive the data
        void Receive(const T &msg);        

    public:

        // Constructor
        Topic(ros::NodeHandle &node, std::string name);

        // Manually set the data in this message (usually called by FCS / simulation)
        void Set(const T &msg);

        // Get a copy of the message
        T Get();

    };

}

#endif