#ifndef UAS_HAL_CONTROLLER_H
#define UAS_HAL_CONTROLLER_H

// All possible peripherals
#include <uas_hal/MsgAltitude.h>
#include <uas_hal/MsgInertial.h>
#include <uas_hal/MsgMagnetic.h>
#include <uas_hal/MsgPosition.h>
#include <uas_hal/MsgAttitude.h>

// Default peripheral data queue length before discard
#define DEFAULT_QUEUE_LENGTH 10
#define DEFAULT_POST_RATE    1.0

namespace uas_hal
{
    // T is the message type sent by the peripheral
    template <class T>
    class Peripheral
    {

    private:

        // Internal parameters
        T               message;        // Message to be sent
        ros::NodeHandle node;           // ROS node
        ros::Publisher  publisher;      // Publisher
        ros::Timer      timer;          // Timer

        // Send the data to the ROS messaging service, for collection by others
        void Broadcast(const ros::TimerEvent& event);

    protected:

        // Manually set the data in this message (usually called by FCS)
        virtual void Receive(const T &msg) = 0;

    public:

        // Constructor
        Peripheral(ros::NodeHandle &h, const char *name);

        // Configure broadcast at a particular rate
        void Reset(const double &rate);

        // Manually set the data in this message (usually called by FCS / simulation)
        void Set(const T &msg);

    };
}

#endif