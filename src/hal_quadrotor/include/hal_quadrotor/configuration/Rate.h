#ifndef HAL_QUADROTOR_RATE_H
#define HAL_QUADROTOR_RATE_H

// Basi ROS includes
#include <ros/ros.h>

// Messages used and produced by this controller
#include <hal_quadrotor/ConfigRate.h>

namespace hal_quadrotor
{
    class Rate
    {

    private:

        // Manages incoming service requests
        ros::ServiceServer service;

        // Configure data broadcast at a given rate (<= 0.0 means disable)
        void Receive(ConfigRate::Request &req, ConfigRate::Response &res);

    public:

        // Constructor
        Rate();

    };
}

#endif