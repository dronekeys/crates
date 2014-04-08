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
        bool Receive(ConfigRate::Request &req, ConfigRate::Response &res);

    protected:

        // HAL needs to ovveride this
        virtual bool ConfigRate(const std::string& name, const double& rate) = 0;

    public:

        // Constructor
        Rate(ros::NodeHandle &node, std::string name);

    };
}

#endif