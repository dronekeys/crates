#ifndef HAL_QUADROTOR_SERVICE_H
#define HAL_QUADROTOR_SERVICE_H

// System includes
#include <ros/ros.h>

// All the possible controllers
#include <hal_quadrotor/controller/Controller.h>
#include <hal_quadrotor/configuration/Configuration.h>

namespace hal_quadrotor
{
    // Service requires a controller class
    template<class C>
    class Service
    {

    private: 
        
        // Manages incoming service requests
        ros::ServiceServer service;

        // The controller associated with this 
        C controller;

    public:

        // Constructor creates publisher handle and sets default rate
        Service(ros::NodeHandle &node, std::string name);
    };
}

#endif