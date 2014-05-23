#ifndef SIM_SENSOR_H
#define SIM_SENSOR_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Required for noise distributions
#include "../../../noise/NoiseFactory.h"

// Basic ROS stuff
#include <ros/ros.h>

namespace gazebo
{
    class Sensor
    {     
    public:

        //! Configure a sensor
        /*!
          \param root the sdf sensor configuration
        */
        virtual bool Configure(physics::LinkPtr link, sdf::ElementPtr root) = 0;

        //! Reset a sensor
        virtual void Reset() = 0;

    };
}

#endif