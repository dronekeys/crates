#ifndef SIM_SENSOR_H
#define SIM_SENSOR_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS stuff
#include <ros/ros.h>

namespace gazebo
{
    class Sensor
    {     
    public:

        //! Configure a sensor
        /*!
          \param msg the message to be populated
        */
        virtual bool Configure(sdf::ElementPtr root) = 0;

        //! Reset a sensor
        /*!
          \param msg the message to be populated
        */
        virtual void Reset() = 0;

    };
}

#endif