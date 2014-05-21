#ifndef SIM_DYNAMICS_H
#define SIM_DYNAMICS_H

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS stuff
#include <ros/ros.h>

namespace gazebo
{
    class Dynamics
    {     
    public:

        //! Configure dynamics
        /*!
          \param msg the message to be populated
        */
        virtual bool Configure(sdf::ElementPtr root) = 0;

        //! Reset dynamics
        /*!
          \param msg the message to be populated
        */
        virtual void Reset() = 0;

        //! Update dynamics
        /*!
          \param msg the message to be populated
        */
        virtual void Update(physics::LinkPtr linkPtr, double dt) = 0;

    };
}

#endif