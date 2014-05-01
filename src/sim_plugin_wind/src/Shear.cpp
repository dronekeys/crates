// Standar dlibraries
#include <boost/shared_ptr.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Basic ROS includes
#include <ros/ros.h>

// Wind message
#include "wind.pb.h"

// Basic constants 
#define METERS_TO_FEET    3.2808399
#define FEET_TO_METERS    0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000

namespace gazebo
{
    // Convenience declaraion
    typedef const boost::shared_ptr<const msgs::Wind> WindPtr;

    class Shear : public ModelPlugin
    {

    private:

        // Pointer to the model object
        physics::ModelPtr  modPtr;

        // Pointer to the update event connection
        event::ConnectionPtr conPtr;

        // For gazebo messaging
        transport::NodePtr nodePtr;
        transport::SubscriberPtr subPtr;

        // Current time
        double tim;

        // Input parameters from SDF
        double speed, dir, mA, z0, kuv, kw;

        // Internal parameters
        math::Quaternion q;
        math::Vector3 d20, wind, drag;
        double mass, s20, a;

        // Update the model based on the time step (and internal control)
        void PrePhysics(const common::UpdateInfo& _info)
        {
            // Time over which dynamics must be updated (needed for thrust update)
            double dt = _info.simTime.Double() - tim;

            // If we have moved forward in time
            if (dt > 0)
            {
                // Extract the altitude and orientation from the state
                a = modPtr->GetLink("body")->GetWorldPose().pos.z;
                q = modPtr->GetLink("body")->GetWorldPose().rot;

                // Calculate the wind vector, taking into account shear
                if (a > mA)
                    wind = FEET_TO_METERS * s20 * (log(METERS_TO_FEET*a/z0)/log(20.0/z0)) * d20;
                else
                    wind.Set(0,0,0);

                // Add the force to the body
                modPtr->GetLink("body")->AddRelativeForce(
                    -drag*(q.RotateVector(wind))
                );
            }

            // Update timer
            tim = _info.simTime.Double();
        }

        // Periodically the simulation produces a wind message
        void ReceiveWind(WindPtr &msg)
        {
            // Wind always blows orthogonal to the down direction
            d20.x = -cos(DEGREES_TO_RADIANS * msg->direction());
            d20.y = -sin(DEGREES_TO_RADIANS * msg->direction());
            d20.z = 0.0;

            // Get the speed at 20ft
            s20 = METERS_TO_FEET * msg->speed();
        }

    public:

        // Default constructor
        Shear() : mA(0.05), z0(0.15), speed(0), dir(0), kuv(-4.97391e-01), kw(-1.35341) 
        {
            // Make sure that ROS actually started, or there will be some issues...
            if (!ros::isInitialized())
                ROS_FATAL("A ROS node has not been initialized");
        }

        //  Configure the propulsion engine
        void Load(physics::ModelPtr model, sdf::ElementPtr root)
        {
            // save the model pointer
            modPtr = model;

            // Speed and direction
            root->GetElement("const")->GetValue()->Get(z0);
            root->GetElement("floor")->GetValue()->Get(mA);

            // Speed and direction
            root->GetElement("speed")->GetValue()->Get(speed);
            root->GetElement("direction")->GetValue()->Get(dir);

            // Aerodynamic drag (math::Rand::GetDblUniform(-MATH_PI,MATH_PI)))
            root->GetElement("drag")->GetElement("kuv")->GetValue()->Get(kuv);
            root->GetElement("drag")->GetElement("k")->GetValue()->Get(kw);

            // Find the drag vector
            drag.Set(kuv,kuv,kw);
            drag *= modPtr->GetLink("body")->GetInertial()->GetMass();

            // Initialise a node pointer
            nodePtr = transport::NodePtr(new transport::Node());
            nodePtr->Init(modPtr->GetWorld()->GetName());

            // Subscribe to messages about wind conditions
            subPtr = nodePtr->Subscribe("~/wind", &Shear::ReceiveWind, this);

            //  Create a pre-physics update call
            conPtr = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Shear::PrePhysics, this, _1));

            // Call a Reset()
            Reset();
        }

        // Reset the propulsion engine
        void Reset()
        {
            // Wind always blows orthogonal to the down direction
            d20.x = -cos(DEGREES_TO_RADIANS * dir);
            d20.y = -sin(DEGREES_TO_RADIANS * dir);
            d20.z = 0.0;

            // Get the speed at 20ft
            s20 = METERS_TO_FEET * speed;
        }
    };

    // Register the plugin
    GZ_REGISTER_MODEL_PLUGIN(Shear);
}
