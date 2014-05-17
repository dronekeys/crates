// Standar dlibraries
#include <boost/shared_ptr.hpp>

// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Basic ROS includes
#include <ros/ros.h>

// Wind message
#include "wind.pb.h"

// Basic constants 
#define METERS_TO_FEET      3.2808399
#define FEET_TO_METERS      0.3048000
#define DEGREES_TO_RADIANS  0.01745329258399
#define RADIANS_TO_DEGREES  57.2957795000000
#define MINIMUM_ALTITUDE    0.05
#define WIND_CONSTANT       0.15
#define INIT_ITERATIONS     0.02
#define INIT_DT             0.02

namespace gazebo
{
    // Convenience declaraion
    typedef const boost::shared_ptr<const msgs::Wind> WindPtr;

    class Aerodynamics : public ModelPlugin
    {

    private:

        // Pointer to the model object
        physics::ModelPtr           modPtr;

        // Pointer to the update event connection
        event::ConnectionPtr        conPtr;

        // For gazebo messaging
        transport::NodePtr          nodePtr;
        transport::SubscriberPtr    subPtr;

        // Boolean options
        bool useShear, useTurbulence;

        // Current time
        double tim, s20, kuv, kw, speed, dir;

        // Internal parameters
        math::Vector3 d20, drag, shear, turbulence, s, l;

        // Update the model based on the time step (and internal control)
        void PrePhysics(const common::UpdateInfo& _info)
        {
            // Time over which dynamics must be updated (needed for thrust update)
            double dt = _info.simTime.Double() - tim;

            // If we have moved forward in time
            if (dt > 0)
            {
                // Extract the altitude and orientation from the state
                double a = modPtr->GetLink("body")->GetWorldPose().pos.z;

                // Vind models only valid above a certain height
                if (a > MINIMUM_ALTITUDE)
                {
                    if (useShear)
                        CalculateShear(dt, a);
                    if (useTurbulence)
                        CalculateTurbulence(dt, a);
                }

                // Add the force to the body
                modPtr->GetLink("body")->AddRelativeForce(
                    drag
                    * modPtr->GetLink("body")->GetInertial()->GetMass()
                    * modPtr->GetLink("body")->GetWorldPose().rot.RotateVector(
                        modPtr->GetLink("body")->GetRelativeLinearVel() 
                        - shear
                        - turbulence
                    )
                );
            }

            // Update timer
            tim = _info.simTime.Double();
        }

        // Update the shear
        void CalculateShear(const double &dt, const double &a)
        {
            if (a > MINIMUM_ALTITUDE)
                shear = FEET_TO_METERS * s20 * (log(METERS_TO_FEET*a/WIND_CONSTANT)/log(20.0/WIND_CONSTANT)) * d20;
            else
                shear.Set(0,0,0);
        }

        // Update the turbulence based on the time step
        void CalculateTurbulence(const double &dt, const double &a)
        {
            // Extract the altitude and orientation from the state
            double d = METERS_TO_FEET * modPtr->GetLink("body")->GetWorldLinearVel().GetLength() * dt;
            double k = 0.177 + 0.000823 * a;

            // sigma
            s.z = s20 * 0.1;
            s.y = s.z / pow(k, 0.4);
            s.x = s.y;
            
            // length scale
            l.z = a;
            l.y = l.z / pow(k, 1.2);
            l.x = l.y;

            // Gust X component
            turbulence.x = math::Rand::GetDblNormal(
                (1-d/l.x) *  turbulence.x,    // Mean
                sqrt(2*d/l.x) * s.x           // Stddev
            );

            // Gust Y component
            turbulence.y = math::Rand::GetDblNormal(
                (1-d/l.y) *  turbulence.y,    // Mean
                sqrt(2*d/l.y) * s.y           // Stddev
            );

            // Gust Z component
            turbulence.z = math::Rand::GetDblNormal(
                (1-d/l.z) *  turbulence.z,    // Mean
                sqrt(2*d/l.z) * s.z           // Stddev
            );
        }

        // Periodically the simulation produces a wind message
        void ReceiveWind(WindPtr &msg)
        {
            // Work out the current platform altitude
            double a = METERS_TO_FEET * modPtr->GetLink("body")->GetWorldPose().pos.z + 0.001;

            // Wind can be initialised only at a minimum altitude
            if (a > MINIMUM_ALTITUDE)
            {
                // We're only interested if the wind changes
                if (dir == msg->direction() && speed == msg->speed())
                    return;

                // Save the new direction and speed
                dir   = msg->direction();
                speed = msg->speed();

                // Wind always blows orthogonal to the down direction
                d20.x = -cos(DEGREES_TO_RADIANS * msg->direction());
                d20.y = -sin(DEGREES_TO_RADIANS * msg->direction());
                d20.z = 0.0;

                // Get the speed at 20ft
                s20 = METERS_TO_FEET * msg->speed();

                // optimization
                double k = 0.177 + 0.000823 * a;
        
                // Initialise sigma
                s.x = 1.0 / pow(k,0.4);
                s.y = 1.0 / pow(k,0.4);
                s.z = 1.0;
                s  *= 0.1 * s20;
                
                // Initialise length scale
                l.x = 1.0 / pow(k,1.2);
                l.y = 1.0 / pow(k,1.2);
                l.z = 1.0;
                l  *= a;

                // Bootstrap the turbulence model
                for (int i = 0; i < INIT_ITERATIONS; i++)
                    CalculateTurbulence(INIT_DT, a);

            }
        }

    public:

        // Constructor
        Aerodynamics() :
            kuv(4.97391e-01), kw(1.35341), 
            useShear(true), useTurbulence(true), 
            d20(0,0,0), s20(0), speed(0.0), dir(0.0),
            turbulence(0,0,0), shear(0,0,0)

        {
            // Make sure that ROS actually started, or there will be some issues...
            if (!ros::isInitialized())
                ROS_FATAL("A ROS node has not been initialized");
        }

        // Destructor
        ~Aerodynamics()
        {
            transport::fini();
        }

        //  Configure the propulsion engine
        void Load(physics::ModelPtr model, sdf::ElementPtr root)
        {
            // save the model pointer
            modPtr = model;

            // Get double parameters
            root->GetElement("drag")->GetElement("kuv")->GetValue()->Get(kuv);
            root->GetElement("drag")->GetElement("kw")->GetValue()->Get(kw);
            
            // Get boolean parameters
            std::string tmp;
            root->GetElement("shear")->GetValue()->Get(tmp);
            useShear = (tmp.compare("true") == 0);
            root->GetElement("turbulence")->GetValue()->Get(tmp);
            useTurbulence = (tmp.compare("true") == 0);
            
            // Set the drag force
            drag.Set(kuv,kuv,kw);

            // Initialise a node pointer
            nodePtr = transport::NodePtr(new transport::Node());
            nodePtr->Init(modPtr->GetWorld()->GetName());

            // Subscribe to messages about wind conditions
            subPtr = nodePtr->Subscribe("~/wind", &Aerodynamics::ReceiveWind, this);

            //  Create a pre-physics update call
            conPtr = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Aerodynamics::PrePhysics, this, _1));

            // Call a Reset()
            Reset();
        }

        // Reset the propulsion engine
        void Reset()
        {
            // Reset the timer
            tim   = 0.0;

            // Reset the wind vectors
            d20.Set(0.0,0.0,0.0);
            shear.Set(0.0,0.0,0.0);
            turbulence.Set(0.0,0.0,0.0);

            // Reset intermediary wind parameters
            s20   = 0.0;
            speed = 0.0;
            dir   = 0.0;
        }
    };

    // Register the plugin
    GZ_REGISTER_MODEL_PLUGIN(Aerodynamics);
}



