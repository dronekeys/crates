// Required for the maths functions
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// ROS communication subsystem (mainly for debugging)
#include <ros/ros.h>

// ROS communication subsystem (mainly for debugging)
#include "wind.pb.h"

namespace gazebo
{
	class Wind : public WorldPlugin
	{

	private:

	    // Pointer to the simulated world
	    physics::WorldPtr 			worldPtr;
		
	    // For gazebo messaging
		transport::NodePtr 			nodePtr;
		transport::PublisherPtr 	pubPtr;
		transport::SubscriberPtr 	subPtr;

		// For ROS timers
		ros::Timer 					timer;
		ros::NodeHandle 			rosNode;

		// Wind broadcast rate
		double 						rate, speed, direction;

      	// Message containing information
      	msgs::Wind 					msg;

	    //  Called to update the world information
		void Update(const ros::TimerEvent& event)
		{
			msg.set_speed(speed);
			msg.set_direction(direction);
			pubPtr->Publish(msg);
		}

    public:

		// Default constructor
		Wind() : rate(0.0), speed(0.0), direction(0.0), rosNode(ros::NodeHandle("wind"))
		{
		    // Maske sure that ROS actually started, or there will be some issues...
		    if (!ros::isInitialized())
		        ROS_FATAL("A ROS node has not been initialized");
		}

		// All sensors must be configured using the current model information and the SDF
		void Load(physics::WorldPtr world, sdf::ElementPtr root)
		{
			// Save the world pointer
			worldPtr = world;

			// Get the rate and start time
			root->GetElement("rate")->GetValue()->Get(rate);
			root->GetElement("default")->GetElement("speed")->GetValue()->Get(speed);
			root->GetElement("default")->GetElement("heading")->GetValue()->Get(direction);
			
			// Create and initialize a new Gazebo transport node
			nodePtr = transport::NodePtr(new transport::Node());
			nodePtr->Init(worldPtr->GetName());

			// Create a publisher on the ~/wind topic
			pubPtr = nodePtr->Advertise<msgs::Wind>("~/wind");

			// ROS timer respects gazebo
			if (rate > 0)
			{
				timer = rosNode.createTimer(
					ros::Duration(1.0/rate),
					&Wind::Update,
					this,
			        false,                                      // Oneshot
			        true                                        // Autostart
				);  
			}    

			// Set up advertisements
			Reset();
		}

		// Reset the publishers and subscribers
		void Reset()
		{
			// Reset         
		}
	};

	// Resgister the plugin
	GZ_REGISTER_WORLD_PLUGIN(Wind);
}