//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>

// HAL includes
#include <uas_hal/peripheral/Position.h>

namespace uas_controller
{
	class GPS : public uas_hal::Position,  public gazebo::ModelPlugin
	{

	private:

	    // Pointer to the current model
	    gazebo::physics::ModelPtr	 modPtr;

	    // Pointer to the update event connection
	    gazebo::event::ConnectionPtr conPtr;
	    
	    // Listen to broadcasts from the atmosphere
		ros::Timer timer;

	    // Prevents needless allocation
	    gazebo::math::Vector3 pos, vel, err_pos, err_vel;

	public:

		// On initial load
	    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
	    {
			// Save the model pointer
			modPtr = _model;

			// Initialise the HAL
			initialize(((std::string) "/hal/" 
				+		(std::string) modPtr->GetName() 
				+ 		(std::string) "/gps").c_str());			      

			// Set up callback for updating the model
            timer = node.createTimer(
                ros::Duration(1.0),  				     		 	// duration
                boost::bind(&GPS::Update, this, _1),  	// callback
                false                                       		// oneshot?
            );
	    }

		// When called published the data
		void Update(const ros::TimerEvent& event)
		{
			// Get the WGS84 position and velocity
			pos = modPtr->GetWorld()->GetSphericalCoordinates()->SphericalFromLocal(
				modPtr->GetWorldPose().pos
			);
			vel = modPtr->GetWorld()->GetSphericalCoordinates()->GlobalFromLocal(
				modPtr->GetWorldLinearVel()
			);
			
			// Get the errors
			err_pos = gazebo::math::Vector3(0,0,0);
			err_vel = gazebo::math::Vector3(0,0,0);

			// Immediately post the GPS message
			post(	"Simulated, L1 code-phase GPS",
                    "Fix (6 satellites)",
                   	"WGS84",
                    pos.x,
                    pos.y,
                    pos.z,
                    vel.x,
                    vel.y,
                    vel.z,
                    err_pos.x,
                    err_pos.y,
                    err_pos.z,
                    err_vel.x,
                    err_vel.y,
                    err_vel.z,
                    0.0);
		}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(GPS)
}