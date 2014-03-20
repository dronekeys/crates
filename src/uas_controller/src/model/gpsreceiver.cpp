//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// HAL includes
#include <uas_hal/peripheral/Position.h>

namespace uas_controller
{
	class GPSReceiver : public uas_hal::Position,  public gazebo::ModelPlugin
	{

	private:

	    // Pointer to the current model
	    gazebo::physics::ModelPtr	 modPtr;

	    // Listen to broadcasts from the atmosphere
		ros::Timer timer;

	    // Prevents needless allocation
	    gazebo::math::Vector3 pos, vel, err_pos, err_vel;


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
			Post(	"Simulated, L1 code-phase GPS",
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
		
	public:

		GPSReceiver() : uas_hal::Position("gpsreceiver")
		{
			ROS_INFO("Loaded gps receiver plugin");
		}

		// On initial load
	    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
	    {
			// Save the model pointer
			modPtr = _model;

			// Set up callback for updating the model
            timer = GetROSNode().createTimer(
                ros::Duration(1.0),  				     		 	// duration
                boost::bind(&GPSReceiver::Update, this, _1),  		// callback
                false                                       		// oneshot?
            );
	    }
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(GPSReceiver)
}