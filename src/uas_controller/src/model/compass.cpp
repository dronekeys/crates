//  Boost includes
#include <boost/bind.hpp>

// ROS includes
#include <ros/ros.h>

// Gazebo includes
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>

// HAL includes
#include <uas_hal/peripheral/Magnetic.h>

namespace uas_controller
{
	class Compass : public uas_hal::Magnetic,  public gazebo::ModelPlugin
	{

	private:

	    // Pointer to the current model
	    gazebo::physics::ModelPtr	 modPtr;

	    // Pointer to the update event connection
	    gazebo::event::ConnectionPtr conPtr;
	    
	    // Listen to broadcasts from the atmosphere
		ros::Timer timer;

	    // Magnetic field
	    gazebo::math::Vector3 mag;

	public:

		Compass() : uas_hal::Magnetic("compass") {}

		// On initial load
	    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) 
	    {
			// Save the model pointer
			modPtr = _model;
		      
			// Set up callback for updating the model (respects sim time)
            timer = GetROSNode().createTimer(
                ros::Duration(1.0),  				     	// duration
                boost::bind(&Compass::Update, this, _1),  	// callback
                false                                       // oneshot?
            );
	    }

		// When called published the data
		void Update(const ros::TimerEvent& event)
		{
		  	// Magnetic field
			mag.x = 1;
			mag.y = 0;
			mag.z = 0;

			// Rotate world magnetic vector from global to local
			mag = modPtr->GetWorldPose().rot.GetInverse().RotateVector(mag);

			// Immediately post the euler angles
			Post(
				mag.x,  // Magnetic field strength X (Gauss)
				mag.y,  // Magnetic field strength Y (Gauss)
				mag.z   // Magnetic field strength Z (Gauss)
			);
		}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(Compass)
}