#include <ros/ros.h>

#include <gazebo/physics/physics.hh>

namespace dronkey {
	class Wireless {
	private:
		
		gazebo::physics::WorldPtr gWorld;

		bool Send();
		
	public:

		Wireless(gazebo::physics::WorldPtr &world);

	};
}