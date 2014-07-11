#include <ros/ros.h>

#include <gazebo/physics/physics.hh>

namespace dronkey {
	class Wireless {
	private:
		
		/**
		 * World Reference
		 */
		gazebo::physics::WorldPtr gWorld;
		
		/**
		 * [areaNodes description]
		 * @return [description]
		 */
		int areaNodes();
	public:

		/**
		 * Set the private world
		 */
		void SetWorld(gazebo::physics::WorldPtr &world);

		/**
		 * Send the data to correct nodes!!
		 * @return Send Successfull or Not.
		 */
		bool Send();

		/**
		 * Constructor
		 */
		Wireless();

	};
}