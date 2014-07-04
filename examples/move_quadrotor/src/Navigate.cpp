#include <Navigate.h>

using namespace dronkey;

Navigate::Navigate(ros::NodeHandle &n){
	hal_quadrotor::Waypoint wp2, wp3, wp4;

	wp2.request.x = 15.0;
	wp2.request.y = 11.0;
	wp2.request.z = 6.0;
	wp2.request.yaw = 1.0;
	movement.push_back(wp2);

	wp3.request.x = 5.0;
	wp3.request.y = 5.0;
	wp3.request.z = 4.0;
	wp3.request.yaw = 1.0;
	movement.push_back(wp3);

	wp4.request.x = 18.0;
	wp4.request.y = 14.0;
	wp4.request.z = 6.0;
	wp4.request.yaw = 1.0;
	movement.push_back(wp4);

	movement_iterator = movement.begin();
	quadState = n.subscribe("/hal/UAV2/Estimate", 1000, &Navigate::StateCallback, this);
}	

void Navigate::StateCallback(const hal_quadrotor::State::ConstPtr& msg){
	ROS_INFO("Quadrotor Update: [%f, %f, %f, %f] Destination State: [%d, %d]", msg->x, msg->y, msg->z, msg->yaw, msg->rch, msg->ctrl);
}

Navigate::~Navigate(){

}