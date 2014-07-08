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

	srvHover = n.serviceClient<hal_quadrotor::Hover>("/hal/UAV2/controller/Hover");
	srvWaypoint = n.serviceClient<hal_quadrotor::Waypoint>("/hal/UAV2/controller/Waypoint");
	quadState = n.subscribe("/hal/UAV2/Estimate", 1000, &Navigate::StateCallback, this);
}	

void Navigate::StateCallback(const hal_quadrotor::State::ConstPtr& msg){
	ROS_INFO("Quadrotor Update: [%f, %f, %f, %f] Destination State: [%d, %d]", msg->x, msg->y, msg->z, msg->yaw, msg->rch, msg->ctrl);
	hal_quadrotor::Waypoint& tmp = *movement_iterator;
	
	if(msg->ctrl == 2 || msg->rch == 1){
		ROS_INFO("GOAL REACHED!!");
		if(msg->ctrl != 2 && msg->rch == 1){
			if(!srvHover.call(req_Hover)){
				ROS_FATAL("NO HOVER!!");
			}
		}

		if(movement.end() != movement_iterator){
			++movement_iterator;
			tmp = *movement_iterator;
			ROS_INFO("!!!--CALLING NEW WAYPOINT--!!!");
			if(!srvWaypoint.call(tmp)){
				ROS_FATAL("NO WAYPOINT!!");
			}
		} else {
			movement_iterator = movement.begin();
		}
	}
}

Navigate::~Navigate(){

}