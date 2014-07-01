#include <ros/ros.h>

#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

#include <hal_quadrotor/State.h>
#include <hal_quadrotor/control/Takeoff.h>
#include <hal_quadrotor/control/Waypoint.h>
#include <hal_quadrotor/control/Hover.h>
#include <hal_sensor_transceiver/Receiver.h>

void StateCallback(const hal_quadrotor::State::ConstPtr& msg){
	ROS_INFO("Quadrotor position: [%f, %f, %f]", msg->x, msg->y, msg->z);
}

void ReceiverCallback(hal_sensor_transceiver::Data msg)
{
  ROS_INFO("Quadrotor Receiver: [%f, %f]", msg.gain, msg.power);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_quadrotor");

	ros::NodeHandle n;

	ros::ServiceClient srvInsert = n.serviceClient<sim::Insert>("/simulator/Insert");
	ros::ServiceClient srvResume = n.serviceClient<sim::Resume>("/simulator/Resume");
	ros::ServiceClient srvPause = n.serviceClient<sim::Pause>("/simulator/Pause");
	
	
	ros::Subscriber topState = n.subscribe("/hal/UAV2/Estimate", 1000, StateCallback);	
	ros::Subscriber receiverState = n.subscribe("/hal/UAV0/sensor/receiver/Data", 1000, ReceiverCallback);

	sim::Pause msgPause;

	if(!srvPause.call(msgPause)){
		ROS_FATAL("Failed to pause the simulator");
		return 1;
	}

	sim::Insert msgInsert;
	msgInsert.request.model_name = "UAV2";
	msgInsert.request.model_type = "model://hummingbird";

	if(!srvInsert.call(msgInsert)){
		ROS_FATAL("Failed to insert a hummingbird");
		return 1;
	}

	sim::Resume msgResume;

	if(!srvResume.call(msgResume)){
		ROS_FATAL("Failed to resume the simulator");
		return 1;
	}

	ROS_INFO("Successfully resumed the simulator");
	
	ros::Rate rate(2);
	rate.sleep();

	hal_quadrotor::Takeoff req_Takeoff;
	req_Takeoff.request.altitude = 4.0;

	ros::ServiceClient srvTakeOff = n.serviceClient<hal_quadrotor::Takeoff>("/hal/UAV2/controller/Takeoff");
	if(!srvTakeOff.call(req_Takeoff)){
		ROS_FATAL("NO TAKE OFF!!");
		return 1;
	} else {
		ROS_INFO("TAKE OFF!!");
	}

	hal_quadrotor::Hover req_Hover;

	ros::ServiceClient srvHover = n.serviceClient<hal_quadrotor::Hover>("/hal/UAV2/controller/Hover");
	if(!srvHover.call(req_Hover)){
		ROS_FATAL("NO HOVER!!");
	}	

	ros::spin();

	//Success!!
	return 0;
}