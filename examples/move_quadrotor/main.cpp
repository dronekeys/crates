#include <ros/ros.h>
#include <Navigate.h>

#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

//#include <hal_sensor_transceiver/Receiver.h>
//#include <hal_sensor_transceiver/Transmitter.h>
#include <hal_quadrotor/control/Takeoff.h>

/*
void ReceiverCallback(hal_sensor_transceiver::Data msg)
{
  ROS_INFO("Quadrotor Receiver: [%f, %f]", msg.gain, msg.power);
}

void TransmitterCallback(hal_sensor_transceiver::TData msg)
{
	ROS_INFO("Quadrotor Transmitter: [%f, %f]", msg.gain, msg.power);
}
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_quadrotor");

	ros::NodeHandle n;

	ros::ServiceClient srvInsert = n.serviceClient<sim::Insert>("/simulator/Insert");
	ros::ServiceClient srvResume = n.serviceClient<sim::Resume>("/simulator/Resume");
	ros::ServiceClient srvPause = n.serviceClient<sim::Pause>("/simulator/Pause");
	
	
	//ros::Subscriber receiverState = n.subscribe("/hal/UAV2/sensor/receiver/Data", 1000, ReceiverCallback);
	//ros::Subscriber tranmsitterState = n.subscribe("/hal/UAV2/sensor/transmitter/Data", 1000, TransmitterCallback);
 
	sim::Pause msgPause;

	if(!srvPause.call(msgPause)){
		ROS_FATAL("Failed to pause the simulator");
		return 1;
	}

	sim::Insert msgInsert;
	msgInsert.request.model_name = "UAV2";
	msgInsert.request.model_type = "model://hummingbird";
	//Insert new quadcopter
	if(!srvInsert.call(msgInsert)){
		ROS_FATAL("Failed to insert a hummingbird");
		return 1;
	}

	sim::Resume msgResume;
	if(!srvResume.call(msgResume)){
		ROS_FATAL("Failed to resume the simulator");
		return 1;
	}

	ros::Rate rate(4);
	rate.sleep();

	hal_quadrotor::Takeoff req_Takeoff;
	req_Takeoff.request.altitude = 4.0;

	//Initialize Clients needeed!!
	ros::ServiceClient srvTakeOff = n.serviceClient<hal_quadrotor::Takeoff>("/hal/UAV2/controller/Takeoff");
	if(!srvTakeOff.call(req_Takeoff)){
		ROS_FATAL("NO TAKE OFF!!");
		return 1;
	}

	dronkey::Navigate navigator(n);
	
	ros::spin();

	//Success!!
	return 0;
}