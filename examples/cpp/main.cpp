// General ROS functionality
#include <ros/ros.h>

// Simulator services
#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

// Quadrotor services
#include <hal_quadrotor/State.h>

// Callback for quadrotor state
void StateCallback(const hal_quadrotor::State::ConstPtr& msg)
{
  ROS_INFO("Quadrotor position: [%f,%f,%f]", msg->x, msg->y, msg->z);
}

// Main entry point of application
int main(int argc, char **argv)
{
  // Initialise the ROS client
  ros::init(argc, argv, "example");

  // Create a node handle, which this C code will use to bind to the ROS server
  ros::NodeHandle n;

  // Create a client for interacting with the Simulator insert and Resume services
  ros::ServiceClient srvInsert = n.serviceClient<sim::Insert>("/simulator/Insert");
  ros::ServiceClient srvResume = n.serviceClient<sim::Resume>("/simulator/Resume");
  ros::ServiceClient srvPause  = n.serviceClient<sim::Pause> ("/simulator/Pause");

  // Subscribe to the state of the quadrotor
  ros::Subscriber topState = n.subscribe("/hal/UAV0/Estimate", 1000, StateCallback);

  // Create a resume message, which takes no arguments
  sim::Pause msgPause;

  // Call the client with this message
  if (!srvPause.call(msgPause))
  {
    ROS_FATAL("Failed to pause the simulator");
    return 1;
  }

  // Create a message instructing a quadrotor UAV0 to be inserted in to the world
  sim::Insert msgInsert;
  msgInsert.request.model_name = "UAV0";
  msgInsert.request.model_type = "model://hummingbird";
  
  // Call the client with this message
  if (!srvInsert.call(msgInsert))
  {
    ROS_FATAL("Failed to insert a hummingbird quadrotor into the simulator");
    return 1;
  }

  // You should now see the quadrotor
  ROS_INFO("Successfully inserted the model into the world");

  // Create a resume message, which takes no arguments
  sim::Resume msgResume;

  // Call the client with this message
  if (!srvResume.call(msgResume))
  {
    ROS_FATAL("Failed to resume the simulator");
    return 1;
  }

  // You should now see the quadrotor
  ROS_INFO("Successfully resumed the simulation");

  // Keep going until ctl+c is pressed
  ros::spin();

  // Success!
  return 0;
}