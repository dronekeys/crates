// Gtest framework
#include <gtest/gtest.h>

// ROS includes
#include <ros/ros.h>

// Class to test
#include <hal/model/Quadrotor.h>

namespace tests
{

  // The fixture for testing class Foo.
  class QuadrotorInstance : public hal::model::Quadrotor, public ::testing::Test
  {

   protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    QuadrotorInstance() : hal::model::Quadrotor()
    {
      // You can do set-up work for each test here.
    }

    virtual ~QuadrotorInstance()
    {

    }

    // The HAL gets the simulated state through this function
    void GetState(hal_model_quadrotor::State& state) {}

    // The HAL sets the simulated state through this function
    void SetState(const hal_model_quadrotor::State& state) {}

    // The HAL sets the control through this function
    void SetControl(const hal_model_quadrotor::Control &control) {}

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp()
    {
      hal::HAL::Init((std::string)"/hal/test");
    }

    virtual void TearDown()
    {
      // Code here will be called immediately after each test (right
    }

    // Objects declared here can be used by all tests in the test case for Foo.
  };

  // Tests that the Foo::Bar() method does Abc.
  TEST_F(QuadrotorInstance, CheckWorking)
  {
    // DO something
    EXPECT_EQ(1, 1);
  }

}

int main(int argc, char **argv)
{
  // Start ROS core
  system("/opt/ros/indigo/bin/roscore > /dev/null &");

  // This needs to be called before all tests
  ros::init(argc,argv,"test");

  // Wait until the ROS master becomes available
  while(!ros::master::check())
    usleep(500*1000);

  // Initialise the tests
  ::testing::InitGoogleTest(&argc, argv);
  
  // Run all the tests
  int ret =  RUN_ALL_TESTS();

  // Shut down ROS cleanly
  ros::shutdown();

  // Start ROS core
  system("/usr/bin/killall roscore");

  // Return result
  return ret;
}