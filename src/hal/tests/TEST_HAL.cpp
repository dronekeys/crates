// Gtest framework
#include <gtest/gtest.h>

// ROS includes
#include <ros/ros.h>

// Class to test
#include <hal/HAL.h>

namespace tests
{

  // The fixture for testing class Foo.
  class InheritedHAL : public hal::HAL, public ::testing::Test
  {
  
    private:

      bool testflag;
   
    protected:
    
      // Constructor
      InheritedHAL() : hal::HAL(), testflag(false)
      {
        
      }

      // Destructor
      virtual ~QuadrotorInstance()
      {

      }

      /// Called by the intermediate class (sensor, controller, model, etc.)
      void OnInit() 
      {
        testflag = true;
      }

      virtual void SetUp()
      {
        // Initialise the HAL
        hal::HAL::Init("/hal/test");
      }

      virtual void TearDown()
      {
        // Code here will be called immediately after each test (right
      }

    // Objects declared here can be used by all tests in the test case for Foo.
  };

  // Tests that the Foo::Bar() method does Abc.
  TEST_F(InheritedHAL, CheckInitCalled)
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
*/