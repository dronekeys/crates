#include <gtest/gtest.h>

#include "../src/noise/NoiseFactory.h"

namespace tests
{

  // The fixture for testing class Foo.
  class DrydenTest : public ::testing::Test
  {
   protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    DrydenTest() {
      // You can do set-up work for each test here.
      gazebo::NoiseFactory::Init();
    }

    virtual ~DrydenTest() {
      // You can do clean-up work that doesn't throw exceptions here.
      gazebo::NoiseFactory::Destroy();
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() {
      // Code here will be called immediately after the constructor (right
      // before each test).
      
      // Create a new dryden
      gazebo::Noise *dryden = (gazebo::Noise*) new gazebo::Dryden();

      // Parameterise the turbulence model
      dryden->Configure(DRYDEN_PARS_WNDSPEED, 1.0);
      dryden->Configure(DRYDEN_PARS_AIRSPEED, 2.0);
      dryden->Configure(DRYDEN_PARS_ALTITUDE, 5.0);

      // Sample from the dryden process
      gazebo::math::Vector3 err = dryden->DrawVector(0.02);

      // Delete the allocated memory
      delete dryden;
    }

    virtual void TearDown() {
      // Code here will be called immediately after each test (right
      // before the destructor).
    }

    // Objects declared here can be used by all tests in the test case for Foo.
  };

  // Tests that the Foo::Bar() method does Abc.
  TEST_F(DrydenTest, CheckWorks)
  {
    // DO something
    EXPECT_EQ(1, 1);
  }

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}