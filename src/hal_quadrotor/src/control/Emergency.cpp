#include <hal_quadrotor/control/Emergency.h>

using namespace hal::quadrotor;

bool Emergency::SetGoal(
    hal_quadrotor::Emergency::Request  &req, 
    hal_quadrotor::Emergency::Response &res
) {
    // Eveything OK
    res.success = true;
    res.status  = "Successfully switched to Emergency controller";
    return true;
}

bool Emergency::Update(const hal_quadrotor::State &state, 
    double dt, hal_quadrotor::Control &control)
{
	// Do something more elegant than this ultimately...
   	control.roll     = 0.0;
    control.pitch    = 0.0;
    control.yaw      = 0.0;
    control.throttle = 0.0;
    return true;
}

// Goal reach implementations
bool Emergency::HasGoalBeenReached()
{
    return false;
}