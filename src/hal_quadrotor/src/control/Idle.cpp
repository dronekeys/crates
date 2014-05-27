#include <hal_quadrotor/control/Idle.h>

using namespace hal::quadrotor;

bool Idle::Update(const hal_quadrotor::State &state, 
    double dt, hal_quadrotor::Control &control)
{
    control.roll     = 0.0;
    control.pitch    = 0.0;
    control.yaw      = 0.0;
    control.throttle = 0.0;
    return true;
}

// Goal reach implementations
bool Idle::HasGoalBeenReached()
{
    return false;
}