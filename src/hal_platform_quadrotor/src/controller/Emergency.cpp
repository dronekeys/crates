#include <hal_platform_quadrotor/controller/Emergency.h>

using namespace hal::controller;

bool Emergency::Receive(
    hal_platform_quadrotor::Emergency::Request  &req, 
    hal_platform_quadrotor::Emergency::Response &res
) {
    return Switch();
}

Emergency::Emergency(const char* name) : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Emergency::Request, hal_platform_quadrotor::Emergency::Response>(name)
{
	Reset();
}

hal_platform_quadrotor::Control Emergency::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;
    return control;
}

// Goal reach implementations
bool Emergency::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void Emergency::Reset()
{
	reach = false;
}