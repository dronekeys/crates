#include <hal_platform_quadrotor/controller/Takeoff.h>

using namespace hal::controller;

bool Takeoff::Receive(
    hal_platform_quadrotor::Takeoff::Request  &req, 
    hal_platform_quadrotor::Takeoff::Response &res
) {
    return Switch();
}

Takeoff::Takeoff(const char* name) : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Takeoff::Request, hal_platform_quadrotor::Takeoff::Response>(name)
{
    Reset();
}

hal_platform_quadrotor::Control Takeoff::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;
    return control;
}

// Goal reach implementations
bool Takeoff::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void Takeoff::Reset()
{

}
