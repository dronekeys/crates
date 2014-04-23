#include <hal_platform_quadrotor/controller/Idle.h>

using namespace hal::controller;

bool Idle::Receive(
    hal_platform_quadrotor::Idle::Request  &req, 
    hal_platform_quadrotor::Idle::Response &res
) {
   return Switch();
}

Idle::Idle(const char* name) : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Idle::Request, hal_platform_quadrotor::Idle::Response>(name)
{
    Reset();
}

hal_platform_quadrotor::Control Idle::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;
    return control;
}

// Goal reach implementations
bool Idle::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void Idle::Reset()
{
	reach = false;
}