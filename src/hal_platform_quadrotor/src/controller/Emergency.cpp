#include <hal_platform_quadrotor/controller/Emergency.h>

using namespace hal::controller;

bool Emergency::Receive(
    hal_platform_quadrotor::Emergency::Request  &req, 
    hal_platform_quadrotor::Emergency::Response &res
) {
	return SwitchController(name);
}

Emergency::Emergency() : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Emergency::Request, hal_platform_quadrotor::Emergency::Response>("Emergency")
{}

hal_platform_quadrotor::Control Emergency::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;
    return control;
}

// Reset the current state
void Emergency::Reset()
{

}