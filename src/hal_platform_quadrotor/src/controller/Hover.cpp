#include <hal_platform_quadrotor/controller/Hover.h>

using namespace hal::controller;

bool Hover::Receive(
    hal_platform_quadrotor::Hover::Request  &req, 
    hal_platform_quadrotor::Hover::Response &res
) {
    // Try and switch control
    return Switch();
}

Hover::Hover() : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Hover::Request, hal_platform_quadrotor::Hover::Response>("Hover")
{}

hal_platform_quadrotor::Control Hover::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;	
    return control;
}

// Reset the current state
void Hover::Reset()
{

}