#include <hal/platform/quadrotor/controller/Hover.h>

using namespace hal::controller;

bool Hover::Receive(
    hal_platform_quadrotor::Hover::Request  &req, 
    hal_platform_quadrotor::Hover::Response &res
) {
    // Try and switch control
    return Switch();
}

Hover::Hover(ros::NodeHandle& node, const char* name) : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Hover::Request, hal_platform_quadrotor::Hover::Response>(node, name)
{
    Reset();
}

hal_platform_quadrotor::Control Hover::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;	
    return control;
}

// Goal reach implementations
bool Hover::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void Hover::Reset()
{
	reach = false;
}