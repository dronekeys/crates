#include <hal/platform/quadrotor/controller/Land.h>

using namespace hal::controller;

bool Land::Receive(
    hal_platform_quadrotor::Land::Request  &req, 
    hal_platform_quadrotor::Land::Response &res
) {
    return Switch();
}

Land::Land(ros::NodeHandle& node, const char *name) : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Land::Request, hal_platform_quadrotor::Land::Response>(node, name)
{
    Reset();
}

hal_platform_quadrotor::Control Land::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;
    return control;
}

// Goal reach implementations
bool Land::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void Land::Reset()
{
	reach = false;
}