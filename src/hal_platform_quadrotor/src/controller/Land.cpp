#include <hal_platform_quadrotor/controller/Land.h>

using namespace hal::controller;

bool Land::Receive(
    hal_platform_quadrotor::Land::Request  &req, 
    hal_platform_quadrotor::Land::Response &res
) {
    return Switch();
}

Land::Land() : Controller<hal_platform_quadrotor::State, hal_platform_quadrotor::Control,
	hal_platform_quadrotor::Land::Request, hal_platform_quadrotor::Land::Response>("Land")
{}

hal_platform_quadrotor::Control Land::Update(
	const hal_platform_quadrotor::State &state, 
	const double &dt
) {
	hal_platform_quadrotor::Control control;
    return control;
}

// Reset the current state
void Land::Reset()
{

}