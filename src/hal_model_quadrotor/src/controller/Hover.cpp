#include <hal/model/quadrotor/controller/Hover.h>

using namespace hal::controller;

bool Hover::Receive(
    hal_model_quadrotor::Hover::Request  &req, 
    hal_model_quadrotor::Hover::Response &res
) {
    // Try and switch control
    return Switch();
}

Hover::Hover(const char* name) : Controller<hal_model_quadrotor::State, hal_model_quadrotor::Control,
	hal_model_quadrotor::Hover::Request, hal_model_quadrotor::Hover::Response>(name)
{
    Reset();
}

hal_model_quadrotor::Control Hover::Update(
	const hal_model_quadrotor::State &state, 
	const double &dt
) {
	hal_model_quadrotor::Control control;	
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