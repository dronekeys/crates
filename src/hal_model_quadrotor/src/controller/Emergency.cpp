#include <hal/model/quadrotor/controller/Emergency.h>

using namespace hal::controller;

bool Emergency::Receive(
    hal_model_quadrotor::Emergency::Request  &req, 
    hal_model_quadrotor::Emergency::Response &res
) {
    return Switch();
}

Emergency::Emergency(const char* name) : Controller<hal_model_quadrotor::State, hal_model_quadrotor::Control,
	hal_model_quadrotor::Emergency::Request, hal_model_quadrotor::Emergency::Response>(name)
{
	Reset();
}

hal_model_quadrotor::Control Emergency::Update(
	const hal_model_quadrotor::State &state, 
	const double &dt
) {
	hal_model_quadrotor::Control control;
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