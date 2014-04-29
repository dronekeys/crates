#include <hal/model/quadrotor/controller/Land.h>

using namespace hal::controller;

bool Land::Receive(
    hal_model_quadrotor::Land::Request  &req, 
    hal_model_quadrotor::Land::Response &res
) {
    return Switch();
}

Land::Land(ros::NodeHandle& node, const char *name) : Controller<hal_model_quadrotor::State, hal_model_quadrotor::Control,
	hal_model_quadrotor::Land::Request, hal_model_quadrotor::Land::Response>(node, name)
{
    Reset();
}

hal_model_quadrotor::Control Land::Update(
	const hal_model_quadrotor::State &state, 
	const double &dt
) {
	hal_model_quadrotor::Control control;
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