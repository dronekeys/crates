#include <hal/model/quadrotor/controller/Idle.h>

using namespace hal::controller;

bool Idle::Receive(
    hal_model_quadrotor::Idle::Request  &req, 
    hal_model_quadrotor::Idle::Response &res
) {
   return Switch();
}

Idle::Idle(ros::NodeHandle& node, const char* name) : Controller<hal_model_quadrotor::State, hal_model_quadrotor::Control,
	hal_model_quadrotor::Idle::Request, hal_model_quadrotor::Idle::Response>(node, name)
{
    Reset();
}

hal_model_quadrotor::Control Idle::Update(
	const hal_model_quadrotor::State &state, 
	const double &dt
) {
	hal_model_quadrotor::Control control;
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