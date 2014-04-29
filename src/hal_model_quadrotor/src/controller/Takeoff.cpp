#include <hal/model/quadrotor/controller/Takeoff.h>

using namespace hal::controller;

bool Takeoff::Receive(
    hal_model_quadrotor::Takeoff::Request  &req, 
    hal_model_quadrotor::Takeoff::Response &res
) {
    return Switch();
}

Takeoff::Takeoff(ros::NodeHandle& node, const char* name) : Controller<hal_model_quadrotor::State, hal_model_quadrotor::Control,
	hal_model_quadrotor::Takeoff::Request, hal_model_quadrotor::Takeoff::Response>(node, name)
{
    Reset();
}

hal_model_quadrotor::Control Takeoff::Update(
	const hal_model_quadrotor::State &state, 
	const double &dt
) {
	hal_model_quadrotor::Control control;
    return control;
}

// Goal reach implementations
bool Takeoff::HasGoalBeenReached()
{
    return reach;
}

// Reset the current state
void Takeoff::Reset()
{

}
