#include <hal/model/controller/Hover.h>

using namespace hal::model;

bool Hover::SetGoal(
    hal_model_quadrotor::Hover::Request  &req, 
    hal_model_quadrotor::Hover::Response &res
) {
    // Try and switch control
    return true;
}

Hover::Hover() : Controller()
{
    Reset();
}

bool Hover::Update(const hal_model_quadrotor::State &state, 
    double dt, hal_model_quadrotor::Control &control)
{
    return true;
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