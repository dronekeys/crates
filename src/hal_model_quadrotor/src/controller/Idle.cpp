#include <hal/model/controller/Idle.h>

using namespace hal::model;

bool Idle::SetGoal(
    hal_model_quadrotor::Idle::Request  &req, 
    hal_model_quadrotor::Idle::Response &res
) {
   return true;
}

Idle::Idle() : Controller()
{
    Reset();
}

bool Idle::Update(const hal_model_quadrotor::State &state, 
    double dt, hal_model_quadrotor::Control &control)
{
    return true;
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