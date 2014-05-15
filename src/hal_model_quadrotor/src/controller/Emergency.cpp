#include <hal/model/controller/Emergency.h>

using namespace hal::model;

bool Emergency::SetGoal(
    hal_model_quadrotor::Emergency::Request  &req, 
    hal_model_quadrotor::Emergency::Response &res
) {
    return true;
}

Emergency::Emergency() : Controller()
{
	Reset();
}

bool Emergency::Update(const hal_model_quadrotor::State &state, 
    double dt, hal_model_quadrotor::Control &control)
{
    return true;
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