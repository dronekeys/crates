#include <hal/model/controller/Land.h>

using namespace hal::model;

bool Land::SetGoal(
    hal_model_quadrotor::Land::Request  &req, 
    hal_model_quadrotor::Land::Response &res
) {
    return true;
}

Land::Land()
{
    Reset();
}

bool Land::Update(const hal_model_quadrotor::State &state, 
    double dt, hal_model_quadrotor::Control &control)
{
    return true;
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