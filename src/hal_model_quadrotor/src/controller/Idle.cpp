#include <hal/model/controller/Idle.h>

using namespace hal::model;

Idle::Idle()
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