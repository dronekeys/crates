#include <hal/model/controller/Takeoff.h>

using namespace hal::model;

bool Takeoff::SetGoal(
    hal_model_quadrotor::Takeoff::Request  &req, 
    hal_model_quadrotor::Takeoff::Response &res
) {
    return true;
}

Takeoff::Takeoff() : Controller()
{
    Reset();
}

bool Takeoff::Update(const hal_model_quadrotor::State &state, 
    double dt, hal_model_quadrotor::Control &control)
{
    return true;
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
