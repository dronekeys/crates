// Standard libraries
#include <hal_quadrotor/Navigation.h>

using namespace hal_quadrotor;

// Constructor
Navigation::Navigation()
{

}

// TODO!
void Navigation::Measurement(const Altitude &msg)
{

}

void Navigation::Measurement(const Inertial &msg)
{

}

void Navigation::Measurement(const Position &msg)
{

}

void Navigation::Measurement(const Magnetic &msg)
{

}

void Navigation::Measurement(const Orientation &msg)
{

}

void Navigation::Measurement(const State &msg)
{
    
}

// Get the latest state estimate
State Navigation::GetState()
{
    return state;
}

// Get the latest state estimate
Orientation Navigation::GetOrientation()
{
    return orientation;
}
