// For base noise type
#include "Noise.h"

using namespace gazebo;

// Incoming noise message
void Noise::Receive(NosiePtr noise)
{
    // Enable or disable the noise
    enabled = noise->enabled();
}

/// Destructor
Noise::~Noise()
{
    // Do nothing
}

/// Constructor
Noise::Noise() 
{
    // Initialise a node pointer
    nodePtr = transport::NodePtr(new transport::Node());
    nodePtr->Init(modPtr->GetWorld()->GetName());

    // Subscribe to messages about wind conditions
    subPtr = nodePtr->Subscribe("~/noise", &Noise::Receive, this);
};

gazebo::Vector3 Noise::DrawVector(physics::linkPtr link, double dt = 0)
{
    if (enabled)
        return Sample(linkPtr, dt);
    return gazebo::Vector3(0,0,0);
}

double Noise::DrawScalar(physics::linkPtr link, double dt = 0)
{
    if (enabled)
        return Sample(linkPtr, dt);
    return 0;
}